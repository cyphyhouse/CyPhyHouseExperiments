import math
from collections import defaultdict
from sys import float_info
from typing import NamedTuple, Any, List, Mapping, Iterable, Tuple, Dict, Iterator, Generator, Callable, TypeVar, \
    Sequence, Hashable, Union, BinaryIO

from genpy.message import Message
from rosbag import Bag


ROSMsgT = TypeVar('ROSMsgT', bound=Message, covariant=True)
DataPoint = Any
StampedDataPoint = NamedTuple('StampedDataPoint', [('stamp', float), ('dp', DataPoint)])

DistTrace = Mapping[str, Iterable[StampedDataPoint]]
Observation = Mapping[str, DataPoint]

StateT = Tuple[float, ...]

MAX_PRECISE_INT = float_info.radix ** float_info.mant_dig
DATAPOINT_END = StampedDataPoint(math.inf, None)


def process_bag_file(file: Union[str, BinaryIO],
                     topic_to_cb: Mapping[str, Callable[[ROSMsgT], DataPoint]]) \
        -> DistTrace:
    """
    Process messages from bag files

    Notes
    -----
    Only the timestamp of messages from the same topic is guaranteed to be monotonically increasing.
    Be careful when merging messages from different topics.
    """
    ret_dist_trace = {topic: [] for topic in topic_to_cb.keys()}  # type: Dict[str, List[StampedDataPoint]]
    with Bag(file) as bag:
        for topic, msg, t in bag.read_messages(topics=topic_to_cb.keys()):
            if hasattr(msg, "header"):
                t = msg.header.stamp  # Use sender's timestamp if available
            t_nsec = t.to_nsec()  # type: int
            quo, rem = divmod(t_nsec, (5 ** 6))
            if t_nsec > MAX_PRECISE_INT or rem != 0:
                raise RuntimeWarning("Timestamp %s nano-sec may lose precision when converted to milli-sec"
                                     "in floating point." % str(t))
            t_msec_f = float(quo) / (2 ** 6)

            if topic in topic_to_cb:
                dp = topic_to_cb[topic](msg)
                ret_dist_trace[topic].append(StampedDataPoint(t_msec_f, dp))
            else:
                continue  # Ignore other topics

    return ret_dist_trace


def _advance_pass_stamp(trace_iter_dict: Dict[str, Tuple[StampedDataPoint, Iterator[StampedDataPoint]]],
                        curr_dist_obs: Dict[str, Any], curr_stamp: float) -> None:
    for topic, (head, tail_iter) in trace_iter_dict.items():
        if head.stamp > curr_stamp:
            # Not updating data point for this topic
            # NOTE: this also makes the topic uses the last data point after reaching the end.
            continue
        # else:
        curr_dp = head.dp
        while head.stamp <= curr_stamp:  # Also skip repeating timestamp
            curr_dp = head.dp  # If timestamp repeats, this will use the last data point
            head = next(tail_iter, DATAPOINT_END)
        trace_iter_dict[topic] = (head, tail_iter)
        # Update the topic in current distributed observation with new data point
        assert curr_dp is not None
        curr_dist_obs[topic] = curr_dp

    assert all(curr_stamp < stamp for (stamp, _), _ in trace_iter_dict.values())


def gen_stamped_state(dist_trace: DistTrace,
                      observation_to_user_state: Callable[[float, Observation], StateT],
                      skip_init: bool = False) -> Generator[StateT, None, None]:
    """ A generator to return observed global states at all timestamps from a distributed trace.

    If there is no data point for a topic at a certain timestamp, the previous data point of that topic is used.
    If no previous data point for a topic is available, e.g., not initialized yet,
    Basically, similar to a zero order hold but works for possibly infinite trace.

    Parameters
    ----------
    dist_trace : DistTrace
        A distributed trace which is a mapping from a topic to its time series of data points
    observation_to_user_state
        User provided function to convert observed data points of all topics at a timestamp to the user defined state.
    skip_init : bool
        Skip initialization phase and start from the first timestamp where all topics have data points.

    Notes
    -----
    TODO should the observation be a consistent global snapshot?
    """
    trace_iter_dict = {}
    for topic, trace in dist_trace.items():
        tail_iter = iter(trace)
        head = next(tail_iter, DATAPOINT_END)
        trace_iter_dict[topic] = (head, tail_iter)

    curr_dist_obs = {}  # type: Dict[str, Any]
    if not skip_init:
        init_stamp = -math.inf
    else:
        init_stamp = max(stamp for (stamp, _), _ in trace_iter_dict.values())
        if init_stamp == math.inf:
            return  # Early return because there is a topic without any data point
        _advance_pass_stamp(trace_iter_dict, curr_dist_obs, init_stamp)

        assert set(curr_dist_obs.keys()) == set(trace_iter_dict.keys())
        yield observation_to_user_state(init_stamp, curr_dist_obs)

    prev_stamp = init_stamp
    while any(stamp != math.inf for (stamp, _), _ in trace_iter_dict.values()):
        curr_stamp = min(stamp for (stamp, _), _ in trace_iter_dict.values())
        assert math.isfinite(curr_stamp), "Current stamp must not be -inf, inf, or NaN"
        assert prev_stamp < curr_stamp, "Stamp must be strictly increasing"

        _advance_pass_stamp(trace_iter_dict, curr_dist_obs, curr_stamp)
        yield observation_to_user_state(curr_stamp, curr_dist_obs)
        prev_stamp = curr_stamp


_StampedStateT = Tuple[float, ...]
_SegmentT = Sequence[_StampedStateT]


def _split_dist_trace(stamped_mode_iseq: Iterable[StampedDataPoint],
                      stamped_state_iseq: Iterable[StampedDataPoint]) \
        -> Generator[Tuple[Hashable, _SegmentT], None, None]:
    """ Cut the original trace into segments based on the timestamps of modes.
    The segment before the first mode is uninitialized.
    The segment after the last waypoint is excluded to remove disturbance due to termination.
    """
    stamped_mode_iter = iter(stamped_mode_iseq)
    stamped_state_iter = iter(stamped_state_iseq)

    prev_stamp, prev_mode = -math.inf, ()
    try:
        stamp, state = next(stamped_state_iter)
        for wp_stamp, wp in stamped_mode_iter:
            segment = []
            while stamp < wp_stamp:
                segment.append((stamp,) + tuple(state))
                stamp, state = next(stamped_state_iter)
            # NOTE: Last mode is automatically excluded due to this line using previous waypoints
            curr_mode = (prev_stamp,) + tuple(prev_mode)
            yield curr_mode, segment
            prev_stamp, prev_mode = wp_stamp, wp
    except StopIteration:  # Exhaust all states
        raise ValueError("No state available in state_topic")


def _concatenate_same_mode(mode_trace_iseq: Iterable[Tuple[Hashable, _SegmentT]]) \
        -> Generator[Tuple[Hashable, _SegmentT], None, None]:
    """Consecutive segments with the same mode are concatenated as one trace."""
    seg_iter = iter(mode_trace_iseq)
    prev_mode, prev_trace = next(seg_iter)
    prev_trace_as_list = list(prev_trace)
    for mode, trace in seg_iter:
        if mode[1:] == prev_mode[1:]:
            prev_trace_as_list.extend(trace)
        else:
            yield prev_mode, prev_trace_as_list
            prev_mode = mode
            prev_trace_as_list = list(trace)

    yield prev_mode, prev_trace_as_list


def dist_trace_to_mode_seg_tuples(dist_trace: DistTrace, mode_topic: str, state_topic: str) \
        -> Sequence[Tuple[Hashable, _SegmentT]]:
    """
    Convert distributed traces to a sequence of tuples of a mode and a trace segment
    Parameters
    ----------
    dist_trace
        Mapping from a topic to a recorded trace of the topic
    mode_topic
        The topic whose timestamps are used to split traces
    state_topic
        The topic whose trace is split into segments
    Returns
    -------
    mode_seg_tuples
        A sequence of tuples of a mode and the trace segment following that mode.
    """

    seg_iter = _split_dist_trace(dist_trace[mode_topic], dist_trace[state_topic])
    seg_iter = _concatenate_same_mode(seg_iter)
    uninit_mode, _ = next(seg_iter)  # Ignore the first segment because its mode is uninitialized
    assert uninit_mode == (-math.inf,)
    return list(seg_iter)


def aggregate_by_mode(mode_seg_iseq: Iterable[Tuple[Hashable, _SegmentT]])\
        -> Mapping[Hashable, List[_SegmentT]]:
    mode_seg_iter = iter(mode_seg_iseq)
    aggregated_dict = defaultdict(list)  # type: Dict[Hashable, List[_SegmentT]]
    for stamped_mode, seg in mode_seg_iter:
        mode = tuple(stamped_mode[1:])  # remove timestamp as dictionary key
        aggregated_dict[mode].append(seg)

    return aggregated_dict
