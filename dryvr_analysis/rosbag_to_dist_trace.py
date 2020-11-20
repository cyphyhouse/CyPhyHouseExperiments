import math
from sys import float_info
from typing import NamedTuple, Any, List, Mapping, Iterable, Tuple, Dict, Iterator, Generator, Callable, TypeVar

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


def process_bag_file(filename: str,
                     topic_to_cb: Mapping[str, Callable[[ROSMsgT], DataPoint]]) \
        -> Tuple[str, DistTrace]:
    """
    Process messages from bag files

    Notes
    -----
    Only the timestamp of messages from the same topic is guaranteed to be monotonically increasing.
    Be careful when merging messages from different topics.
    """
    ret_dist_trace = {topic: [] for topic in topic_to_cb.keys()}  # type: Dict[str, List[StampedDataPoint]]
    with Bag(filename) as bag:
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

    return filename, ret_dist_trace


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
