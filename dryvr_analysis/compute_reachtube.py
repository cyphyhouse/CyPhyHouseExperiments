#!/usr/bin/env python3

import pickle
from typing import Any, Sequence, Optional

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator
from matplotlib.patches import Rectangle

from reach_compute import get_discrepancy_parameters, get_reachtube


def rosplane_trace_filter(stamped_mode: np.ndarray, trace: np.ndarray) -> bool:
    # CENTER = np.array([200, 200, -50.0])  # Right Turn to reach (200, 200)
    # CENTER = np.array([-10.0, 40.0, -50.0])  # Straight Line from (200, 200) -> (0, 0)
    CENTER = np.array([10.0, 0.0, -50.0])  # Left Turn to reach (0, 0)
    MARGIN = np.array([5.0, 5.0, 5.0])
    final_pos = trace[-1][1:4]
    mode = stamped_mode[1:]

    if 450 <= len(trace) \
            and np.all((CENTER - MARGIN) <= final_pos) \
            and np.all(final_pos <= (CENTER + MARGIN)):
        return True
    return False


def hectorquad_trace_filter(stamped_mode: np.ndarray, trace: np.ndarray) -> bool:
    DESIRED_MODE = np.array([0.0, 0.0, 2.5])

    return np.all(stamped_mode[1:4] == DESIRED_MODE)


def preprocessing(mode_trace_seq, trace_predicate, max_trace_len: int = 2000) -> Sequence[np.ndarray]:
    filtered_seq = [trace for stamped_mode, trace in mode_trace_seq if trace_predicate(stamped_mode, trace)]
    if not filtered_seq:
        return ()
    trace_len = min(max_trace_len, min(len(trace) for trace in filtered_seq))
    time_step = 0.01  # 0.000001  # Reassign timestamps assuming each state is sampled perfectly periodically
    for i, trace in enumerate(filtered_seq):
        temp = trace[-trace_len:]  # Choose the suffix
        filtered_seq[i] = np.array([np.array([j * time_step] + temp[j][1:].tolist()) for j in range(0, len(temp))])
    return filtered_seq


def plot_z_dim(rtsegment, traces, num_states: Optional[int] = None):
    dim_ind = 3
    if num_states is None or num_states > len(rtsegment):
        num_states = len(rtsegment)
    assert num_states > 0

    fig, ax = plt.subplots(1)
    for trace_ind in range(traces.shape[0]):
        ax.plot(traces[trace_ind, :num_states, 0], traces[trace_ind, :num_states, dim_ind], color='black')

    prev_t_max = 0.0

    # Draw the contour of the rectangles using the step function
    t_seq = rtsegment[:num_states, 0, 0]
    x_min_seq = rtsegment[:num_states, 0, dim_ind]
    x_max_seq = rtsegment[:num_states, 1, dim_ind]
    ax.step(t_seq, x_min_seq, where='post', color='gray', alpha=0.7,)
    ax.step(t_seq, x_max_seq, where='post', color='gray', alpha=0.7,)
    ax.fill_between(t_seq, x_min_seq, x_max_seq, step='post', alpha=0.1,
                    color='lightgray')

    ax.set_xlabel("t in sec.")
    ax.set_ylabel("z(t) in meters")
    ax.autoscale_view()
    plt.show()


def plot_xy_dims(rtsegment, traces, num_states: Optional[int] = None):
    x_dim, y_dim = 1, 2
    if num_states is None or num_states > len(rtsegment):
        num_states = len(rtsegment)
    assert num_states > 0

    fig, ax = plt.subplots(1)
    for trace_ind in range(traces.shape[0]):
        ax.plot(traces[trace_ind, :num_states, x_dim], traces[trace_ind, :num_states, y_dim], color='black')

    # Draw the contour of the rectangles using the step function
    prev_t_max = 0.0
    for hrect_ind in range(0, num_states, num_states//80):
        x_min, x_max = rtsegment[hrect_ind, 0, x_dim], rtsegment[hrect_ind, 1, x_dim]
        y_min, y_max = rtsegment[hrect_ind, 0, y_dim], rtsegment[hrect_ind, 1, y_dim]
        ax.add_patch(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, alpha=0.3,
                               color='lightgray', fill=False))

    ax.grid(True, linestyle='--')
    ax.autoscale_view()
    ax.set_aspect("equal")
    ax.set_xlabel("x in meters")
    ax.set_ylabel("y in meters")

    major = MultipleLocator(50)
    minor = MultipleLocator(10)
    ax.xaxis.set_major_locator(major)
    ax.xaxis.set_minor_locator(minor)
    ax.yaxis.set_major_locator(major)
    ax.yaxis.set_minor_locator(minor)
    plt.show()


def compute_initial_radii(training_traces, center_trace):
    initial_radii = [0.0] * (len(center_trace[0]) - 1)
    for i in range(len(training_traces)):
        trace = training_traces[i]
        for j in range(1, len(initial_radii) + 1):
            initial_radii[j - 1] = max(initial_radii[j - 1], abs(center_trace[0][j] - trace[0][j]))
    return np.array(initial_radii)


def compute_reachtube(processed_trace_seq):
    training_traces = np.array(processed_trace_seq)

    # NOTE The first trace is used as the center trace for no reason. Can be other traces.
    center_trace = training_traces[0]
    initial_radii = compute_initial_radii(training_traces, center_trace)
    discrepancy_parameters = get_discrepancy_parameters(training_traces, initial_radii)

    result = get_reachtube(center_trace, initial_radii, discrepancy_parameters=discrepancy_parameters)

    # assert 100% training accuracy (all trajectories are contained)
    for trace_ind in range(training_traces.shape[0]):
        if not (np.all(result[:, 0, :] <= training_traces[trace_ind, 1:, :]) and
                np.all(result[:, 1, :] >= training_traces[trace_ind, 1:, :])):
            assert np.any(np.abs(training_traces[trace_ind, 0, 1:] - center_trace[0, 1:]) > initial_radii)
            print("Warning: Trace #%d" % trace_ind,
                  "of this initial set is sampled outside of the initial set because of floating point error"
                  " and is not contained in the initial set")

    plot_z_dim(result, training_traces)
    plot_xy_dims(result, training_traces)


def main(argv: Any) -> None:
    for pickle_file_io in argv.pickle_file:
        mode_trace_seq = pickle.load(pickle_file_io)

        # TODO Need to switch the filter to process traces from different modes in aircraft models
        processed_trace_seq = preprocessing(mode_trace_seq, hectorquad_trace_filter)

        if not processed_trace_seq:
            print("No trace is suitable for computing reachtube after preprocessing. Abort.")
            return

        print("Normalized Trace Length: %d" % len(processed_trace_seq[0]))
        assert len(processed_trace_seq[0]) > 0
        print("Aircraft Initial Position: %s" % (processed_trace_seq[0][0][1:4]))
        print("Aircraft Final Position: %s" % (processed_trace_seq[0][-1][1:4]))
        compute_reachtube(processed_trace_seq)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('pickle_file', nargs='+', type=argparse.FileType('rb'))
    main(parser.parse_args())
