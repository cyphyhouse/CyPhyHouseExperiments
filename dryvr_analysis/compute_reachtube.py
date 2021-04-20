#!/usr/bin/env python3
import os.path
import pickle
from typing import Any, Dict, Iterable, Optional

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle, Ellipse

from dryvr_core import get_discrepancy_parameters, get_reachtube

FONT = {"fontname": "Nimbus Roman", "fontsize": 18}


def plot_z_dim(ax, rtsegment, traces: Optional[Iterable[np.ndarray]] = None, num_states: Optional[int] = None):
    t_dim, z_dim = 0, 3
    if num_states is None or num_states > len(rtsegment):
        num_states = len(rtsegment)
    if traces is None:
        traces = np.empty((0, num_states, z_dim+1))  # zero number of traces
    assert num_states > 0

    for trace in traces:
        ax.plot(trace[:, t_dim], trace[:, z_dim], color='black')

    # Draw the contour of the rectangles using the step function
    t_seq = rtsegment[:num_states, 0, t_dim]
    x_min_seq = rtsegment[:num_states, 0, z_dim]
    x_max_seq = rtsegment[:num_states, 1, z_dim]
    ax.step(t_seq, x_min_seq, where='post', color='gray', alpha=0.7,)
    ax.step(t_seq, x_max_seq, where='post', color='gray', alpha=0.7,)
    ax.fill_between(t_seq, x_min_seq, x_max_seq, step='post', alpha=0.1,
                    color='lightgray')

    ax.set_xlabel("t in sec.", **FONT)
    ax.set_ylabel("z(t) in meters", **FONT)


def plot_xy_dims(ax, rtsegment, traces: Optional[Iterable[np.ndarray]] = None, num_states: Optional[int] = None):
    x_dim, y_dim = 1, 2
    if num_states is None or num_states > len(rtsegment):
        num_states = len(rtsegment)
    if traces is None:
        traces = np.empty((0, num_states, y_dim+1))  # zero number of traces
    assert num_states > 0

    for trace in traces:
        ax.plot(trace[:, x_dim], trace[:, y_dim], color='black')

    for hrect_ind in range(0, num_states, 20):
        x_min, x_max = rtsegment[hrect_ind, 0, x_dim], rtsegment[hrect_ind, 1, x_dim]
        y_min, y_max = rtsegment[hrect_ind, 0, y_dim], rtsegment[hrect_ind, 1, y_dim]
        # ax.add_patch(Ellipse(((x_min + x_max)/2, (y_min + y_max)/2), x_max - x_min, y_max - y_min,  # alpha=0.3,
        #                        color='lightgray', fill=False))
        ax.add_patch(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min,
                               color='lightgray', fill=False))

    ax.grid(True, linestyle='--')
    ax.set_xlabel("x in meters", **FONT)
    ax.set_ylabel("y in meters", **FONT)


def save_plots(out_file_name: str, mode, rtube, training_traces) -> None:
    # Plotting
    fig = plt.figure(figsize=(6, 3))
    ax0 = plt.subplot2grid((1, 2), (0, 0))
    plot_xy_dims(ax0, rtube, training_traces)
    ax0.set_aspect("equal", adjustable='datalim')
    plt.xticks(**FONT)
    plt.yticks(**FONT)
    ax1 = plt.subplot2grid((1, 2), (0, 1))
    plot_z_dim(ax1, rtube, training_traces)
    plt.xticks(**FONT)
    plt.yticks(**FONT)

    if any(isinstance(mode, ty) for ty in [list, tuple, np.ndarray]):
        mode_desc = str(tuple(round(f, 4) if isinstance(f, float) else f for f in mode))
    else:
        mode_desc = str(mode)
    # fig.text(0.5, 0, "Mode: %s" % mode_desc, wrap=True, horizontalalignment='center', **FONT)

    plt.tight_layout()
    plt.savefig(out_file_name + ".pdf", format="pdf")
    plt.close(fig)


def compute_initial_radii(training_traces, center_trace):
    initial_radii = [0.0] * (len(center_trace[0]) - 1)
    for i in range(len(training_traces)):
        trace = training_traces[i]
        for j in range(1, len(initial_radii) + 1):
            initial_radii[j - 1] = max(initial_radii[j - 1], abs(center_trace[0][j] - trace[0][j]))
    return np.array(initial_radii)


def compute_reachtube(training_traces: np.ndarray, center_trace_idx: int = 0):
    if len(training_traces) <= 1:
        raise ValueError("Require at least 2 traces for computing reachtube")

    center_trace = training_traces[center_trace_idx]
    initial_radii = compute_initial_radii(training_traces, center_trace)
    discrepancy_parameters = get_discrepancy_parameters(training_traces, initial_radii)

    result = get_reachtube(center_trace, initial_radii, discrepancy_parameters=discrepancy_parameters)

    # assert 100% training accuracy (all trajectories are contained)
    for trace_ind in range(training_traces.shape[0]):
        if not (np.all(result[:, 0, :] <= training_traces[trace_ind, 1:, :]) and
                np.all(result[:, 1, :] >= training_traces[trace_ind, 1:, :])):
            assert not np.any(np.abs(training_traces[trace_ind, 0, 1:] - center_trace[0, 1:]) > initial_radii)
            print("Warning: Trace #%d" % trace_ind,
                  "of this initial set is sampled outside of the initial set because of floating point error"
                  " and is not contained in the initial set")
    return result


def main(argv: Any) -> None:

    no_ext, *_ = argv.pickle_file.name.split(os.path.extsep)
    out_file_name_base = os.path.basename(no_ext)
    mode_traces_map = pickle.load(argv.pickle_file)  # type: Dict[Any, Iterable[Iterable]]
    # TODO Validate the input pickled data

    mode_rtube_map = {}
    for i, (mode, processed_trace_seq) in enumerate(mode_traces_map.items()):
        if len(processed_trace_seq) <= 1:
            continue
        center_trace = processed_trace_seq[0]
        print("Normalized Trace Length: %d" % len(center_trace))
        assert len(processed_trace_seq[0]) > 0
        print("Aircraft Initial Position of center trace: %s" % (center_trace[0][1:4]))
        print("Aircraft Final Position of center trace: %s" % (center_trace[-1][1:4]))

        training_traces = np.array(processed_trace_seq)
        assert training_traces.ndim == 3
        rtube = compute_reachtube(training_traces, 0)
        mode_rtube_map[mode] = rtube

        if argv.plot:
            file_name = "%s-mode-%d" % (out_file_name_base, i)
            save_plots(file_name, mode, rtube, training_traces)

    if argv.save_rtube:
        with open(out_file_name_base + ".rtube.pickle", "wb") as rtube_pickle_file:
            pickle.dump(mode_rtube_map, rtube_pickle_file)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('pickle_file', type=argparse.FileType('rb'))
    parser.add_argument('--plot', action="store_const", default=True, const=True, dest='plot',
                        help="Do not generate files for plots")
    parser.add_argument('--no-plot', action="store_const", const=False, dest='plot',
                        help="Do not generate files for plots")
    parser.add_argument('-s', '--save-rtube', action="store_true", dest='save_rtube',
                        help="Save a Python dict from each mode to its reachtube in a pickle file")
    main(parser.parse_args())
