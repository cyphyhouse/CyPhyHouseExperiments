#!/usr/bin/env python3

import pickle
from typing import Any, Sequence

import numpy as np

DESIRED_MODE = np.array([0.0, 0.0, 2.5])


def preprocessing(mode_trace_seq, desired_mode, max_trace_len: int = 1000) -> Sequence[np.ndarray]:
    filtered_seq = [trace for stamped_mode, trace in mode_trace_seq if np.all(stamped_mode[1:] == desired_mode)]
    trace_len = min(max_trace_len, min(len(trace) for trace in filtered_seq))

    for i, trace in enumerate(filtered_seq):
        filtered_seq[i] = trace[-trace_len:]  # Choose the suffix
    return filtered_seq


def main(argv: Any) -> None:
    for pickle_file_io in argv.pickle_file:
        mode_trace_seq = pickle.load(pickle_file_io)
        # TODO compute reachtube from traces
        processed_trace_seq = preprocessing(mode_trace_seq, DESIRED_MODE)
        print("Number of traces: %d" % len(processed_trace_seq))
        for trace in processed_trace_seq:
            print("="*80)
            print("Trace Length: %d" % len(trace))
            final_stamped_state = trace[-1]
            print("Final Position: %s" % str(final_stamped_state[1:4]))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('pickle_file', nargs='+', type=argparse.FileType('rb'))
    main(parser.parse_args())
