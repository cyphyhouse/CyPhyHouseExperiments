#!/usr/bin/env python3

import pickle
from typing import Any

import numpy as np


def main(argv: Any) -> None:
    for pickle_file_io in argv.pickle_file:
        mode_trace_seq = pickle.load(pickle_file_io)
        # TODO compute reachtube from traces
        for mode, trace in mode_trace_seq:
            print("="*80)
            print("Stamped Waypoint: %s" % mode)
            print("Trace Length: %d" % len(trace))
            avg = np.sum(trace[1:, 0] - trace[:-1, 0]) / (len(trace) - 1)
            print("Average State Sampling Period: %f microseconds" % avg)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('pickle_file', nargs='+', type=argparse.FileType('rb'))
    main(parser.parse_args())
