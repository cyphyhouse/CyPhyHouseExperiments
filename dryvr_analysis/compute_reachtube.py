import pickle
from typing import Any


def main(argv: Any) -> None:
    for pickle_file_io in argv.pickle_file:
        mode_trace_seq = pickle.load(pickle_file_io)
        for mode, trace in mode_trace_seq:
            print("="*80)
            print("Waypoint: %s" % mode)
            print("Trace: %s" % trace)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('pickle_file', nargs='+', type=argparse.FileType('rb'))
    main(parser.parse_args())
