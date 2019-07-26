"""Main entry point for deployment or simulation"""

from .konfig import Konfig


def main(argv) -> None:
    # TODO Parse arguments better
    config_filename = argv[1]
    device_name = argv[2]
    out_filename = argv[3]

    with open(config_filename, 'r') as f:
        conf = Konfig(f)
    print(conf.gen_all_local_configs())
    with open(out_filename, 'w', encoding='utf8') as out_file:
        conf.dump_local_config(device_name, out_file)

    # TODO Run simulation or deployment scripts


if __name__ == "__main__":
    import sys
    main(sys.argv)
