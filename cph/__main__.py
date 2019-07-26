"""Main entry point for deployment or simulation"""

from .konfig import Konfig
from . import deploy_irl


def main(argv) -> None:
    # TODO Parse arguments better
    task_krd = argv[1]
    global_cfg_filename = argv[2]
    local_cfg_filename = 'local.yml'

    # TODO Call Koord Compiler
    task_py = task_krd

    with open(global_cfg_filename, 'r') as f:
        conf = Konfig(f)

    # TODO Run simulation or deployment scripts
    if True:
        device_list = deploy_irl.get_device_list()

        # TODO Use device list to generate/update global config file

        for device_info in device_list:
            with open(local_cfg_filename, 'w', encoding='utf8') as out_file:
                conf.dump_local_config(device_info.name, out_file)

            # TODO Put task.py and local config to right places

            deploy_irl.upload_and_exec(
                device_addr=device_info.addr,
                username='pi',   # TODO: read username and password from somewhere
                password='cyphyhouse',
                local_path="./flydrone.bash",
                remote_path="./flydrone.bash",
                command="bash ./flydrone.bash")


if __name__ == "__main__":
    import sys
    main(sys.argv)
