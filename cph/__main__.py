"""Main entry point for deployment or simulation"""

import yaml

from .konfig import Konfig
from . import deploy_irl


def main(argv) -> None:
    # TODO Parse arguments better
    task_krd = argv[1]
    global_cfg_filename = argv[2]
    local_cfg_filename = '.local.yml'

    # TODO Call Koord Compiler
    task_py = task_krd

    with open(global_cfg_filename, 'r') as f:
        conf = Konfig(f)


    # TODO Run simulation or deployment scripts
    if True:
        device_map = deploy_irl.get_device_map()

        print("Discovered devices:\n" + '\n'.join([str(d) for d in device_map]))  # TODO Logging

        # TODO Use device list to generate/update global config file

        for local_cfg in conf.gen_all_local_configs():
            device = local_cfg['device']
            with open(device['bot_name'] + local_cfg_filename, 'w', encoding='utf8') as out_file:
                yaml.dump(device, out_file, default_flow_style=True)

            if device['bot_name'] not in device_map:
                print("[WARNING] Device \"" + device['bot_name'] + "\" is not available. " + \
                      "Agents on this device are not deployed.")
                continue

            # TODO Put task.py and local config to right places
            deploy_irl.upload_and_exec(
                device_addr=device['ip'],
                username=device['username'],
                password=device['password'],
                local_path="flydrone.bash",
                remote_path="./flydrone.bash")


if __name__ == "__main__":
    import sys
    main(sys.argv)
