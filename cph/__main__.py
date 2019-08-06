"""Main entry point for deployment or simulation"""

import base64
from typing import Any, Dict
import yaml

from .konfig import Konfig
from . import deploy_irl


def dump_start_script(app_py_path: str,
                      local_cfg: Dict[str, Any],
                      script_path: str) -> None:
    """
    Dump the start script with generated Python code and config files encoded with base64 strings and decoded
    :param app_py_path: Path to Koord compiled code
    :param local_cfg: Agent specific local config generated from global config
    :param script_path: Path where the script file will be dumped
    :return: None
    """
    # FIXME This is a temporary solution for sending files.
    #  There may be a limit on how long the encoded string can be.
    tmp_app_name = b"/tmp/app.py"  # FIXME Decide where to put temporary files
    tmp_cfg_name = b"/tmp/local.yml"

    with open(app_py_path, 'rb') as in_file:
        b64_app_py = base64.b64encode(in_file.read())

    local_yml = yaml.dump(local_cfg, encoding='utf-8', default_flow_style=True)
    b64_local_yml = base64.b64encode(local_yml)

    with open(script_path, 'wb', encoding='utf8') as out_file:
        out_file.write(b"echo " + b64_app_py + b" | base64 --decode > "
                       + tmp_app_name + b"\n")
        out_file.write(b"echo " + b64_local_yml + b" | base64 --decode > "
                       + tmp_cfg_name + b"\n")
        # TODO Bash commands for launching ROS

        out_file.write(b"sleep 10s\n")  # FIXME Wait for ROS/hardware to initialize

        cmd_list = [b"python3", b"-m", b"src.scripts.run",
                    b"--app", tmp_app_name,
                    b"--config", tmp_cfg_name]
        out_file.write(b" ".join(cmd_list))


def main(argv) -> None:
    # TODO Parse arguments better
    app_krd = argv[1]
    global_cfg_filename = argv[2]

    # TODO Call Koord Compiler
    app_py = app_krd

    with open(global_cfg_filename, 'r') as f:
        conf = Konfig(f)

    # TODO Run simulation or deployment scripts
    if True:
        device_map = deploy_irl.get_device_map()

        print("Discovered devices:\n" + '\n'.join([str(d) for d in device_map]))  # TODO Logging

        # TODO Use device list to generate/update global config file

        for local_cfg in conf.gen_all_local_configs():
            device = local_cfg['device']

            if device['bot_name'] not in device_map:
                print("[WARNING] Device \"" + device['bot_name'] + "\" is not available. " + \
                      "Agents on this device are not deployed.")
                continue

            # FIXME Use library to handle filesystem paths
            start_bash_path = "start_" + device['bot_name'] + ".bash"
            dump_start_script(app_py, local_cfg, start_bash_path)

            deploy_irl.upload_and_exec(
                device_addr=device['ip'],
                username=device['username'],
                password=device['password'],
                local_path=start_bash_path,
                remote_path=start_bash_path)


if __name__ == "__main__":
    import sys
    main(sys.argv)
