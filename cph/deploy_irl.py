"""Utilility functions for deploying in Intelligent Robot Lab environment"""

import base64
from ipaddress import IPv4Address
import paramiko
import socket
from typing import Any, Dict, NamedTuple
import yaml

DeviceInfo = NamedTuple(
    'DeviceInfo',
    [
        ('name', str),
        ('addr', IPv4Address),
        ('status', str)
    ]
)


def get_device_map() -> Dict[str, DeviceInfo]:
    """
    The function broadcasts to LAN and wait for responses from devices to compile a list.
    :return: The list of discovered devices
    """
    buffer_size = 1024
    client_port = 60651
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sender:
        sender.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sender.sendto(b'INFO', ('<broadcast>', client_port))
        print("[INFO] Device query sent!")  # TODO logging instead of printing

        sender.settimeout(3.0)  # Set timeout to avoid waiting forever
        device_map = {}
        while True:
            try:
                info, address = sender.recvfrom(buffer_size)
                info = info.decode("utf-8").split(' ')
                device_info = DeviceInfo(name=info[0], addr=address[0], status=info[1])
                device_map[device_info.name] = device_info
                print("[INFO]", info[0], " from ", address)
            except socket.timeout:
                break
    print("[INFO] Discover finished")  # TODO logging instead of printing
    return device_map


def upload_and_exec(device_addr: IPv4Address,
                    local_path: str,  # TODO use Path instead of str?
                    remote_path: str,
                    username: str = None,
                    password: str = None,
                    ) -> None:
    """
    Upload a file and execute the file as a bash script thru SSH
    :param device_addr: IP address to upload to
    :param local_path: The local path of file to upload
    :param remote_path: The remote **path of file** after upload
    :param username: User name for login
    :param password: Password for login
    :return:
    """

    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh_client.connect(hostname=str(device_addr),
                       username=username, password=password)

    print("[INFO] Upload file to: ", device_addr)  # TODO logging instead of printing
    sftp_client = ssh_client.open_sftp()
    sftp_client.put(local_path, remote_path)
    sftp_client.close()

    command = ' '.join(["bash", remote_path])
    print("[INFO] Sending command \"" + command + "\" to ", device_addr)  # TODO logging instead of printing
    stdin, stdout, stderr = ssh_client.exec_command(command)

    ssh_client.close()


def dump_start_script(app_py_path: str,
                      local_cfg: Dict[str, Any],
                      script_path: str) -> None:
    """
    Dump the start script with generated Python code and config files encoded
     as base64 strings and decoded on remote machine in generated Bash script
    :param app_py_path: Path to Koord compiled code
    :param local_cfg: Agent specific local config generated from global config
    :param script_path: Path where the script file will be dumped
    :return: None
    """
    # FIXME This is a temporary solution for sending files.
    #  There may be a limit on how long the encoded string can be.
    tmp_app_path = b"/tmp/app.py"  # FIXME Decide where to put temporary files
    tmp_cfg_path = b"/tmp/local.yml"

    with open(app_py_path, 'rb') as in_file:
        b64_app_py = base64.b64encode(in_file.read())

    local_yml = yaml.dump(local_cfg, encoding='utf-8', default_flow_style=True)
    b64_local_yml = base64.b64encode(local_yml)

    with open(script_path, 'wb') as out_file:
        out_file.write(b"echo " + b64_app_py + b" | base64 --decode > "
                       + tmp_app_path + b"\n")
        out_file.write(b"echo " + b64_local_yml + b" | base64 --decode > "
                       + tmp_cfg_path + b"\n")
        # TODO Bash commands for launching ROS

        out_file.write(b"sleep 10s\n")  # FIXME Wait for ROS/hardware to initialize

        cmd_list = [b"python3", b"-m", b"src.scripts.run",
                    b"--app", tmp_app_path,
                    b"--config", tmp_cfg_path]
        out_file.write(b" ".join(cmd_list))
