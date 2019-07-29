"""Utilility functions for deploying in Intelligent Robot Lab environment"""

from ipaddress import IPv4Address
import paramiko
import socket
from typing import List, NamedTuple


DeviceInfo = NamedTuple(
    'DeviceInfo',
    [
        ('name', str),
        ('addr', IPv4Address),
        ('status', str)
    ]
)


def get_device_list() -> List[DeviceInfo]:
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
        device_list = []
        while True:
            try:
                info, address = sender.recvfrom(buffer_size)
                info = info.decode("utf-8").split(' ')
                device_info = DeviceInfo(name=info[0], addr=address[0], status=info[1])
                device_list.append(device_info)
                print(info[0], " from ", address)
            except socket.timeout:
                break
    print("[INFO] Discover finished")  # TODO logging instead of printing
    return device_list


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
    :param command: command to execute after the file is uploaded
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
