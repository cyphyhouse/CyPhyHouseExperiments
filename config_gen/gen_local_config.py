#!/usr/bin/env python

from ipaddress import ip_address, IPv4Address
from typing import Any, Dict, Tuple
import yaml


def validate_global_config(cfg: Dict[str, Any]) -> bool:
    # TODO validate yaml file according to a schema
    # Some items to check
    # + Type for each field, e.g., ip, port, bot_type, etc.
    # + 'on_device' for an agent must appear in 'devices'
    # + No 'bot_name' field for all devices
    # + 'motion_automaton' for an agent must be in 'motion_automata' of the assigned device
    return True


def gen_all_local_configs(cfg: Dict[str, Any]) \
        -> Dict[Tuple[IPv4Address, int], Dict[str, Any]]:
    assert validate_global_config(cfg)

    device_map = cfg['devices']
    ret = {}
    for agent in cfg['agents']:
        bot_name = agent['on_device']
        assert bot_name in device_map
        local_cfg = cfg.copy()
        del local_cfg['agents']
        local_cfg['agent'] = agent
        del local_cfg['devices']
        assert 'bot_name' not in device_map[bot_name]
        device = {'bot_name': bot_name}
        device.update(device_map[bot_name])
        local_cfg['device'] = device

        endpoint = (ip_address(device['ip']), int(device['port']))
        ret[endpoint] = local_cfg

    return ret


def main(argv) -> None:
    # This main function is for testing internally. Use it carefully.
    # TODO Parse arguments better
    config_filename = argv[1]
    endpoint = argv[2].split(':', 1)
    out_filename = argv[3]

    ip, port = ip_address(endpoint[0]), int(endpoint[1])

    with open(config_filename, 'r') as f:
        global_cfg = yaml.safe_load(f)
        if not validate_global_config(global_cfg):
            raise ValueError("Invalid YAML file: " + config_filename)

    local_cfg_map = gen_all_local_configs(global_cfg)
    local_cfg = local_cfg_map[(ip, port)]
    with open(out_filename, 'w', encoding='utf8') as out_file:
        yaml.dump(local_cfg, out_file, allow_unicode=True)


if __name__ == "__main__":
    import sys
    main(sys.argv)
