#!/usr/bin/env python

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
        -> Dict[str, Dict[str, Any]]:
    assert validate_global_config(cfg)

    device_map = cfg['devices']
    ret = {}
    for agent in cfg['agents']:
        bot_name = agent['on_device']
        assert bot_name in device_map
        local_cfg = cfg.copy()
        local_cfg['num_agents'] = len(cfg['agents'])
        del local_cfg['agents']
        local_cfg['agent'] = agent
        del local_cfg['devices']
        assert 'bot_name' not in device_map[bot_name]
        device = {'bot_name': bot_name}
        device.update(device_map[bot_name])
        local_cfg['device'] = device

        ret[bot_name] = local_cfg

    return ret


def main(argv) -> None:
    # This main function is for testing internally. Use it carefully.
    # TODO Parse arguments better
    config_filename = argv[1]
    device_name = argv[2]
    out_filename = argv[3]

    with open(config_filename, 'r') as f:
        global_cfg = yaml.safe_load(f)
        if not validate_global_config(global_cfg):
            raise ValueError("Invalid YAML file: " + config_filename)

    local_cfg_map = gen_all_local_configs(global_cfg)
    local_cfg = local_cfg_map[device_name]
    with open(out_filename, 'w', encoding='utf8') as out_file:
        yaml.dump(local_cfg, out_file, allow_unicode=True)


if __name__ == "__main__":
    import sys
    main(sys.argv)
