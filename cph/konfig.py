#!/usr/bin/env python

from typing import Any, Dict, List, TextIO, Tuple
import yaml


class Konfig:
    def __init__(self, cfg_file: TextIO):
        self.__global_cfg = yaml.safe_load(cfg_file)
        self.validate_global_config()

    @property
    def udp_bcast_addr(self) -> Tuple[str, int]:
        return self.__global_cfg['udp_bcast_ip'], self.__global_cfg['udp_port']

    def gen_all_agent_addrs(self) -> List[Tuple[str, int]]:
        bot_name_iter = (a['on_device'] for a in self.__global_cfg['agents'])
        ip_iter = (self.__global_cfg['devices'][bot]['ip'] for bot in bot_name_iter)
        return [(ip, self.__global_cfg['udp_port']) for ip in ip_iter]

    def validate_global_config(self) -> bool:
        # TODO validate yaml file according to konfig.schema.yml using yamale
        # Some items to check
        # + Type for each field, e.g., ip, port, bot_type, etc.
        # + 'on_device' for an agent must appear in 'devices'
        # + No 'bot_name' field for all devices
        # + 'motion_automaton' for an agent must be in 'motion_automata' of the assigned device

        cfg = self.__global_cfg
        if len(cfg['agents']) > len(cfg['devices']):
            # TODO Logging
            print("[ERROR] Specifed more agents (" + len(cfg['agents']) + \
                  ") than available devices (" + len(cfg['devices'] + ")"))
            return False

        return self.__global_cfg is not None

    def gen_all_local_configs(self) \
            -> List[Dict[str, Any]]:
        assert self.validate_global_config()

        cfg = self.__global_cfg
        device_map = cfg['devices']
        ret = []
        for agent in cfg['agents']:
            bot_name = agent['on_device']
            assert bot_name in device_map
            local_cfg = cfg.copy()  # Shallow copy all top level entries
            local_cfg['num_agents'] = len(cfg['agents'])
            del local_cfg['agents']
            local_cfg['agent'] = agent
            del local_cfg['devices']
            assert 'bot_name' not in device_map[bot_name]
            device = device_map[bot_name].copy()  # Shallow copy
            device['bot_name'] = bot_name
            device['port'] = cfg['udp_port']
            if 'motion' in device:
                del device['motion']
                device.update(device_map[bot_name]['motion'])
            local_cfg['device'] = device

            ret.append(local_cfg)

        return ret

