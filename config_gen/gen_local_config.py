#!/usr/bin/env python

from typing import Any, Dict, Optional
import yaml


def __validate(cfg) -> bool:
    # TODO validate yaml file according to a schema
    return True


def gen_local_config(
                cfg: Dict[str, Any],
                agent_id: Optional[int] = None,
                device_id: Optional[int] = None,
                ) -> Dict[str, Any]:
    if not __validate(cfg):
        raise ValueError("Invalid YAML file" + config_filename)

    local_cfg = cfg.copy()

    if agent_id is not None:
        # Make AgentConfig
        try:
             agents = local_cfg.pop('agents')
             import pdb
             pdb.set_trace()
             local_cfg['agent'] = agents[agent_id]
        except IndexError:
            raise IndexError("Agent index out of bound")


    if device_id is not None:
        # Make MoatConfig
        try:
             devices = local_cfg.pop('devices')
             local_cfg['device'] = devices[device_id]
        except IndexError:
            raise IndexError("Device index out of bound")

    return local_cfg


if __name__ == "__main__":
    import sys
    # TODO Parse arguments better
    config_filename = sys.argv[1]
    agent_id = int(sys.argv[2])
    device_id = int(sys.argv[3])
    out_filename = "config.yml"

    with open(config_filename, 'r') as f:
        global_cfg = yaml.safe_load(f)
    local_cfg = gen_local_config(global_cfg, agent_id, device_id)
    with open(out_filename, 'w', encoding='utf8') as out_file:
        yaml.dump(local_cfg, out_file, allow_unicode=True) 
