import argparse
import datetime
from multiprocessing import Event, Process, Queue
from queue import Empty
from typing import List, Sequence, Tuple, Dict, Any
import yaml

from dist_mutex_contr.airspace_manager import AirspaceManager
from dist_mutex_contr.agent import Agent
from dist_mutex_contr.motion import build_motion_controller, MotionInitInfo
from dist_mutex_contr.tioa_base import run_as_process
import eceb_scenarios
import city_scenarios
import ACAS_scenarios

import rospy

def _multicast(mgr_queue_pair, agent_queue_list: List[Tuple[Queue, Queue]]) -> Sequence[bytes]:
    act_list = []
    mgr_i_q, mgr_o_q = mgr_queue_pair
    # TODO deliver messages only to automatons using the actions
    for _, o_q in agent_queue_list:
        try:
            act = o_q.get_nowait()
            act_list.append(act)
            mgr_i_q.put_nowait(act)
        except Empty:
            continue

    try:
        act = mgr_o_q.get_nowait()
        act_list.append(act)
        for i_q, _ in agent_queue_list:
            i_q.put_nowait(act)
    except Empty:
        pass
    return act_list


def test_protocol(scenario: Dict[str, Tuple[MotionInitInfo, List]]) -> None:
    print(scenario)
    for uid, (init_info, wps) in scenario.items():
        print(uid)
    stop_ev = Event()

    air_mgr = AirspaceManager()
    air_mgr_i_queue, air_mgr_o_queue = Queue(), Queue()
    air_mgr_proc = Process(target=run_as_process,
                           kwargs={"aut": air_mgr,
                                   "i_queue": air_mgr_i_queue,
                                   "o_queue": air_mgr_o_queue,
                                   "stop_ev": stop_ev})

    agent_list = [Agent(uid, build_motion_controller(init_info), wps)
                  for uid, (init_info, wps) in scenario.items()]
    agent_proc_list = []  # type: List[Process]
    agent_queue_list = []  # type: List[Tuple[Queue, Queue]]
    for aut in agent_list:
        aut_i_queue, aut_o_queue = Queue(), Queue()
        agent_queue_list.append((aut_i_queue, aut_o_queue))
        agent_proc_list.append(Process(target=run_as_process,
                                       kwargs={"aut": aut,
                                               "i_queue": aut_i_queue,
                                               "o_queue": aut_o_queue,
                                               "stop_ev": stop_ev}))

    proc_list = [air_mgr_proc] + agent_proc_list

    act_list = []  # type: List[bytes]
    try:
        for proc in proc_list:
            proc.start()
            while not proc.is_alive():
                pass
        # Stay active if any agent is alive
        while any(agent_proc.is_alive() for agent_proc in agent_proc_list):
            # Multicast messages
            sent_act_list = _multicast((air_mgr_i_queue, air_mgr_o_queue),
                                       agent_queue_list)
            act_list.extend(sent_act_list)
    finally:
        stop_ev.set()  # Signal all automatons to stop especially AirspaceManager
        air_mgr_i_queue.close()
        air_mgr_o_queue.close()
        for agent_i_queue, agent_o_queue in agent_queue_list:
            agent_i_queue.close()
            agent_o_queue.close()
        for proc in proc_list:
            proc.join(2)
        for proc in proc_list:
            if proc.is_alive():
                print(proc.name, "is still alive. Escalate to SIGTERM")
                proc.terminate()

        print("========== Recorded Actions =========")
        # TODO More detailed statistics
        print("Total actions: %d, Requests: %d, Replies: %d, Releases: %d" %
              (len(act_list),
               sum(act[0] == "request" for act in act_list),
               sum(act[0] == "reply" for act in act_list),
               sum(act[0] == "release" for act in act_list)))


def __build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run UTM protocol with the given scene and scenarios"
    )
    parser.add_argument('scene', type=argparse.FileType('r'),
                        help="Yaml file of the scene with vehicle types and positions")
    return parser


def __process_scene_yaml(fd) -> Dict[str, Dict[str, MotionInitInfo]]:
    cfg = yaml.safe_load(fd)
    if isinstance(cfg, list):
        world_name = "irl_arena.world"
        cfg_devices = cfg  # type: Sequence[Dict[str, Any]]
    elif isinstance(cfg, dict):
        world_name = cfg['world_name']
        cfg_devices = cfg['devices']
    else:
        raise ValueError("Unexpected value in YAML file")

    return world_name, {dev["bot_name"]: MotionInitInfo(
                bot_name=dev["bot_name"],
                bot_type=dev["bot_type"],
                position=tuple(dev["init_pos"]),
                yaw=dev.get("init_yaw", 0.0),
                topic_prefix=dev["bot_name"]
            ) for dev in cfg_devices}


def main(argv=None):
    parser = __build_argparser()
    if argv is None:
        argv = parser.parse_args()
    else:
        argv = parser.parse_args(argv)

    # rospy.init_node('utm_agents')

    # Include device init info from scene yaml file into scenarios
    world_name, device_info_map = __process_scene_yaml(argv.scene)

    # Select default waypoints based on world file. Change here to select different predefined waypoint paths
    if world_name == "city.world":
        # selected_scenario = city_scenarios.AIRPORT
        # selected_agents = {
        #     'drone0',
        #     'drone1',
        #     'plane0',
        #     'plane1',
        # }

        selected_scenario = ACAS_scenarios.ACAS_01_9
        selected_agents = {
            'drone0',
            'drone1',
        }
    elif world_name == "eceb.world":
        selected_scenario = eceb_scenarios.SIMPLE_CORRIDOR
        selected_agents = {'drone0',
                           'drone1',
                           'drone2',
                           'drone3',
                           'drone4',
                           'drone5'
                          }
    else:
        # TODO
        raise NotImplementedError("Need to handle different maps and waypoints")

    if not selected_agents.issubset(set(device_info_map.keys())):
        raise ValueError("Not all selected agents are specified in %s" % argv.scene.name)

    sc = {key: (device_info_map[key], wps) for key, wps in selected_scenario.items() if key in selected_agents}
    print(datetime.datetime.now().replace(microsecond=0).isoformat(), ':')
    test_protocol(sc)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("KeyboardInterrupt.")
    finally:
        print("Ending test driver thread...")
