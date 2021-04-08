import datetime
from multiprocessing import Manager, Process
from queue import Empty, Queue
from typing import List, Sequence, Tuple

from dist_mutex_contr.airspace_manager import AirspaceManager
from dist_mutex_contr.agent import Agent
from dist_mutex_contr.motion import MotionHectorQuad
from dist_mutex_contr.tioa_base import run_as_process
import eceb_scenarios


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


def test_protocol(scenario) -> None:
    sync_mgr = Manager()
    stop_ev = sync_mgr.Event()

    air_mgr = AirspaceManager()
    air_mgr_i_queue, air_mgr_o_queue = sync_mgr.Queue(), sync_mgr.Queue()
    air_mgr_proc = Process(target=run_as_process,
                           kwargs={"aut": air_mgr,
                                   "i_queue": air_mgr_i_queue,
                                   "o_queue": air_mgr_o_queue,
                                   "stop_ev": stop_ev})

    agent_list = [Agent(uid, MotionHectorQuad(uid), wps)
                  for uid, wps in scenario.items()]
    agent_proc_list = []  # type: List[Process]
    agent_queue_list = []  # type: List[Tuple[Queue, Queue]]
    for aut in agent_list:
        aut_i_queue, aut_o_queue = sync_mgr.Queue(), sync_mgr.Queue()
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
        stop_ev.set()  # Stop all automatons especially AirspaceManager
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


if __name__ == "__main__":
    try:
        selected_scenario = eceb_scenarios.SIMPLE_CORRIDOR
        selected_agents = {'drone0', 'drone1', 'drone2', 'drone3', 'drone4', 'drone5'}
        sc = {key: val for key, val in selected_scenario.items() if key in selected_agents}
        print(datetime.datetime.now().replace(microsecond=0).isoformat(), ':')
        test_protocol(sc)
    except KeyboardInterrupt:
        print("KeyboardInterrupt.")
    finally:
        print("Ending test driver thread...")
