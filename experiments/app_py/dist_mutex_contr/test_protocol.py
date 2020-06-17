from multiprocessing import connection, Event, Pipe, Process
from typing import List, Sequence

from reachtube import Contract
from scipy.spatial import Rectangle

from .airspace_manager import AirspaceManager
from .agent import Agent
from .motion import MotionHectorQuad
from .tioa_base import AutomatonBase, run_as_process


def test_agent() -> None:
    stop_ev = Event()
    conn, agent_conn = Pipe()

    aut = Agent(uid=1, motion=MotionHectorQuad("/drone1"))

    p = Process(target=run_as_process, kwargs={"aut": aut,
                                               "conn": agent_conn,
                                               "stop_ev": stop_ev})
    p.start()

    try:
        for i in range(10):
            if conn.poll(1.0):
                act = conn.recv()
                print(act)
                if act[0] == "request":
                    reply = input("> ")
                    conn.send(("reply", {"uid": aut.uid,
                                         "acquired": Contract()}))
            else:
                print("Response timeout")
    finally:
        stop_ev.set()  # Stop all automatons
        p.join()
        conn.close()
        agent_conn.close()


def test_contract_manager() -> None:
    aut = AirspaceManager()
    stop_ev = Event()
    conn, manager_conn = Pipe()

    p = Process(target=run_as_process, kwargs={"aut": aut,
                                               "conn": manager_conn,
                                               "stop_ev": stop_ev})
    p.start()
    try:
        uid = 0
        for i in range(10):
            if conn.poll(1.0):
                act = conn.recv()
                print(act)
            elif i % 3 != 2:
                uid = i % 5
                target = Contract.from_stamped_rectangles([
                    (0.0, Rectangle(mins=[0, 0, 0], maxes=[1, 1, 0.5])),
                    (0.5, Rectangle(mins=[0, 0.5, 0], maxes=[2, 3, 0.5])),
                    (1.0, Rectangle(mins=[0.5, 0.5, 1.0], maxes=[1.5, 1.5, 1.5]))
                    ])
                conn.send(("request", {"uid": uid, "target": target}))
            else:
                releasable = Contract.from_stamped_rectangles([
                    (0.0, Rectangle(mins=[0, 0, 0], maxes=[1, 1, 0.5])),
                    (0.5, Rectangle(mins=[0, 0.5, 0], maxes=[2, 2, 0.5]))
                    ])
                print("Agent " + str(uid) + " release > " + str(releasable))
                conn.send(("release", {"uid": uid, "releasable": releasable}))
    finally:
        stop_ev.set()  # Stop all automatons
        p.join()
        conn.close()
        manager_conn.close()


def _multicast(conn_seq: Sequence[connection.Connection]) -> Sequence[bytes]:
    act_list = []
    conn_w_msg_list = connection.wait(conn_seq, timeout=0.0)
    # TODO deliver messages only to automatons using the actions
    for conn_w_msg in conn_w_msg_list:
        assert isinstance(conn_w_msg, connection.Connection)
        act = conn_w_msg.recv()
        act_list.append(act)
        for conn in conn_seq:
            if conn != conn_w_msg:
                conn.send(act)
    return act_list


def test_protocol(num_agents: int = 5) -> None:
    stop_ev = Event()

    aut_list = [AirspaceManager()]  # type: List[AutomatonBase]

    aut_list.extend(Agent(uid, MotionHectorQuad(uid))
                    for uid in ("drone" + str(i) for i in range(num_agents)))

    proc_list = []  # type: List[Process]
    channel_conn_list = []  # type: List[connection.Connection]
    for aut in aut_list:
        channel_conn, aut_conn = Pipe()
        channel_conn_list.append(channel_conn)
        proc_list.append(Process(target=run_as_process,
                                 kwargs={"aut": aut,
                                         "conn": aut_conn,
                                         "stop_ev": stop_ev}))

    act_list = []  # type: List[bytes]
    try:
        for proc in proc_list:
            proc.start()
            while not proc.is_alive():
                pass

        while len(act_list) < 1000 and not stop_ev.is_set():
            # Multicast messages
            sent_act_list = _multicast(channel_conn_list)
            act_list.extend(sent_act_list)
    finally:
        stop_ev.set()  # Stop all automatons
        for proc in proc_list:
            proc.join(2)
        for proc in proc_list:
            if proc.is_alive():
                print(proc.name, "is still alive. Escalate to SIGTERM")
                proc.terminate()
        for conn in channel_conn_list:
            conn.close()

        print("========== Recorded Actions =========")
        for act in act_list:
            print(act)


if __name__ == "__main__":
    try:
        # TODO make them individual test files
        # test_agent()
        # test_contract_manager()
        test_protocol(num_agents=3)
    except KeyboardInterrupt:
        print("KeyboardInterrupt.")
    finally:
        print("Ending test driver thread...")
