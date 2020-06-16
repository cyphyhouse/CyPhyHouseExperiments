from multiprocessing import Event, Pipe, Process
from multiprocessing import connection
from typing import List, Sequence, Tuple, Any

from .airspace_manager import AirspaceManager
from .agent import Agent
from .tioa_base import AutomatonBase, run_as_process


def test_agent() -> None:
    stop_ev = Event()
    conn, agent_conn = Pipe()

    aut = Agent(uid=1)

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
                                         "acquired": set(reply)}))
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
                target = input("Agent " + str(uid) + " request > ")
                conn.send(("request", {"uid": uid, "target": set(target)}))
            else:
                releasable = input("Agent " + str(uid) + " release > ")
                conn.send(("release", {"uid": uid, "releasable": set(releasable)}))
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

    def functor(uid):
        def build_motion() -> None:
            return None
        return build_motion

    aut_list = [(AirspaceManager(), None)]  # type: List[Tuple[AutomatonBase, Any]]
    aut_list += [(Agent(uid), functor(uid)) for uid in range(num_agents)]

    proc_list = []  # type: List[Process]
    channel_conn_list = []  # type: List[connection.Connection]
    for aut, aut_ctrl in aut_list:
        channel_conn, aut_conn = Pipe()
        channel_conn_list.append(channel_conn)
        proc_list.append(Process(target=run_as_process,
                                 kwargs={"aut": aut,
                                         "conn": aut_conn,
                                         "ctrl": aut_ctrl,
                                         "stop_ev": stop_ev}))

    act_list = []  # type: List[bytes]
    try:
        for proc in proc_list:
            proc.start()
            while not proc.is_alive():
                pass

        while len(act_list) < 10 and not stop_ev.is_set():
            # Multicast messages
            sent_act_list = _multicast(channel_conn_list)
            act_list.extend(sent_act_list)
    finally:
        stop_ev.set()  # Stop all automatons
        for proc in proc_list:
            proc.join()
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
        test_protocol()
    except KeyboardInterrupt:
        print("KeyboardInterrupt.")
    finally:
        print("Ending test driver thread...")
