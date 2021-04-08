from multiprocessing import Event, Pipe
from multiprocessing.context import Process

from reachtube import Contract

from dist_mutex_contr.agent import Agent
from dist_mutex_contr.motion import MotionHectorQuad
from dist_mutex_contr.tioa_base import run_as_process


def test_agent() -> None:
    stop_ev = Event()
    conn, agent_conn = Pipe()

    waypoints = [(-80.0, -65.0, 3.0), (-45.0, -66.0, 3.0), (-33.0, -65.0, 3.0), (-33.0, -65.0, 0.3)]
    aut = Agent(uid=1, motion=MotionHectorQuad("/drone1"), waypoints=waypoints)

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
