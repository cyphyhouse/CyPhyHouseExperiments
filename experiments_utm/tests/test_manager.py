from multiprocessing import Event, Pipe
from multiprocessing.context import Process

from reachtube import Contract
from scipy.spatial import Rectangle

from dist_mutex_contr.airspace_manager import AirspaceManager
from dist_mutex_contr.tioa_base import run_as_process


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