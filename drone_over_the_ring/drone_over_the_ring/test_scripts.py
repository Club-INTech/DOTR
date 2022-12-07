from drone_control import Drone
from gate_descriptor import GateDescriptor
import numpy as np
import sys
import time


PAUSE = 3
command_list = ["forward 20",
                "back 20"]

def mock_routine(img: np.ndarray) -> GateDescriptor:
    return GateDescriptor()


if __name__ == "__main__":
    print("===== Welcome to test mode =====")
    if len(sys.argv) <= 1:
        print("Please provide drone mode:[order, stream, control]")
        sys.exit(1)

    _mode = sys.argv[1]
    drone = None
    if _mode == "order":
        drone = Drone(mock_routine,
                      use_order=True,
                      use_video=False,
                      use_navigation=False,
                      debug=False)
    elif _mode == "stream":
        drone = Drone(mock_routine,
                      use_order=True,
                      use_video=True,
                      use_navigation=False,
                      debug=True)
    elif _mode == "control":
        print("Method not implemented yet")
        sys.exit(0)
    else:
        print(f'Method {_mode} does not exist')
        sys.exit(1)

    drone.run()
    for _cmd in command_list:
        drone.execute_order(_cmd)
        time.sleep(PAUSE)
    drone.stop()
