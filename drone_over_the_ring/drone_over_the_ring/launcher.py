from typing import Tuple
import numpy as np
import sys
import getopt
from drone_control import Drone
from djitellopy import Tello
from gate_descriptor import GateDescriptor
from detect import GateDetector
import providers as prs
import const


def mock_routine(img: np.ndarray) -> Tuple[np.ndarray, GateDescriptor]:
    return img, GateDescriptor()


def _help():
    print("Usage: python3 test_scripts.py [-h] [--mode]")
    print("=====================")
    print()
    print("-h or --help         print help")
    print()
    print("This launcher contains only predefined modes.")
    print("If you want use your own mode create ")
    print("and configure drone object directly")
    print()
    print("--mode={viz|ord|rc|rcam|full|sim}")
    print("viz mode allows show images processed by yolo without flying")
    print("ord mode is the same thing as viz mode but allows to log orders")
    print("rc mode allows to control drone with keyboard")
    print("rccam mode is rc mode with camera")
    print("full mode is an autonomous navigation mode")
    print("sim mode is an autonoumous simulated mode")
    print()
    print("=====================")


if __name__ == "__main__":

    print("===== Welcome to launcher =====")

    try:
        opts, args = getopt.getopt(sys.argv[1:], "h", ["help",
                                                       "mode="])
    except getopt.GetoptError as err:
        print(err)
        print()
        _help()
        sys.exit(1)

    gate_detector = GateDetector(weights="./config/wheights_training.pt")

    _mode = "full"  # defaults to full mode
    _img_process_routine = gate_detector.run
    _use_navigation = True
    _use_control = False
    _order_provider = prs.BasicOrderProvider()
    _video_provider = prs.EmptyVideoProvider()

    for opt, arg in opts:
        if opt in ("-h", "--help"):
            _help()
            sys.exit(0)
        elif opt == "--mode":
            _mode = arg

    if _mode == "viz":
        tello = Tello(retry_count=const.RETRY)
        _use_navigation = False
        _video_provider = prs.TelloVideoProvider(tello)
    elif _mode == "ord":
        tello = Tello(retry_count=const.RETRY)
        _video_provider = prs.TelloVideoProvider(tello)
    elif _mode == "rc":
        _img_process_routine = mock_routine
        tello = Tello(retry_count=const.RETRY)
        _use_navigation = False
        _use_control = True
        _order_provider = prs.TelloOrderProvider(tello)
    elif _mode == "rcam":
        _img_process_routine = mock_routine
        tello = Tello(retry_count=const.RETRY)
        _use_navigation = False
        _use_control = True
        _order_provider = prs.TelloOrderProvider(tello)
        _video_provider = prs.TelloVideoProvider(tello)
    elif _mode == "full":
        tello = Tello(retry_count=const.RETRY)
        _order_provider = prs.TelloOrderProvider(tello)
        _video_provider = prs.TelloVideoProvider(tello)
    elif _mode == "sim":
        print("Not implemented yet")
        sys.exit(1)
    else:
        print("This control mode is not implemented")
        sys.exit(1)

    drone = Drone(_img_process_routine,
                  use_navigation=_use_navigation,
                  use_control=_use_control,
                  order_provider=_order_provider,
                  video_provider=_video_provider)
    drone.run()
