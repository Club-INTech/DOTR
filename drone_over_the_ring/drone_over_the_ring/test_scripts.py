from drone_control import Drone
from gate_descriptor import GateDescriptor
import numpy as np
import keyboard
import sys
import getopt
import yaml
import time
from detect import GateDetector
import cv2 as cv


PAUSE = 5
MOV_SPEED = 40
ROT_SPEED = 20
Z_SPEED = 10

DELAY = 0.05

def mock_routine(img: np.ndarray) -> GateDescriptor:
    return GateDescriptor()


def _help():
    print("Usage: python3 test_scripts.py [-hf] [--ctrl] [--video] [--cmdlist] [--output]")
    print("=====================")
    print()
    print("-h or --help         print help")
    print("-f or --full         full test mode with autonomous navigation")
    print()
    print("--ctrl={rc|cmd}")
    print("rc mode allows to control the drone with keyboard")
    print("cmd mode allows to execute orders predefined with --cmdlist")
    print()
    print("--video={on|off}")
    print("Put video on or off")
    print()
    print("--cmdlist={file}")
    print("Provide file with commands")
    print()
    print("--output={folder}")
    print("Output folder for output images")
    print()
    print("=====================")


if __name__ == "__main__":

    print("===== Welcome to test mode =====")

    try:
        opts, args = getopt.getopt(sys.argv[1:], "hf", ["help",
                                                        "full",
                                                        "ctrl=",
                                                        "video=",
                                                        "cmdlist=",
                                                        "output="])
    except getopt.GetoptError as err:
        print(err)
        print()
        _help()
        sys.exit(2)

    _ctrl_mode = None
    _video_mode = False
    _cmd_list = None
    _output = None
    _full_mode = False

    for opt, arg in opts:
        if opt in ("-h", "--help"):
            _help()
            sys.exit(0)
        elif opt in ("-f", "--full"):
            _full_mode = True
            break
        elif opt == "--ctrl":
            _ctrl_mode = arg
        elif opt == "--video":
            if arg == "on":
                _video_mode = True
            else:
                _video_mode = False
        elif opt == "--cmdlist":
            _cmd_list = arg
        elif opt == "--output":
            _output = arg

    drone = None
    gate_detector = GateDetector(weights="/home/gaetan/DOTR/drone_over_the_ring/drone_over_the_ring/config/wheights_training.pt")

    if _full_mode:
        print("===== Enter full mode =====")
        drone = Drone(img_process_routine=gate_detector.run,
                    use_order=True,
                    use_video=True,
                    use_control=True,
                    use_navigation=True,
                    navigation_mock=False,
                    video_mock=False,
                    debug=True)
        drone.run()
        sys.exit(0)
    else:

        use_control = True
        if _ctrl_mode is None:
            use_control = False

        drone = Drone(img_process_routine=gate_detector.run,
                      use_order=True,
                      use_video=_video_mode,
                      use_control=use_control,
                      use_navigation=False,
                      navigation_mock=False,
                      video_mock=False,
                      debug=True)

        drone.run()

        if _ctrl_mode == "cmd":
            if _cmd_list is None:
                print("You must provide a command list in yaml format")
                sys.exit(1)
            cmd_list = None
            try:
                cmd_list = yaml.safe_load(_cmd_list)
            except yaml.YAMLError as err:
                print(err)
                sys.exit(1)

            time.sleep(PAUSE)
            for _cmd in cmd_list["orders"]:
                drone.execute_order(_cmd)
                time.sleep(PAUSE)
            drone.stop()

        elif _ctrl_mode == "rc":
            state_vector = [0, 0, 0, 0]
            drone.execute_order("takeoff")
            time.sleep(PAUSE)
            while True:
                event = keyboard.read_event()

                if event.event_type == keyboard.KEY_DOWN and event.name == "q":
                    break

                if event.event_type == keyboard.KEY_DOWN and event.name == "a":
                    if state_vector[0] == 0:
                        state_vector[0] = -MOV_SPEED
                elif event.event_type == keyboard.KEY_UP and event.name == "a":
                    if state_vector[0] == -MOV_SPEED:
                        state_vector[0] = 0

                if event.event_type == keyboard.KEY_DOWN and event.name == "d":
                    if state_vector[0] == 0:
                        state_vector[0] = MOV_SPEED
                elif event.event_type == keyboard.KEY_UP and event.name == "d":
                    if state_vector[0] == MOV_SPEED:
                        state_vector[0] = 0

                if event.event_type == keyboard.KEY_DOWN and event.name == "w":
                    if state_vector[1] == 0:
                        state_vector[1] = MOV_SPEED
                elif event.event_type == keyboard.KEY_UP and event.name == "w":
                    if state_vector[1] == MOV_SPEED:
                        state_vector[1] = 0

                if event.event_type == keyboard.KEY_DOWN and event.name == "s":
                    if state_vector[1] == 0:
                        state_vector[1] = -MOV_SPEED
                elif event.event_type == keyboard.KEY_UP and event.name == "s":
                    if state_vector[1] == -MOV_SPEED:
                        state_vector[1] = 0

                if event.event_type == keyboard.KEY_DOWN and event.name == "i":
                    if state_vector[2] == 0:
                        state_vector[2] = MOV_SPEED
                elif event.event_type == keyboard.KEY_UP and event.name == "i":
                    if state_vector[2] == MOV_SPEED:
                        state_vector[2] = 0

                if event.event_type == keyboard.KEY_DOWN and event.name == "k":
                    if state_vector[2] == 0:
                        state_vector[2] = -MOV_SPEED
                elif event.event_type == keyboard.KEY_UP and event.name == "k":
                    if state_vector[2] == -MOV_SPEED:
                        state_vector[2] = 0

                if event.event_type == keyboard.KEY_DOWN and event.name == "j":
                    if state_vector[3] == 0:
                        state_vector[3] = -MOV_SPEED
                elif event.event_type == keyboard.KEY_UP and event.name == "j":
                    if state_vector[3] == -MOV_SPEED:
                        state_vector[3] = 0

                if event.event_type == keyboard.KEY_DOWN and event.name == "l":
                    if state_vector[3] == 0:
                        state_vector[3] = MOV_SPEED
                elif event.event_type == keyboard.KEY_UP and event.name == "l":
                    if state_vector[3] == MOV_SPEED:
                        state_vector[3] = 0

                _cmd = "rc {state_vector[0]} {state_vector[1]} " + \
                    "{state_vector[2]} {state_vector[3]}"
                drone.execute_order(_cmd)
                time.sleep(DELAY)

            drone.stop()

        else:
            print("Control method {_ctrl_mode} is not implemented")
            sys.exit(1)
