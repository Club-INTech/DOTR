import time
import numpy as np
import multiprocessing as mp
import socket
from dataclasses import dataclass
from enum import Enum
import cv2 as cv
from typing import Callable, Tuple
from gate_descriptor import GateDescriptor, GateType
from datetime import datetime
import yaml
from djitellopy import Tello


class NavigationStep(Enum):
    NOT_DETECTED = 1
    DETECTED = 2
    GO_THROUGH = 3


@dataclass(init=True, repr=True, eq=True, order=False)
class DroneState():
    prev_dx: float = 0.0
    dx: float = 0.0
    prev_dy: float = 0.0
    dy: float = 0.0
    prev_dz: float = 0.0
    dz: float = 0.0
    prev_dyaw: float = 0.0
    dyaw: float = 0.0
    not_detected_count: int = 0
    not_detected_limit: int = 20
    gate_navigation_step: NavigationStep \
        = NavigationStep.NOT_DETECTED
    going_through_gate: bool = False


class Drone():

    def __init__(self,
                 img_process_routine: Callable[[np.ndarray], Tuple[np.ndarray, GateDescriptor]],
                 navigation_config: str = "config/default_nav_config.yaml",
                 use_order: bool = True,
                 use_video: bool = True,
                 use_control: bool = True,
                 use_navigation: bool = True,
                 debug: bool = False) -> None:

        self.img_process_routine = img_process_routine

        self.__use_order = use_order
        self.__use_video = use_video
        self.__use_navigation = use_navigation
        self.__use_control = use_control
        
        self.__order_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.parent_conn, self.child_conn = mp.Pipe()
        self.__order_queue = mp.Queue(maxsize=10)
        self.stop = False

        self.RETRY = 3
        self.TIMEOUT = 5
        self.WAITING = 2
        self.TAKEOFF_DELAY = 10
        self.VIDEO_STREAM_DELAY = 0.2
        self.DELAY = 0.5
        self.STOP_DELAY = 20

        self.debug = debug

        self.kpx = 5.0
        self.kdx = 4.0
        self.krx = 20.0

        self.kpy = 5.0
        self.kdy = 4.0
        self.krx = 20.0

        self.kryaw = 10.0

        with open(navigation_config, 'r') as _stream:
            _conf = yaml.safe_load(_stream)
            coefs = ["kpx", "kdx", "krx", "kpy", "kdy", "krx", "kryaw"]
            for _c in coefs:
                if _c in _conf:
                    self.__setattr__(_c, _conf[_c])

        self.tello = Tello(retry_count=self.RETRY)
        self.tello.connect()

        self.order_worker = mp.Process(
                target=self.__order_executor,
                args=(self.__order_queue,
                      self.TIMEOUT,
                      self.tello))

        self.video_receiver_worker = mp.Process(
                target=self.__video_receiver,
                args=(self.tello,
                      self.child_conn)
                )
        
    def __order_executor(self,
                         ord_q: mp.Queue,
                         timeout: int,
                         tello: Tello) -> None:

        while True:
            _cmd = ord_q.get(block=True)
            tello.send_control_command(_cmd)
            

    def execute_order(self,
                      order: str) -> None:
        self.__order_queue.put(order)

    def __video_receiver(self,
                         tello: Tello,
                         conn) -> None:

        frame_grabber = tello.get_frame_read()
        while True:
            conn.send(frame_grabber.frame)

    def run(self) -> None:

        if self.__use_order:
            self.order_worker.start()
            time.sleep(self.DELAY)
        if self.__use_video:
            self.video_receiver_worker.start()
            time.sleep(self.DELAY)
        if self.__use_control:
            self.execute_order("takeoff")
            time.sleep(self.TAKEOFF_DELAY)

        while not self.stop:
            if self.__use_video and self.debug:
                img = self.parent_conn.recv()
                _img, _ = self.img_process_routine(img)
                cv.imshow("frame", _img)
                if cv.waitKey(1) == ord('q'):
                    self.stop = True
            elif self.__use_navigation:
                print("Not implementd")
            else:
                print("This functionning mode is not known")

        if self.__use_video:
            self.execute_order("streamoff")
            time.sleep(self.DELAY)
        if self.__use_control:
            self.execute_order("rc 0 0 0 0")
            self.execute_order("land")
            time.sleep(self.STOP_DELAY)

        self.video_receiver_worker.kill()
        self.order_worker.kill()
        cv.destroyAllWindows()
        self.child_conn.close()
        self.parent_conn.close()


    def stop(self) -> None:
        self.stop = True
