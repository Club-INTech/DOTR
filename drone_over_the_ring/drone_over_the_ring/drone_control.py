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
from const import *


class NavigationStep(Enum):
    NOT_DETECTED = 1
    DETECTED = 2
    GO_THROUGH = 3


@dataclass(init=True, repr=True, eq=True, order=False)
class DroneState():
    eps_x: float = 0.05
    eps_y: float = 0.05
    eps_z: float = 0.05
    eps_yaw: float = 0.08
    s: float = 0.3
    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0
    yaw_sign: float = 1.0
    prev_dyaw: float = 0.0
    dyaw: float = 0.0
    not_detected_count: int = 0
    not_detected_limit: int = 20
    gate_count: int = 0
    gate_navigation_step: NavigationStep \
        = NavigationStep.NOT_DETECTED
    going_through_gate: bool = False


class Drone():

    def __init__(self,
                 img_process_routine: Callable[[np.ndarray], Tuple[np.ndarray, GateDescriptor]],
                 navigation_config: str = "/home/gaetan/DOTR/drone_over_the_ring/drone_over_the_ring/config/default_nav_config.yaml",
                 use_order: bool = True,
                 use_video: bool = True,
                 use_control: bool = True,
                 use_navigation: bool = True,
                 navigation_mock: bool = False,
                 video_mock: bool = False,
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
        self.navigation_mock = navigation_mock
        self.video_mock = video_mock

        self.__kpx = 5.0
        self.__krx = 20.0

        self.__kpy = 5.0
        self.__kry = 20.0

        self.__kpz = 5.0
        self.__krz = 20.0

        self.__kpyaw = 15.0
        self.__kryaw = 10.0

        with open(navigation_config, 'r') as _stream:
            _conf = yaml.safe_load(_stream)
            coefs = ["kpx", "krx", "kpy", "kry", "kpz", "krz", "kpyaw", "kryaw"]
            for _c in coefs:
                if _c in _conf:
                    self.__setattr__(_c, _conf[_c])

        self.tello = None
        if self.navigation_mock and self.video_mock:
            print("command")
            print("streamon")
        else:
            self.tello = Tello(retry_count=self.RETRY)
            if self.navigation_mock:
                print("command")
                print("streamon")
            else:
                self.tello.connect()
                self.tello.streamon()

        self.order_worker = mp.Process(
                target=self.__order_executor,
                args=(self.__order_queue,
                      self.TIMEOUT,
                      self.navigation_mock,
                      self.tello))

        self.video_receiver_worker = mp.Process(
                target=self.__video_receiver,
                args=(self.tello,
                      self.child_conn)
                )
        
    def __order_executor(self,
                         ord_q: mp.Queue,
                         timeout: int,
                         navigation_mock: bool,
                         tello: Tello) -> None:
        if tello is None:
            return
        while True:
            _cmd = ord_q.get(block=True)
            if navigation_mock:
                print(_cmd)
            else:
                tello.send_control_command(_cmd)
            

    def execute_order(self,
                      order: str) -> None:
        self.__order_queue.put(order)

    def __video_receiver(self,
                         tello: Tello,
                         conn) -> None:
        if tello is None:
            return
        frame_grabber = tello.get_frame_read()
        while True:
            conn.send(frame_grabber.frame)

    def run(self) -> None:

        if self.__use_order and not self.mock:
            self.order_worker.start()
            time.sleep(self.DELAY)
        if self.__use_video and not self.mock:
            self.video_receiver_worker.start()
            time.sleep(self.DELAY)
        if self.__use_control:
            self.execute_order("takeoff")
            time.sleep(self.TAKEOFF_DELAY)

        if self.__use_navigation and self.__use_control and self.__use_video and self.__use_order:
            _st = DroneState()
            while not self.stop:
                img = None
                if not self.video_mock:
                    img = self.parent_conn.recv()
                _img, _desc = self.img_process_routine(img)
                
                if _st.gate_count >= GATE_NUMBER:
                    self.stop = True
                    break

                if _desc.type_ == GateType.NO_GATE:
                    _st.not_detected_count += 1
                    if _st.not_detected_count >= _st.not_detected_limit:
                        self.execute_order("rc 0 0 0 0")
                        _st.not_detected_count = 0
                        _st.gate_navigation_step = NavigationStep.NOT_DETECTED
                    else:
                        continue
                else:
                    _st.not_detected_count = 0
                    if _st.gate_navigation_step == NavigationStep.NOT_DETECTED:
                        self.execute_order("rc 0 0 0 0")
                        _st.gate_navigation_step = NavigationStep.DETECTED
                    _st.dx = _desc.x
                    _st.dy = _desc.y
                    _st.dz = _desc.z
                    _st.dyaw = _st.yaw_sign * _desc.alpha

                if _st.gate_navigation_step == NavigationStep.NOT_DETECTED:
                    self.execute_order("rc 0 0 0 10")
                elif _st.gate_navigation_step == NavigationStep.DETECTED:

                    if abs(_st.dyaw) <= _st.eps_yaw and abs(_st.dx) <= _st.eps_x \
                            and abs(_st.dy) <= _st.eps_y and abs(_st.dz) <= _st.eps_z:
                        _cmd = "forward " + str(1.5 * _st.s)
                        self.execute_order(_cmd)
                        _st.gate_navigation_step = NavigationStep.NOT_DETECTED
                        time.sleep(self.WAITING)
                        _st.gate_count += 1
                        continue

                    if abs(_st.prev_dyaw) > abs(_st.dyaw):
                        _st.dyaw = (-1) * _st.dyaw
                        _st.yaw_sign = -1.0
                    
                    # calculate of true x and y on the line perpendicular to the center
                    # of gate and distance s from this center

                    if abs(_st.dyaw) <= _st.eps_yaw:
                        _st.dx = _st.dx
                        _st.dy = _st.dy - _st.s
                    else:
                        # perpendicular line
                        _alpha = - (1 / _st.dyaw)
                        _beta = _st.dy - _alpha * _st.dx

                        # sign of dyaw used to choose equation solution
                        _k = 1
                        if _st.dyaw < 0:
                            _k = -1

                        # resolve equation ax^2 + bx +c = 0
                        a = (1 + _alpha ** 2)
                        b = (2* _st.dy / _st.dyaw - 2 * _beta / _st.dyaw - 2 * _st.dx)
                        c = (_st.dx ** 2 + _st.dy ** 2 + _beta ** 2 - 2 * _beta * _st.dy - _st.s ** 2)
                        x12 = np.roots([a, b, c])
                        if _k < 0:
                            _st.dx = min(x12[0], x12[1])
                        else:
                            _st.dx = max(x12[0], x12[1])
                        _st.dy = _alpha * _st.dx + _beta 

                    _max_d = max([_st.dx, _st.dy, _st.dz])
                    raw_speed_x = self.__kpx * _st.dx + self.__krx * (_st.dx / _max_d)
                    raw_speed_y = self.__kpy * _st.dy + self.__kry * (_st.dy / _max_d)
                    raw_speed_z = self.__kpz * _st.dz + self.__krz * (_st.dz / _max_d)
                    raw_speed_yaw = self.__kpyaw * _st.dyaw + self.__kryaw * (_st.dyaw / _max_d)

                    speed_x = int(min(MAX_DRONE_SPEED, max(MIN_DRONE_SPEED, raw_speed_x)))
                    speed_y = int(min(MAX_DRONE_SPEED, max(MIN_DRONE_SPEED, raw_speed_y)))
                    speed_z = int(min(MAX_DRONE_SPEED, max(MIN_DRONE_SPEED, raw_speed_z)))
                    speed_yaw = int(min(MAX_YAW_SPEED, max(MIN_YAW_SPEED, raw_speed_yaw)))

                    _cmd = "rc {speed_x} {speed_y} {speed_z} {speed_yaw}".format(speed_x=speed_x,
                                                                                 speed_y=speed_y,
                                                                                 speed_z=speed_z,
                                                                                 speed_yaw=speed_yaw)
                    self.execute_order(_cmd)
                    _st.prev_dyaw = _st.dyaw

        elif self.__use_video and self.debug:
            while not self.stop:
                img = self.parent_conn.recv()
                _img, _desc = self.img_process_routine(img)
                cv.imshow("frame", _img)
                if cv.waitKey(1) == ord('q'):
                    self.stop = True

        else:
            print("This method is not known")

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
