import time
import numpy as np
import multiprocessing as mp
import socket
from dataclasses import dataclass
from enum import Enum
import cv2 as cv
from typing import Callable, Tuple
from gate_descriptor import GateDescriptor, GateType
import threading
from datetime import datetime
import yaml
from djitellopy import Tello
from const import *
import logging


class NavigationStep(Enum):
    '''
        Enumeration allowing to track drone state.
        NOT_DETECTED
    '''
    NOT_DETECTED = 1
    DETECTED = 2
    GO_THROUGH = 3


@dataclass(init=True, repr=True, eq=True, order=False)
class DroneState():
    eps_d: float = 0.18
    eps_yaw: float = 0.1
    final_eps_yaw: float = 0.08
    s: float = 1.6
    vx: int = 0
    vy: int = 0
    vz: int = 0
    vyaw: int = 0
    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0
    sumX: float = 0.0
    sumY: float = 0.0
    sumZ: float = 0.0
    sumYaw: float = 0.0
    offsetZ: float = 0.50
    yaw_sign: float = 1.0
    prev_dyaw: float = 0.0
    dyaw: float = 0.0
    not_detected_count: int = 0
    not_detected_limit: int = 5
    gate_count: int = 0
    gate_navigation_step: NavigationStep \
        = NavigationStep.NOT_DETECTED
    going_through_gate: bool = False


class Drone():

    def __init__(self,
                 img_process_routine: Callable[[np.ndarray], Tuple[np.ndarray, GateDescriptor]],
                 navigation_config: str = "./config/default_nav_config.yaml",
                 use_order: bool = True,
                 use_video: bool = True,
                 use_control: bool = True,
                 use_navigation: bool = True,
                 navigation_mock: bool = False,
                 video_mock: bool = False,
                 debug: bool = False) -> None:

        self.img_process_routine = img_process_routine
        self.video_lock = threading.Lock()

        self.__use_order = use_order
        self.__use_video = use_video
        self.__use_navigation = use_navigation
        self.__use_control = use_control

        self.parent_conn, self.child_conn = mp.Pipe()
        self.__order_queue = mp.Queue(maxsize=10)
        self.stop = False

        self.RETRY = 3
        self.TIMEOUT = 5
        self.WAITING = 2
        self.FORWARD_WAITING = 4
        self.TAKEOFF_DELAY = 5
        self.VIDEO_STREAM_DELAY = 0.2
        self.DELAY = 0.5
        self.STOP_DELAY = 20

        self.debug = debug
        self.navigation_mock = navigation_mock
        self.video_mock = video_mock

        self.__kpx = 10.0
        self.__krx = 15.0
        self.__kix = 0.1
        
        self.__kpy = 10.0
        self.__kry = 15.0
        self.__kiy = 0.1

        self.__kpz = 10.0
        self.__krz = 15.0
        self.__kiz = 0.1

        self.__kpyaw = 10.0
        self.__kryaw = 15.0
        self.__kiyaw = 0.1

        with open(navigation_config, 'r') as _stream:
            _conf = yaml.safe_load(_stream)
            coefs = ["kpx", "krx", "kix", "kpy", 
                     "kry", "kiy", "kpz", "krz", 
                     "kiz", "kpyaw", "kryaw", "kiyaw"]
            for _c in coefs:
                if _c in _conf:
                    self.__setattr__("_Drone__" + _c, _conf[_c])

        self.tello = None
        if self.navigation_mock and self.video_mock:
            print("command")
            print("streamon")
        else:
            self.tello = Tello(retry_count=self.RETRY)
            self.tello.connect()
            self.tello.streamon()
            #self.tello.LOGGER.setLevel(logging.ERROR)
            if self.navigation_mock:
                print("command")
                print("streamon")

        self.order_worker = mp.Process(
                target=self.__order_executor,
                args=(self.__order_queue,
                      self.TIMEOUT,
                      self.navigation_mock,
                      self.tello))

        self.video_receiver_worker = mp.Process(
                target=self.__video_receiver,
                args=(self.tello,
                      self.video_lock,
                      self.child_conn)
                )

    def __sign(self, x) -> int:
        if x < 0:
            return -1
        else:
            return 1
        
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
                if _cmd == "takeoff":
                    tello.takeoff()
                else:
                    tello.send_command_without_return(_cmd)
                    
    def __clear_video_conn(self):
        while self.parent_conn.poll():
            print("clearing video conn")
            _ = self.parent_conn.recv()
            
    def execute_order(self,
                      order: str) -> None:
        self.__order_queue.put(order)

    def __video_receiver(self,
                         tello: Tello,
                         video_lock: mp.Lock,
                         conn) -> None:
        if tello is None:
            return
        frame_grabber = tello.get_frame_read()
        while True:
            if not video_lock.locked():    
                conn.send(frame_grabber.frame)

    def run(self) -> None:

        if self.__use_order:
            self.order_worker.start()
            time.sleep(self.DELAY)
        if self.__use_video and not self.video_mock:
            self.video_receiver_worker.start()
            time.sleep(self.DELAY)
        if self.__use_control:
            self.tello.takeoff()
            time.sleep(self.TAKEOFF_DELAY)
            self.execute_order("up 60")
            time.sleep(self.TAKEOFF_DELAY)

        if self.__use_navigation and self.__use_control and self.__use_video and self.__use_order:
            #print("===== In full mode =====")
            _st = DroneState()
            while not self.stop:
                #print("===== In navigation loop =====")
                img = None
                if not self.video_mock:
                    img = self.parent_conn.recv()
                _img, _desc = self.img_process_routine(img)
                
                if not self.video_mock and self.debug:
                    data1 = "safe_distance: {s:.3f}/dx: {x:.3f}/dy: {y:.3f}/dz: {z:.3f}/dyaw: {dyaw:.3f}/prev_dyaw: {prev_dyaw:.3f}/".format(
                                s=_st.s,
                                x=_st.dx,
                                y=_st.dy,
                                z=_st.dz,
                                dyaw=_st.dyaw,
                                prev_dyaw=_st.prev_dyaw)
                    data2 = "vx: {vx}/vy: {vy}/vz: {vz}/vyaw: {vyaw:.3f}/not_detected_count: {ndc}/gate_count: {gc}/gate_detection_step: {gds}" .format(
                                vx=_st.vx,
                                vy=_st.vy,
                                vz=_st.vz,
                                vyaw=_st.dyaw,
                                ndc=_st.not_detected_count,
                                gc=_st.gate_count,
                                gds=str(_st.gate_navigation_step))
                    data = data1 + data2
                    row_start = 500
                    row_step = 15
                    row = row_start
                    column = 700
                    for i, _d in enumerate(data.split("/")):
                        cv.putText(img=_img, text=_d, org=(column, row), 
                                   fontFace=cv.FONT_HERSHEY_TRIPLEX, fontScale=0.5, color=(0, 255, 0), 
                                   thickness=1)
                        row += row_step
                    cv.imshow("frame", _img)
                    if cv.waitKey(1) == ord('q'):
                        self.stop = True
                
                if _st.gate_count >= GATE_NUMBER:
                    self.stop = True
                    break

                if _desc.type_ == GateType.NO_GATE:
                    _st.not_detected_count += 1
                    if _st.not_detected_count >= _st.not_detected_limit:
                        self.execute_order("rc 0 0 0 0")
                        _st.vx = 0
                        _st.vy = 0
                        _st.vz = 0
                        _st.vyaw = 0
                        _st.not_detected_count = 0
                        _st.gate_navigation_step = NavigationStep.NOT_DETECTED
                        _st.sumX = 0
                        _st.sumY = 0
                        _st.sumZ = 0
                        _st.sumYaw = 0
                    else:
                        continue
                else:
                    _st.not_detected_count = 0
                    if _st.gate_navigation_step == NavigationStep.NOT_DETECTED:
                        self.execute_order("rc 0 0 0 0")
                        _st.vx = 0
                        _st.vy = 0
                        _st.vz = 0
                        _st.vyaw = 0
                        _st.gate_navigation_step = NavigationStep.DETECTED
                    _st.dx = _desc.x
                    _st.dy = _desc.y
                    _st.dz = _desc.z - _st.offsetZ
                    _st.dyaw = _st.yaw_sign * _desc.alpha

                if _st.gate_navigation_step == NavigationStep.NOT_DETECTED:
                    self.execute_order("rc 0 0 0 40")
                    _st.vx = 0
                    _st.vy = 0
                    _st.vz = 0
                    _st.vyaw = 30
                elif _st.gate_navigation_step == NavigationStep.DETECTED:

                    if  abs(_st.dyaw) - abs(_st.prev_dyaw) >= _st.eps_yaw:
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

                    print((_st.dx*_st.dx)+(_st.dy*_st.dy)+(_st.dz*_st.dz)) 
                    if (_st.dx**2)+(_st.dy**2)+(_st.dz**2) <= _st.eps_d and abs(_st.dyaw) <= _st.final_eps_yaw:                
                        print("!!!!!===== Going through gate =====!!!!!")
                        self.execute_order("rc 0 0 0 0")
                        time.sleep(self.WAITING)
                        self.tello.move_forward(int(1.2 * _st.s*100))
                        _st.gate_navigation_step = NavigationStep.NOT_DETECTED
                        _st.sumX = 0
                        _st.sumY = 0
                        _st.sumZ = 0
                        _st.sumYaw = 0
                        time.sleep(self.FORWARD_WAITING)
                        self.video_lock.acquire()
                        self.__clear_video_conn()
                        self.video_lock.release()
                        _st.gate_count += 1
                        continue
                    
                    # Update of the Sum
                    _st.sumX = min(abs(_st.sumX + _st.dx),MAX_INT_SUM) * self.__sign(_st.sumX)
                    _st.sumY = min(abs(_st.sumY + _st.dy),MAX_INT_SUM) * self.__sign(_st.sumY)
                    _st.sumZ = min(abs(_st.sumZ + _st.dz),MAX_INT_SUM) * self.__sign(_st.sumZ)
                    _st.sumYaw = min(abs(_st.sumYaw + _st.dyaw),MAX_INT_SUM) * self.__sign(_st.sumYaw)

                    _max_d = max([_st.dx, _st.dy, _st.dz])
                    raw_speed_x = self.__kpx * _st.dx + self.__krx * (_st.dx / _max_d) + self.__kix * _st.sumX
                    raw_speed_y = self.__kpy * _st.dy + self.__kry * (_st.dy / _max_d) + self.__kiy * _st.sumY
                    raw_speed_z = self.__kpz * _st.dz + self.__krz * (_st.dz / _max_d) + self.__kiz * _st.sumZ
                    raw_speed_yaw = self.__kpyaw * _st.dyaw + self.__kryaw * (_st.dyaw / _max_d) + self.__kiyaw * _st.sumYaw

                    speed_x = self.__sign(raw_speed_x) * int(min(MAX_DRONE_SPEED, max(MIN_DRONE_SPEED, abs(raw_speed_x))))
                    speed_y = self.__sign(raw_speed_y) * int(min(MAX_DRONE_SPEED, max(MIN_DRONE_SPEED, abs(raw_speed_y))))
                    speed_z = self.__sign(raw_speed_z) * int(min(MAX_DRONE_SPEED, max(MIN_DRONE_SPEED, abs(raw_speed_z))))
                    speed_yaw = self.__sign(raw_speed_yaw) * int(min(MAX_YAW_SPEED, max(MIN_YAW_SPEED, abs(raw_speed_yaw))))
                    
                    _st.vx = speed_x
                    _st.vy = speed_y
                    _st.vz = speed_z
                    _st.vyaw = speed_yaw

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