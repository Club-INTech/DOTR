import time
import numpy as np
import multiprocessing as mp
import socket
from dataclasses import dataclass
from enum import Enum
import cv2.cv2 as cv
from multiprocessing.sharedctypes import Synchronized
from typing import Callable
from gate_descriptor import GateDescriptor, GateType
from datetime import datetime
import yaml


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
    not_detected_count: int = 20
    not_detected_limit: int = 20
    gate_navigation_step: NavigationStep \
        = NavigationStep.NOT_DETECTED
    going_through_gate: bool = False


class Drone():

    def __init__(self,
                 img_process_routine: Callable[[np.ndarray], GateDescriptor],
                 navigation_config: str = "config/default_nav_config.yaml",
                 use_order: bool = True,
                 use_video: bool = True,
                 use_navigation: bool = True,
                 debug: bool = False) -> None:

        self.tilt = mp.Value('i', 0)

        self.__use_order = use_order
        self.__use_video = use_video
        self.__use_navigation = use_navigation

        self.__order_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.__server_address = ('', 8889)
        self.__drone_address = ('192.168.10.1', 8889)
        self.__drone_video_server_address = ('', 11111)

        self.RETRY = 3
        self.TIMEOUT = 5
        self.WAITING = 2
        self.VIDEO_STREAM_DELAY = 0.05
        self.DELAY = 0.5
        self.STOP_DELAY = 20

        self.__order_queue = mp.Queue()
        self.__img_process_queue = mp.Queue()
        self.log_file = "log/drone " + str(datetime.now())
        self.log_file = self.log_file.replace(" ", "_")

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

        self.__order_socket.settimeout(self.TIMEOUT)
        self.__order_socket.bind(self.__server_address)
        self.__video_socket.bind(self.__drone_video_server_address)

        self.order_worker = mp.Process(
                target=self.__order_executor,
                args=(self.__order_queue,
                      self.__order_socket,
                      self.log_file,
                      self.tilt))

        self.video_receiver_worker = mp.Process(
                target=self.__video_receiver,
                args=(self.__img_process_queue,
                      debug)
                )

        self.navigation_worker = mp.Process(
                target=self.__navigation_cycle,
                args=(self.__img_process_queue,
                      self.__order_queue,
                      img_process_routine,
                      self.tilt))

    def __order_executor(self,
                         ord_q: mp.Queue,
                         sock: socket.socket,
                         log_file: str,
                         tilt: Synchronized) -> None:

        while True:
            _cmd = ord_q.get(block=True)
            f = open(log_file, 'a')
            f.write(_cmd + " : Waiting for response\n")
            for _ in range(self.RETRY):
                sock.sendto(_cmd.encode('utf-8'), self.__drone_address)
                time.sleep(self.WAITING)

                # TODO: set tilt of the drone

                try:
                    bytes_, _ = sock.recvfrom(1024)
                    if bytes_ == b'ok':
                        f.write(_cmd + " : ok\n")
                        break
                    else:
                        f.write(_cmd + " : failure\n")
                except socket.timeout as e:
                    err = e.args[0]
                    if err == 'timed out':
                        time.sleep(self.WAITING)
                        f.write(_cmd + " : timed out\n")
                    else:
                        f.write(_cmd + " : " + str(err) + "\n")
                    continue
                except socket.error as e:
                    f.write(_cmd + " : " + str(e.args[0]) + "\n")
                    continue
            f.close()

    def execute_order(self,
                      order: str) -> None:
        self.__order_queue.put(order)

    def __video_receiver(self,
                         img_q: mp.Queue,
                         debug=False) -> None:

        _stream_now = time.process_time()
        while True:
            frame = b''
            while True:
                byte_arr, _ = self.__video_socket.recvfrom(2048)
                frame += byte_arr
                if len(byte_arr) != 1460:
                    break

            img = np.asarray(frame, dtype=np.uint8)

            if time.process_time() - _stream_now \
                    >= self.VIDEO_STREAM_DELAY:
                img_q.put(img)
                _stream_now = time.process_time()

            if debug:
                cv.imshow('frame', img)
                if cv.waitKey(1) == ord('q'):
                    break

    def __navigation_cycle(self,
                           img_q: mp.Queue,
                           ord_q: mp.Queue,
                           process_routine: Callable[[np.ndarray, float],
                                                     GateDescriptor],
                           tilt: Synchronized) -> None:

        # prev_state = (0.0, 0.0, 0.0, 0.0, False)
        while True:
            img = img_q.get(block=True)
            _gate_descriptor = process_routine(img, tilt.value)
            if _gate_descriptor.type_ == GateType.NO_GATE:
                ord_q.put("cw 360")
            else:
                pass
                # state = (_gate_descriptor.x,
                         # _gate_descriptor.y,
                         # _gate_descriptor.z,
                         # _gate_descriptor.pixel_width
                         # / _gate_descriptor.pixel_height,
                         # True)

            # TODO: implement navigation logic

    def run(self) -> None:

        if self.__use_order:
            self.execute_order("command")
            time.sleep(self.DELAY)
            self.order_worker.start()
            time.sleep(self.DELAY)
        if self.__use_video:
            self.execute_order("streamon")
            time.sleep(self.DELAY)
            self.video_receiver_worker.start()
            time.sleep(self.DELAY)
        if self.__use_navigation:
            self.navigation_worker.start()

    def join(self):
        if self.__use_navigation:
            self.navigation_worker.join()
        if self.__use_video:
            self.video_receiver_worker.join()
        if self.__use_order:
            self.order_worker.join()
        cv.destroyAllWindows()

    def stop(self) -> None:
        if self.navigation_worker.is_alive():
            self.navigation_worker.kill()
            time.sleep(self.DELAY)
        if self.video_receiver_worker.is_alive():
            self.execute_order("streamoff")
            self.video_receiver_worker.kill()
            time.sleep(self.DELAY)

        self.execute_order("rc 0 0 0 0")
        self.execute_order("land")
        time.sleep(self.STOP_DELAY)
        print("===== Goodbye =====")

        if self.order_worker.is_alive():
            self.order_worker.kill()
