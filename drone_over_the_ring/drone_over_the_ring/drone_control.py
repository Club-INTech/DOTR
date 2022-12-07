import time
import numpy as np
import multiprocessing as mp
import socket
import cv2.cv2 as cv
from multiprocessing.sharedctypes import Synchronized
from typing import Callable, List
from gate_descriptor import GateDescriptor
from datetime import datetime


class Drone():

    def __init__(self,
                 img_process_routine: Callable[[np.ndarray], GateDescriptor],
                 use_order: bool = True,
                 use_video: bool = True,
                 use_navigation: bool = True,
                 debug: bool = False) -> None:

        self.orientation = mp.Value('i', 0)

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
        self.DELAY = 0.5
        self.STOP_DELAY = 20

        self.__order_queue = mp.Queue()
        self.__img_process_queue = mp.Queue()
        self.log_file = "log/drone " + str(datetime.now())
        self.log_file = self.log_file.replace(" ", "_")

        self.__order_socket.settimeout(self.TIMEOUT)
        self.__order_socket.bind(self.__server_address)
        self.__video_socket.bind(self.__drone_video_server_address)

        self.order_worker = mp.Process(
                target=self.__order_executor,
                args=(self.__order_queue,
                      self.__order_socket,
                      self.log_file,
                      self.orientation))

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
                      self.orientation))

    def __order_executor(self,
                         ord_q: mp.Queue,
                         sock: socket.socket,
                         log_file: str,
                         orientation: Synchronized) -> None:

        while True:
            _cmd = ord_q.get(block=True)
            print(_cmd)
            print(ord_q)
            f = open(log_file, 'a')
            f.write(_cmd + " : Waiting for response\n")
            for _ in range(self.RETRY):
                sock.sendto(_cmd.encode('utf-8'), self.__drone_address)
                time.sleep(self.WAITING)
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
        # TODO: change mode depending on movement order

    def execute_order(self,
                      order: str) -> None:
        self.__order_queue.put(order)

    def __video_receiver(self,
                         img_q: mp.Queue,
                         debug=False) -> None:
        while True:
            frame = b''
            while True:
                byte_arr, _ = self.__video_socket.recvfrom(2048)
                frame += byte_arr
                if len(byte_arr) != 1460:
                    break

            img = np.asarray(frame, dtype=np.uint8)
            img_q.put(img)

            if debug:
                cv.imshow('frame', img)
                if cv.waitKey(1) == ord('q'):
                    break

    def __navigation_cycle(self,
                           img_q: mp.Queue,
                           ord_q: mp.Queue,
                           process_routine: Callable[[np.ndarray],
                                                     GateDescriptor],
                           orientation: Synchronized) -> None:

        img = img_q.get(block=True)
        _gate_descriptor = process_routine(img)
        # TODO: imgproc + choice of mode influences camera's matrix
        # TODO: implement navigation logic

    def run(self) -> None:
        if self.__use_order:
            self.execute_order("command")
            time.sleep(self.DELAY)
            self.execute_order("takeoff")
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
            self.video_receiver_worker.kill()
            time.sleep(self.DELAY)

        self.execute_order("streamoff")
        self.execute_order("land")
        time.sleep(self.STOP_DELAY)
        print("===== Goodbye =====")

        if self.order_worker.is_alive():
            self.order_worker.kill()
