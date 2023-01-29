import time
import sys
import numpy as np
import keyboard
import multiprocessing as mp
from multiprocessing.connection import Connection
import cv2 as cv
from typing import Callable, Tuple, Union
from gate_descriptor import GateDescriptor, GateType
import threading
import providers as prs
import const
from navigation import Navigation
from drone_state import DroneState, NavigationStep


class Drone():

    def __init__(self,
                 img_process_routine: Callable[[np.ndarray],
                                               Tuple[np.ndarray,
                                                     GateDescriptor]],
                 navigation_config: str = "./config/default_nav_config.yaml",
                 use_navigation: bool = True,
                 use_control: bool = False,
                 order_provider: Union[prs.OrderProvider, None] = None,
                 video_provider: Union[prs.VideoProvider, None] = None,
                 ) -> None:

        self.__img_process_routine = img_process_routine
        self.__use_navigation = use_navigation
        self.__use_control = use_control
        self.__order_provider = order_provider
        self.__video_provider = video_provider

        self.__parent_conn, self.__child_conn = mp.Pipe()
        self.__video_lock = threading.Lock()
        self.__order_queue = mp.Queue(maxsize=10)

        self.__stop = False
        self.tello = None

        self.prev_t = 0

        if self.__order_provider is not None:
            self.__order_provider.connect()
        else:
            print("Order provider is None")
            sys.exit(1)
        if self.__video_provider is not None:
            self.__video_provider.streamon()
        else:
            print("Video provider is None")
            sys.exit(1)

        self.__navigator = Navigation(navigation_config)
        self.__drone_state = DroneState()

        self.order_worker = mp.Process(
                target=self.__order_executor,
                args=(self.__order_provider,
                      self.__order_queue))

        self.video_receiver_worker = mp.Process(
                target=self.__video_receiver,
                args=(self.__video_provider,
                      self.__child_conn,
                      self.__video_lock))

    def __clear_video_conn(self):
        while self.__parent_conn.poll():
            _ = self.__parent_conn.recv()

    def __order_executor(self,
                         order_provider: prs.OrderProvider,
                         order_queue: mp.Queue) -> None:

        while True:
            _cmd = order_queue.get(block=True)
            order_provider.execute_order(_cmd)

    def execute_order(self,
                      order: str) -> None:
        if time.time_ns() - self.prev_t >= const.ORDER_DELAY:
            self.__order_queue.put(order)
            self.prev_t = time.time_ns()

    def __video_receiver(self,
                         video_provider: prs.VideoProvider,
                         conn: Connection,
                         video_lock: threading.Lock) -> None:

        video_provider.init_frame_grabber()
        while True:
            if not video_lock.locked():
                conn.send(video_provider.get_frame())

    def run(self) -> None:

        self.order_worker.start()
        time.sleep(const.DELAY)
        self.video_receiver_worker.start()
        time.sleep(const.DELAY)
        self.__order_provider.takeoff()
        time.sleep(const.TAKEOFF_DELAY)
        self.__order_provider.execute_order("up 60")
        time.sleep(const.TAKEOFF_DELAY)

        if self.__use_navigation:

            while not self.__stop:
                img = self.__parent_conn.recv()
                if img is None:
                    continue
                _img, _desc = self.__img_process_routine(img)

                if self.__drone_state.gate_count >= const.GATE_NUMBER:
                    self.__stop = True
                    break

                data1 = "safe_distance: {s:.3f}/dx: {x:.3f}/dy: {y:.3f}/dz: {z:.3f}/dyaw: {dyaw:.3f}/".format(
                                s=const.SAFE_DISTANCE,
                                x=self.__drone_state.dx,
                                y=self.__drone_state.dy,
                                z=self.__drone_state.dz,
                                dyaw=self.__drone_state.dyaw,
                                prev_dyaw=self.__drone_state.prev_dyaw)
                data2 = "vx: {vx}/vy: {vy}/vz: {vz}/vyaw: {vyaw:.3f}/not_detected_count: {ndc}/gate_count: \{gc}/gate_detection_step: {gds}" .format(
                            vx=self.__drone_state.vx,
                            vy=self.__drone_state.vy,
                            vz=self.__drone_state.vz,
                            vyaw=self.__drone_state.vyaw,
                            ndc=self.__drone_state.not_detected_count,
                            gc=self.__drone_state.gate_count,
                            gds=str(self.__drone_state.gate_navigation_step))
                data = data1 + data2
                row_start = 500
                row_step = 15
                row = row_start
                column = 700
                for _, _d in enumerate(data.split("/")):
                    cv.putText(img=_img, text=_d,
                               org=(column, row), fontFace=cv.FONT_HERSHEY_TRIPLEX,
                               fontScale=0.5, color=(0, 255, 0), thickness=1)
                    row += row_step
                cv.imshow("frame", _img)
                if cv.waitKey(1) == ord('q'):
                    self.__stop = True

                if _desc.type_ == GateType.NO_GATE:
                    self.__drone_state.not_detected_count += 1
                    if self.__drone_state.not_detected_count \
                            >= const.NOT_DETECTED_LIMIT:

                        self.execute_order("rc 0 0 0 0")
                        self.__drone_state.reset()

                    else:
                        continue
                else:
                    self.__drone_state.not_detected_count = 0
                    if self.__drone_state.gate_navigation_step \
                            == NavigationStep.NOT_DETECTED:

                        self.execute_order("rc 0 0 0 0")
                        self.__drone_state.reset()
                        self.__drone_state.gate_navigation_step \
                            = NavigationStep.DETECTED

                    self.__drone_state.set_position(_desc)

                if self.__drone_state.gate_navigation_step \
                        == NavigationStep.NOT_DETECTED:

                    self.execute_order("rc 0 0 0 40")
                    self.__drone_state.reset()
                    self.__drone_state.vyaw = 40

                elif self.__drone_state.gate_navigation_step \
                        == NavigationStep.DETECTED:

                    self.__drone_state.check_yaw_sign()
                    self.__drone_state.update_safe_point()

                    if self.__drone_state.is_at_safe_point():
                        self.execute_order("rc 0 0 0 0")
                        time.sleep(const.WAITING)
                        self.__order_provider.forward(
                                int(1.2 * const.SAFE_DISTANCE*100))
                        self.__drone_state.reset()
                        time.sleep(const.FORWARD_WAITING)
                        self.__video_lock.acquire()
                        self.__clear_video_conn()
                        self.__video_lock.release()
                        self.__drone_state.gate_count += 1
                        continue

                    speed_x, speed_y, speed_z, speed_yaw \
                        = self.__navigator.update_speed(self.__drone_state)

                    self.__drone_state.vx = speed_x
                    self.__drone_state.vy = speed_y
                    self.__drone_state.vz = speed_z
                    self.__drone_state.vyaw = speed_yaw

                    _cmd = "rc {speed_x} {speed_y} {speed_z} {speed_yaw}".format(speed_x=speed_x,
                                                          speed_y=speed_y,
                                                          speed_z=speed_z,
                                                          speed_yaw=speed_yaw)
                    self.execute_order(_cmd)
                    self.__drone_state.prev_dyaw = self.__drone_state.dyaw

        elif self.__use_control:
            state_vector = [0.0, 0.0, 0.0, 0.0]
            while not self.__stop:

                _img = self.__parent_conn.recv()
                if _img is not None:
                    cv.imshow("frame", _img)
                    if cv.waitKey(0) == ord("q"):
                        self.__stop = True
                        break

                event = keyboard.read_event()

                if event.event_type == keyboard.KEY_DOWN and event.name == "q":
                    self.__stop = True
                    break

                if event.event_type == keyboard.KEY_DOWN and event.name == "a":
                    if state_vector[0] == 0:
                        state_vector[0] = -const.MAX_DRONE_SPEED
                elif event.event_type == keyboard.KEY_UP and event.name == "a":
                    if state_vector[0] == -const.MAX_DRONE_SPEED:
                        state_vector[0] = 0

                if event.event_type == keyboard.KEY_DOWN and event.name == "d":
                    if state_vector[0] == 0:
                        state_vector[0] = const.MAX_DRONE_SPEED
                elif event.event_type == keyboard.KEY_UP and event.name == "d":
                    if state_vector[0] == const.MAX_DRONE_SPEED:
                        state_vector[0] = 0

                if event.event_type == keyboard.KEY_DOWN and event.name == "w":
                    if state_vector[1] == 0:
                        state_vector[1] = const.MAX_DRONE_SPEED
                elif event.event_type == keyboard.KEY_UP and event.name == "w":
                    if state_vector[1] == const.MAX_DRONE_SPEED:
                        state_vector[1] = 0

                if event.event_type == keyboard.KEY_DOWN and event.name == "s":
                    if state_vector[1] == 0:
                        state_vector[1] = -const.MAX_DRONE_SPEED
                elif event.event_type == keyboard.KEY_UP and event.name == "s":
                    if state_vector[1] == -const.MAX_DRONE_SPEED:
                        state_vector[1] = 0

                if event.event_type == keyboard.KEY_DOWN and event.name == "i":
                    if state_vector[2] == 0:
                        state_vector[2] = const.MAX_DRONE_SPEED
                elif event.event_type == keyboard.KEY_UP and event.name == "i":
                    if state_vector[2] == const.MAX_DRONE_SPEED:
                        state_vector[2] = 0

                if event.event_type == keyboard.KEY_DOWN and event.name == "k":
                    if state_vector[2] == 0:
                        state_vector[2] = -const.MAX_DRONE_SPEED
                elif event.event_type == keyboard.KEY_UP and event.name == "k":
                    if state_vector[2] == -const.MAX_DRONE_SPEED:
                        state_vector[2] = 0

                if event.event_type == keyboard.KEY_DOWN and event.name == "j":
                    if state_vector[3] == 0:
                        state_vector[3] = -const.MAX_YAW_SPEED
                elif event.event_type == keyboard.KEY_UP and event.name == "j":
                    if state_vector[3] == -const.MAX_YAW_SPEED:
                        state_vector[3] = 0

                if event.event_type == keyboard.KEY_DOWN and event.name == "l":
                    if state_vector[3] == 0:
                        state_vector[3] = const.MAX_YAW_SPEED
                elif event.event_type == keyboard.KEY_UP and event.name == "l":
                    if state_vector[3] == const.MAX_YAW_SPEED:
                        state_vector[3] = 0

                _cmd = "rc {speed_x} {speed_y} {speed_z} {speed_yaw}".format(speed_x=int(state_vector[0]),
                                            speed_y=int(state_vector[1]),
                                            speed_z=int(state_vector[2]),
                                            speed_yaw=int(state_vector[3]))
                self.execute_order(_cmd)
                time.sleep(const.DELAY)

        else:
            while not self.__stop:
                img = self.__parent_conn.recv()
                _img, _ = self.__img_process_routine(img)
                cv.imshow("frame", _img)
                if cv.waitKey(1) == ord("q"):
                    self.__stop = True
                    break

        self.__video_provider.streamoff()
        time.sleep(const.DELAY)
        self.execute_order("rc 0 0 0 0")
        self.execute_order("land")
        time.sleep(const.STOP_DELAY)

        self.video_receiver_worker.kill()
        self.order_worker.kill()
        cv.destroyAllWindows()
        self.__child_conn.close()
        self.__parent_conn.close()

    def stop(self) -> None:
        self.__stop = True
