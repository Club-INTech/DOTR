from djitellopy import Tello
from typing import Union
import numpy as np
from abc import ABC, abstractmethod


class OrderProvider(ABC):

    @abstractmethod
    def connect(self) -> None:
        pass

    @abstractmethod
    def takeoff(self) -> None:
        pass

    @abstractmethod
    def forward(self, x: int) -> None:
        pass

    @abstractmethod
    def execute_order(self, _cmd: str) -> None:
        pass


class VideoProvider(ABC):

    @abstractmethod
    def init_frame_grabber(self) -> None:
        pass

    @abstractmethod
    def streamon(self) -> None:
        pass

    @abstractmethod
    def streamoff(self) -> None:
        pass

    @abstractmethod
    def get_frame() -> Union[np.ndarray, None]:
        pass


class BasicOrderProvider(OrderProvider):

    def __init__(self) -> None:
        pass

    def connect(self) -> None:
        print("Connected to drone")

    def takeoff(self) -> None:
        print("Executing takeoff")

    def forward(self, x: int) -> None:
        print("Executing forward " + str(x))

    def execute_order(self, _cmd: str) -> None:
        print("Executing " + _cmd)


class TelloOrderProvider(OrderProvider):

    def __init__(self, tello: Tello) -> None:
        self.tello = tello

    def connect(self) -> None:
        self.tello.connect()

    def takeoff(self) -> None:
        self.tello.takeoff()

    def forward(self, x: int) -> None:
        self.tello.move_forward(x)

    def execute_order(self, _cmd: str) -> None:
        self.tello.send_command_without_return(_cmd)


class ROSOrderProvider(OrderProvider):
    pass


class EmptyVideoProvider(VideoProvider):

    def __init__(self) -> None:
        pass

    def streamon(self) -> None:
        print("Putting stream on")

    def streamoff(self) -> None:
        print("Putting stream off")

    def init_frame_grabber(self) -> None:
        pass

    def get_frame(self) -> Union[np.ndarray, None]:
        return None


class TelloVideoProvider(VideoProvider):

    def __init__(self, tello: Tello) -> None:
        self.tello = tello

    def streamon(self) -> None:
        self.tello.streamon()

    def streamoff(self) -> None:
        self.tello.streamoff()

    def init_frame_grabber(self) -> None:
        self.frame_grabber = self.tello.get_frame_read()

    def get_frame(self) -> Union[np.ndarray, None]:
        return self.frame_grabber.frame


class ROSVideoProvider(VideoProvider):
    pass

