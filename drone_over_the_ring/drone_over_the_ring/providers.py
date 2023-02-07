from djitellopy import Tello
from typing import Union
import numpy as np
from abc import ABC, abstractmethod
import time


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

    def __init__(self, tello: Tello) -> None:
        self.tello = tello

    def connect(self) -> None:
        self.tello.connect()
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


try:
    import rclpy
    from rclpy.node import Node
    from tello_msgs.srv import TelloAction
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image


    class ROSOrderProvider(OrderProvider, Node):

        def __init__(self) -> None: 
            Node.__init__(self, "ros_order_client")
            self.cli = self.create_client(TelloAction, "drone1/tello_action")
            self.req = None

        def connect(self) -> None:
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for tello_action service")
            self.req = TelloAction.Request()

        def execute_order(self, _cmd: str) -> None:
            _cmd_list = _cmd.split(" ")
            _cmd = _cmd_list[0]
            for i in range(1, len(_cmd_list)):
                _cmd += " "
                _cmd += str(float(_cmd_list[i]) / 100.0)
            self.req.cmd = _cmd
            _ = self.cli.call_async(self.req)

        def takeoff(self) -> None:
            self.execute_order("takeoff")
            time.sleep(10)

        def forward(self, x: int) -> None:
            d = float(x)
            speed = 40
            t1 = d / speed
            self.execute_order("rc 0 40 0 0")
            time.sleep(t1)
            self.execute_order("rc 0 0 0 0")
            time.sleep(1)


    class ROSVideoProvider(VideoProvider, Node):

        def __init__(self) -> None:
            Node.__init__(self, "ros_video_provider")
            self.bridge = CvBridge()
            qos_profile = QoSProfile(
                    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                    depth=1)
            self.subscription = self.create_subscription(
                    Image,
                    "drone1/image_raw",
                    self.listener_callback,
                    qos_profile=qos_profile)
            self.frame = None

        def streamon(self) -> None:
            pass

        def init_frame_grabber(self) -> None:
            pass

        def streamoff(self) -> None:
            pass

        def get_frame(self) -> Union[np.ndarray, None]:
            return self.frame

        def listener_callback(self, msg):
            self.frame = self.bridge.imgmsg_to_cv2(
                    msg,
                    desired_encoding="passthrough")

    rclpy.init(args=None)

except ImportError:
    print("No ROS found")

    class ROSOrderProvider(OrderProvider):
        pass

    class ROSVideoProvider(VideoProvider):
        pass

    pass

