from dataclasses import dataclass
from enum import Enum
from gate_descriptor import GateDescriptor
import const
import numpy as np


class NavigationStep(Enum):
    """
    Enum of possible navigation steps. Either no gate is detected
    or a gate was detected

    Attributes
    ----------
    NOT_DETECTED: int = 1
        no gate was detected
    DETECTED: int = 2
        a gate was detected
    """

    NOT_DETECTED = 1
    DETECTED = 2


@dataclass(init=True, repr=True, eq=True, order=False)
class DroneState():
    """
    Drone state class used to store drone to gate relative
    position, yaw sign, speeds across every axis, sums of
    relative distances and other parameters. It has some
    helpers method allowing to simplify navigation.

    Attributes
    ----------

    vx: int = 0
        speed across x
    vy: int = 0
        speed across y
    vz: int = 0
        speed across z
    vyaw: int = 0
        speed around z axis
    dx: float = 0.0
        relative distance on x
    dy: float = 0.0
        relative distance on y
    dz: float = 0.0
        relative distance on z
    sumX: float = 0.0
        sum of distances on x
    sumY: float = 0.0
        sum of distances on y
    sumZ: float = 0.0
        sum of distances on z
    sumYaw: float = 0.0
        sum of yaw angles
    yaw_sign: float = 1.0
        yaw angle sign
    prev_dyaw: float = 0.0
        previous yaw
    dyaw: float = 0.0
        yaw (rotation angle around z axis)
    not_detected_count: int = 0
        counter of the number of times the gate was not detected
        consecutively
    gate_count: int = 0
        a count of crossed gates
    gate_navigation_step: NavigationStep = NavigationStep.NOT_DETECTED
        a step of gate navigation

    Methods
    -------
    reset():
        resets all drone state's parameters
    set_position(_desc):
        sets a reltive position and other parameters according to
        the gate descriptor passed in as an argument
    check_yaw_sign():
        checks if yaw sign is good and changes it if not
    is_at_safe_point():
        checks if drone is in safe point and can to move forward
    update_safe_point():
        calculates a safe point in which drone is capable to
        forward and cross the gate
    __sign(x):
        gives a sign of x
    """

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
    yaw_sign: float = 1.0
    prev_dyaw: float = 10.0
    dyaw: float = 0.0
    not_detected_count: int = 0
    gate_count: int = 0
    gate_navigation_step: NavigationStep \
        = NavigationStep.NOT_DETECTED

    def reset(self) -> None:
        """
        Allows to reset drone state setting speeds, accumulated
        sums and not detected count to 0 and navigation step to
        NOT_DETECTED.
        """
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.vyaw = 0
        self.sumX = 0
        self.sumY = 0
        self.sumZ = 0
        self.sumYaw = 0
        self.gate_navigation_step = NavigationStep.NOT_DETECTED
        self.not_detected_count = 0

    def set_position(self, _desc: GateDescriptor) -> None:
        """_summary_

        Args:
            _desc (GateDescriptor): _description_
        """
        self.dx = _desc.x
        self.dy = _desc.y
        self.dz = _desc.z - const.OFFSET_Z  # offset as camera is pointed down
        self.dyaw = self.yaw_sign * _desc.alpha
        self.sumX = min(abs(self.sumX + self.dx), const.MAX_INT_SUM) \
            * self.__sign(self.sumX)  # restriction on maximal sign
        self.sumY = min(abs(self.sumY + self.dy), const.MAX_INT_SUM) \
            * self.__sign(self.sumY)
        self.sumZ = min(abs(self.sumZ + self.dz), const.MAX_INT_SUM) \
            * self.__sign(self.sumZ)
        self.sumYaw = min(abs(self.sumYaw + self.dyaw), const.MAX_INT_SUM) \
            * self.__sign(self.sumYaw)

    def check_yaw_sign(self) -> None:
        if const.MIN_EPS_YAW <= abs(self.dyaw) - abs(self.prev_dyaw) <= const.MAX_EPS_YAW:
            print(str(self.dyaw) + " " + str(self.prev_dyaw))
            self.dyaw = (-1) * self.dyaw
            self.yaw_sign = -1.0 * self.yaw_sign

    def is_at_safe_point(self) -> bool:
        return (self.dx**2) + (self.dz**2) <= const.EPS_D \
                and abs(self.dyaw) <= const.FINAL_EPS_YAW \
                and const.MIN_APPROACH <= self.dy <= const.MAX_APPROACH

    def update_safe_point(self) -> None:
        self.dy = self.dy - const.SAFE_DISTANCE

    def __sign(self, x) -> int:
        if x < 0:
            return -1
        else:
            return 1
