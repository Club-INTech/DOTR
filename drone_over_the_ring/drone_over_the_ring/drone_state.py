from dataclasses import dataclass
from enum import Enum
from gate_descriptor import GateDescriptor
from const import *
import numpy as np


class NavigationStep(Enum):
    """
        Enumeration allowing to track drone state.
        NOT_DETECTED
    """
    NOT_DETECTED = 1
    DETECTED = 2
    GO_THROUGH = 3


@dataclass(init=True, repr=True, eq=True, order=False)
class DroneState():
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
    prev_dyaw: float = 0.0
    dyaw: float = 0.0
    not_detected_count: int = 0
    gate_count: int = 0
    gate_navigation_step: NavigationStep \
        = NavigationStep.NOT_DETECTED
    
    def reset(self) -> None:
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
        self.dx = _desc.x
        self.dy = _desc.y
        self.dz = _desc.z - OFFSET_Z
        self.dyaw = self.yaw_sign * _desc.alpha
        self.sumX = min(abs(self.sumX + self.dx),MAX_INT_SUM) * self.__sign(self.sumX)
        self.sumY = min(abs(self.sumY + self.dy),MAX_INT_SUM) * self.__sign(self.sumY)
        self.sumZ = min(abs(self.sumZ + self.dz),MAX_INT_SUM) * self.__sign(self.sumZ)
        self.sumYaw = min(abs(self.sumYaw + self.dyaw),MAX_INT_SUM) * self.__sign(self.sumYaw)
        
    def check_yaw_sign(self) -> None:
        if abs(self.dyaw) - abs(self.prev_dyaw) >= EPS_YAW:
            self.dyaw = (-1) * self.dyaw
            self.yaw_sign = -1.0
            
    def is_at_safe_point(self) -> bool:
        return (self.dx**2)+(self.dy**2)+(self.dz**2) <= EPS_D and abs(self.dyaw) <= FINAL_EPS_YAW
    
    def update_safe_point(self) -> None:
        
        # change only y if drone is aligned with the gate
        if abs(self.dyaw) <= EPS_YAW:
            self.dx = self.dx
            self.dy = self.dy - SAFE_DISTANCE
        else:
            # slope of the perpendicular line
            _alpha = - (1 / self.dyaw)
            _beta = self.dy - _alpha * self.dx

            # sign of dyaw used to choose equation root
            _k = 1
            if self.dyaw < 0:
                _k = -1

            # resolve equation ax^2 + bx +c = 0
            a = (1 + _alpha ** 2)
            b = (2* self.dy / self.dyaw - 2 * _beta / self.dyaw - 2 * self.dx)
            c = (self.dx ** 2 + self.dy ** 2 + _beta ** 2 - 2 * _beta * self.dy - SAFE_DISTANCE ** 2)
            
            # roots of the equation
            x12 = np.roots([a, b, c])
            
            # root choice
            if _k < 0:
                self.dx = min(x12[0], x12[1])
            else:
                self.dx = max(x12[0], x12[1])
            self.dy = _alpha * self.dx + _beta
            
    def __sign(self, x) -> int:
        if x < 0:
            return -1
        else:
            return 1