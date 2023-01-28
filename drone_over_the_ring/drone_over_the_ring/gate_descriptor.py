from enum import Enum
from dataclasses import dataclass
import numpy as np
from camera import CAMERA_HEIGHT, CAMERA_WIDTH, CAMERA_FOCAL

CIRCULAR_GATE_REAL_SIZE = 0.69
SQUARE_GATE_REAL_SIZE = 1.0
HEX_GATE_REAL_SIZE = 0.665


class GateType(Enum):
    NO_GATE = 1
    CIRCLE_GATE = 2
    SQUARE_GATE = 3
    HEX_GATE = 4


@dataclass(init=True, repr=True, eq=True, order=False, unsafe_hash=False)
class GateDescriptor():
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    alpha: float = 0.0
    distance: float = 0.0
    pixel_width: int = 1
    pixel_height: int = 1
    type_: GateType = GateType.NO_GATE
    score: float = 0.0
    
    def get_ratio(self) -> float:
        return min(self.pixel_width / self.pixel_height, 1.0)
    
    def set_xyz_from_image(self, x_min, y_min, x_max, y_max):
        self.alpha = np.arccos(self.get_ratio())
        real_ratio = CIRCULAR_GATE_REAL_SIZE / (x_max - x_min)
        self.distance = CAMERA_FOCAL * real_ratio
        self.x = real_ratio * ( (x_min + x_max) / 2 - CAMERA_WIDTH/2 )
        self.z = -real_ratio * ( (y_min + y_max) / 2 - CAMERA_HEIGHT/2 ) 
        self.y = np.sqrt(self.distance**2 - self.x**2 - self.z**2)