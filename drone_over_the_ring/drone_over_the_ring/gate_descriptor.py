from enum import Enum
from dataclasses import dataclass


CIRCULAR_GATE_REAL_SIZE = 1.0
SQUARE_GATE_REAL_SIZE = 1.0
HEX_GATE_REAL_SIZE = 1.0


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
    pixel_width: int = 1
    pixel_height: int = 1
    type_: GateType = GateType.NO_GATE

    def get_ratio(self) -> float:
        return self.pixel_width / self.pixel_height
