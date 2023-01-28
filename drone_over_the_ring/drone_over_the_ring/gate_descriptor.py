from enum import Enum
from dataclasses import dataclass
import numpy as np
import const


class GateType(Enum):
    """
    Enum for the types of the gates that can be detected.
    There is a type for no gate detected.

    Args:
        Enum (_type_): a type of a gate
    """
    NO_GATE = 1
    CIRCLE_GATE = 2
    SQUARE_GATE = 3
    HEX_GATE = 4


@dataclass(init=True, repr=True, eq=True, order=False, unsafe_hash=False)
class GateDescriptor():
    """
    GateDescriptor class allows to save gate's parameters in real world
    (x, y, z for example) which can be used to control the drone and
    making it go through the chosen gate.

    Attributes
    ----------
    x : float
        A relative position of the drone across x axis
    y : float
        A relative position of the drone across y axis
    z : float
        A relative position of the drone across z axis
    alpha : float
        A relative  angle of rotation around z aix of the drone
    distance: float
        A distance from the drone to the gate
    pixel_width: int
        A width of the gate in pixels
    pixel_height:
        A height of the gate in pixels
    type_: GateType
        A type of the gate
    score: float
        A maximum match score

    Methods
    -------
    get_ratio():
        Calculates a ratio width to height in pixels in order to
        find the rotation angle
    set_xyz_from_image(x_min, y_min, x_max, y_max):
        Allows to calculate all gate's parameters: x, y, z, alpha and distance
    """

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
        """
        Calculates a width to height ratio which cannot be greater then one
        (false detections)

        Returns:
            float: a ratio leveled off to one
        """

        return min(self.pixel_width / self.pixel_height, 1.0)

    def set_xyz_from_image(self,
                           x_min: int,
                           y_min: int,
                           x_max: int,
                           y_max: int) -> None:
        """
        Calculates gate's parameters, such as x, y, z, distance and alpha using
        pxiel-to-real-size ratio and detected gate metrics.

        Args:
            x_min (int): minimal x of detected gate in pixels
            y_min (int): minimal y of detected gate in pixels
            x_max (int): maximal x of detected gate in pixels
            y_max (int): maximal y of detected gate in pixels
        """

        self.alpha = np.arccos(self.get_ratio())
        real_ratio = const.CIRCULAR_GATE_REAL_SIZE / (y_max - y_min)  # pixel-to-real-size ratio
        self.distance = const.DEFAULT_CAMERA_FOCAL * real_ratio
        self.x = real_ratio * ((x_min + x_max) / 2.0 - const.CAMERA_WIDTH / 2.0)
        self.z = (-1.0) * real_ratio * ((y_min + y_max) / 2.0 - const.CAMERA_HEIGHT / 2.0)
        self.y = np.sqrt(self.distance**2 - self.x**2 - self.z**2)

