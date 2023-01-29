MAX_DRONE_SPEED = 25.0  # maximum drone speed along x, y and z axes
MIN_DRONE_SPEED = 0.0  # minimum drone speed along x, y and z axes
MAX_YAW_SPEED = 20.0  # maximum drone rotation speed
MIN_YAW_SPEED = 0.0  # minimum drone rotation speed
GATE_NUMBER = 2  # number of gates to go through
MAX_INT_SUM = 40  # the maximum sum of integral

CAMERA_WIDTH = 960  # camera's width in pixels
CAMERA_HEIGHT = 720  # camera's height in pixels
DEFAULT_CAMERA_FOCAL = 991  # camera's default focal
EPS_EDGE = 10  # limit in pixel to consider if a bboxe is in edge effect

CIRCULAR_GATE_REAL_SIZE = 0.69  # real diameter of circular gate
SQUARE_GATE_REAL_SIZE = 1.0  # real diameter of square gate
HEX_GATE_REAL_SIZE = 0.665  # real diameter of hex gate

RETRY = 3  # send command retry count
WAITING = 2  # a waiting delay to execute rc 0 0 0 0 command before forawrd
FORWARD_WAITING = 4  # waiting to execute forward command
TAKEOFF_DELAY = 3  # a delay to tke on drone takeoff
DELAY = 0.5  # a delay to start a process
STOP_DELAY = 15  # drone stop delay

EPS_D = 0.1  # possible distance deviation in position before forward
EPS_YAW = 0.05  # yaw sing change value (previous 0.1)
FINAL_EPS_YAW = 0.1  # yaw angle deviation in position before forward
SAFE_DISTANCE = 1.8  # a limit safe detect distance from the center of the gate
OFFSET_Z = 0.35  # offset on the z axis (camera is pointed down)
NOT_DETECTED_LIMIT = 5  # a maximum number of non-detection before reset
