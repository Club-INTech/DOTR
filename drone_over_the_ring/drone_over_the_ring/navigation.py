import yaml
from drone_state import DroneState
from typing import Tuple
from const import MAX_DRONE_SPEED, MAX_YAW_SPEED, MIN_DRONE_SPEED, MIN_YAW_SPEED


class Navigation():
    
    def __init__(self, navigation_config: str) -> None:
        
        self.__kpx = 10.0
        self.__krx = 15.0
        self.__kix = 0.1
        
        self.__kpy = 10.0
        self.__kry = 15.0
        self.__kiy = 0.1

        self.__kpz = 10.0
        self.__krz = 15.0
        self.__kiz = 0.1

        self.__kpyaw = 10.0
        self.__kryaw = 15.0
        self.__kiyaw = 0.1
        
        with open(navigation_config, 'r') as _stream:
            _conf = yaml.safe_load(_stream)
            coefs = ["kpx", "krx", "kix", "kpy", 
                     "kry", "kiy", "kpz", "krz", 
                     "kiz", "kpyaw", "kryaw", "kiyaw"]
            for _c in coefs:
                if _c in _conf:
                    self.__setattr__("_Navigation__" + _c, _conf[_c])
                    
    def update_speed(self, _ds: DroneState) -> Tuple[int, int, int, int]:
        
        _max_d = max([_ds.dx, _ds.dy, _ds.dz])
        raw_speed_x = self.__kpx * _ds.dx + self.__krx * (_ds.dx / _max_d) + self.__kix * _ds.sumX
        raw_speed_y = self.__kpy * _ds.dy + self.__kry * (_ds.dy / _max_d) + self.__kiy * _ds.sumY
        raw_speed_z = self.__kpz * _ds.dz + self.__krz * (_ds.dz / _max_d) + self.__kiz * _ds.sumZ
        raw_speed_yaw = self.__kpyaw * _ds.dyaw + self.__kryaw * (_ds.dyaw / _max_d) + self.__kiyaw * _ds.sumYaw

        speed_x = self.__sign(raw_speed_x) * int(min(MAX_DRONE_SPEED, max(MIN_DRONE_SPEED, abs(raw_speed_x))))
        speed_y = self.__sign(raw_speed_y) * int(min(MAX_DRONE_SPEED, max(MIN_DRONE_SPEED, abs(raw_speed_y))))
        speed_z = self.__sign(raw_speed_z) * int(min(MAX_DRONE_SPEED, max(MIN_DRONE_SPEED, abs(raw_speed_z))))
        speed_yaw = self.__sign(raw_speed_yaw) * int(min(MAX_YAW_SPEED, max(MIN_YAW_SPEED, abs(raw_speed_yaw))))
        
        return (speed_x, speed_y, speed_z, speed_yaw)
    
    def __sign(self, x) -> int:
        if x < 0:
            return -1
        else:
            return 1