import numpy as np
import cv2 as cv

def camera_configuration():
    print("===== Welcome to camera cofiguration program =====")
    print("The output will be saved in ./config/camera_config.yaml")
    print("There is multiple configurations/matrixes which are applied depending on the drones action as tangential distortion changes")
    print("Place the chessboard in front of the drone and press any key to begin calibration...")

    _ = input()


