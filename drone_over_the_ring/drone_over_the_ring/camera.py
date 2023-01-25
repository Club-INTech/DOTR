import cv2 as cv
from detection_for_calib import GateDetector
#from djitellopy import Tello

def camera_configuration():
    print("===== Welcome to camera cofiguration program =====")
    print("The output will be saved in ./config/camera_config.yaml")
    print("There is multiple configurations/matrixes which are applied depending on the drones action as tangential distortion changes")
    print("Place the chessboard in front of the drone and press any key to begin calibration...")

    _ = input()




# def find_focal():
#     """_summary_

#     """
#     distance = 2.85
#     CIRCULAR_GATE_REAL_SIZE = 0.69

#     tello = Tello()
#     tello.connect()
#     tello.streamon()

#     frame_read = tello.get_frame_read()

#     gate_detector = GateDetector(weights="/home/gaetan/DOTR/drone_over_the_ring/drone_over_the_ring/config/wheights_training.pt")

#     im0, gate = gate_detector.run(frame_read.frame)
#     gate = gate.cpu()
#     pixel_height = gate[0][3] - gate[0][1]
#     print(pixel_height)
#     f = distance * pixel_height / CIRCULAR_GATE_REAL_SIZE

#     print(gate)

#     print(f)
#     cv.imshow('osef', im0)
#     cv.waitKey(10000)

# find_focal()
