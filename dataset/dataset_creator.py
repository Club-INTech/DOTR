import cv2
import time
from multiprocessing import Process, Queue
from djitellopy import Tello
import logging
import os

# Path to save the images
PATH_TO_SAVE = r'D:\DATASET_Drone'

# Function to manage the increment of the index of the images for the name
def index(path):
    num = len(os.listdir(path))
    output = str('{:06d}'.format(num))
    return output
    
tello = Tello()
tello.connect()

tello.streamon()
frame_read = tello.get_frame_read()

while True:
    img = frame_read.frame
    cv2.imshow("drone", img)

    key = cv2.waitKey(1) & 0xff
    if key == 27: # ESC
        break
    elif key == ord('i'):
        cv2.imwrite(PATH_TO_SAVE + r'\image'+str(index(PATH_TO_SAVE))+r'.jpg', img)
