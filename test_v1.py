from jetcam.csi_camera import CSICamera
import cv2
import os
from threading import Thread
import time

# camera module for capturing input data

camera = CSICamera(width=272, height=204, capture_width=3280, capture_height=2464, capture_fps=21, flip_method=2)
# camera = CSICamera(width=1632, height=924, capture_width=3264, capture_height=1848, capture_fps=30)
running = False
currentImage = [[[]]]
thread = None

def camera_start():
    global running, camera, thread
    if running == False:
        camera.running = True
        running = True
        def __capture():
            try:
                global running, camera, currentImage
                # update loop that constantly updates the most recent image which can be read at any time
                while running:
                    start = time.time()
                    currentImage = camera.value
                    time.sleep(max(0.0125-(time.time()-start), 0))
            except Exception as err:
                print(err)
        thread = Thread(target = __capture)
        thread.start()

def camera_stop():
    global running, camera, thread
    if running == True:
        running = False
        thread.join()
        camera.running = False

def read():
    global currentImage
    return currentImage


