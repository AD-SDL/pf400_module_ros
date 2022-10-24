import cv2

from time import sleep
from datetime import datetime
from threading import Thread

class CameraModuleDriver():

    def __init__(self):

        self.camera = cv2.VideoCapture(0)
    
        self.stop_camera = False


    def capture_image(self):
        result, image = self.camera.read()

        if result:
            cv2.imshow("plate_image", image)
            now = datetime.now()
            date = now.date()
            time = now.time()
            print("Saving plate output image.")
            cv2.imwrite("output_plate_image_" + str(date) + "_"+ str(time) + ".png", image)
            cv2.waitKey(0)
            cv2.destroyWindow("plate_image")
        else:
            print("No image detected, trying again ...")

if __name__ == "__main__":

    cam = CameraModuleDriver()
    cam.capture_image()
    


