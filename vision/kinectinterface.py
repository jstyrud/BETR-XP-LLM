""" Minimal interface to connect to Kinect camera """
import cv2

class KinectInterface:
    """
    Class for interfacing with the kinect camera
    """
    def __init__(self):
        self.cam = cv2.VideoCapture(1)

    def save_image(self):
        """ Saves the image from the camera """
        if self.cam:
            result, image = self.cam.read()
            if result:
                cv2.imwrite("capture.jpg", image)
            else:
                print("Error, no image received")

    def get_image(self):
        """ Returns image from the camera """
        if self.cam:
            result, image = self.cam.read()
            if result:
                return image
            else:
                return None

    def show_image(self):
        """ Shows image from camera """
        if self.cam:
            result, image = self.cam.read()
            if result:
                cv2.imshow("capture", image)
                cv2.waitKey(0)
                cv2.destroyWindow("capture")
