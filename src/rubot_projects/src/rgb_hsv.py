#!/usr/bin/env python3
import numpy as np
import cv2


class BGR_HSV(object):


    def __init__(self):
        pass

    def rgb_hsv(self, rgb):
        assert len(rgb) == 3, "RGB has to have 3 components"
        bgr = [rgb[2], rgb[1], rgb[0]]
        bgr_numpy = np.uint8([[bgr]])
        hsv_numpy = cv2.cvtColor(bgr_numpy, cv2.COLOR_BGR2HSV)
        hsv_numpy_percentage = [hsv_numpy[0][0][0] / 179.0, hsv_numpy[0][0][1] / 255.0, hsv_numpy[0][0][2] / 255.0]
        return hsv_numpy[0][0], hsv_numpy_percentage


if __name__ == '__main__':
    bgr_hsv = BGR_HSV()
    R = int(input("R="))
    G = int(input("G="))
    B = int(input("B="))
    rgb_now = [R, G, B]
    hsv_numpy_now, hsv_numpy_percentage_now = bgr_hsv.rgb_hsv(rgb=rgb_now)
    print(hsv_numpy_now)
    print(hsv_numpy_percentage_now)