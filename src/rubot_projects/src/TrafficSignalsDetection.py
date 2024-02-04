#!/usr/bin/env python3
import cv2
import numpy as np


def signal_detected(photo):
    image = cv2.imread(photo)
    cv2.imshow('Signal',image)
    signal = "left"
    return signal

if __name__ == '__main__':
    # Read signal
    #image = cv2.imread('left.png')
    photo = "left.png"
    signal = signal_detected(photo)
    print("Siganl detected: ", signal)
    #cv2.imshow('Signal',image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
