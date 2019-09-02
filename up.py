import cv2
import numpy as np
import time
import urllib
import threading
import signal
import LeArm
import kinematics_apple as kin
import RPi.GPIO as GPIO
def main():
    time.sleep(1)
    LeArm.setServo(7,2200,1500)
    time.sleep(2)

if (__name__ == "__main__"):
    main()
