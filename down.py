import cv2
import numpy as np
import time
import urllib
import threading
import signal
import LeArm
import kinematics_apple as kin
import RPi.GPIO as GPIO
#LeArm.setServo(5,1360,500)
def main():
    time.sleep(0.5)
    LeArm.setServo(7,500,1500)
    time.sleep(2)
if (__name__ == "__main__"):
    main()

