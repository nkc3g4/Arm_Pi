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
    #time.sleep(1)
    kin.ki_move(-1250, 3250, 800.0, 1500)
    #LeArm.setServo(7,2200,1500)
    time.sleep(2)

if (__name__ == "__main__"):
    main()
