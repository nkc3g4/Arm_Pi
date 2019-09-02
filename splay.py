import cv2
import numpy as np
import time
import urllib
import threading
import signal
import LeArm
import kinematics_apple as kin
import RPi.GPIO as GPIO
LeArm.setServo(1,800,500)
time.sleep(1.5)
