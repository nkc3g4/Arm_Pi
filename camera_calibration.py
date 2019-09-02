#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# 摄像头和机械臂的位置校准


import cv2
import numpy as np
import time
import urllib
import threading
import signal
import LeArm
import kinematics as kin
import RPi.GPIO as GPIO

stream = None
bytes = ''
orgFrame = None
minFrame = None
Running = False
get_image_ok = False
# 校准按键
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
key = 22
GPIO.setup(key, GPIO.IN, GPIO.PUD_UP)
# 校准标志
correction_flag = False


# 暂停信号的回调
def cv_stop(signum, frame):
    global Running

    print("Stop face detection")
    if Running is True:
        Running = False
    cv2.destroyWindow('face_detection')
    cv2.destroyAllWindows()


# 继续信号的回调
def cv_continue(signum, frame):
    global stream
    global Running
    if Running is False:
        # 开关一下连接
        if stream:
            stream.close()
        #stream = urllib.urlopen("http://127.0.0.1:8080/?action=stream?dummy=param.mjpg")
	stream = urllib.urlopen("http://192.168.10.199:8081/video")
        bytes = ''
        Running = True


#   注册信号回调
signal.signal(signal.SIGTSTP, cv_stop)
signal.signal(signal.SIGCONT, cv_continue)


def get_image():
    global Running
    global orgFrame, minFrame
    global bytes
    global get_image_ok
    while True:
        if Running:
            try:
                bytes += stream.read(2048)  # 接收数据
                a = bytes.find('\xff\xd8')  # 找到帧头
                b = bytes.find('\xff\xd9')  # 找到帧尾
                if a != -1 and b != -1:
                    jpg = bytes[a:b + 2]  # 取出图片数据
                    bytes = bytes[b + 2:]  # 去除已经取出的数据
                    orgFrame = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.CV_LOAD_IMAGE_COLOR)  # 对图片进行解码
                    minFrame = cv2.resize(orgFrame, (320, 240), interpolation=cv2.INTER_LINEAR)  # 将图片缩放到 320*240
                    get_image_ok = True
            except Exception as e:
                print(e)
                continue
        else:
            time.sleep(0.01)


# 显示图像线程
th1 = threading.Thread(target=get_image)
th1.setDaemon(True)  # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
th1.start()


# 机械臂位置校准
def Arm_Pos_Corr():
    LeArm.setServo(1, 1200, 500)
    time.sleep(0.5)
    kin.ki_move(0, 2250, 200.0, 1500)


# 运行程序前按下KEY2,进入校准机械臂位置， 校准完成后，再按下KEY退出
run_corr_one = 0
cv_continue(1, 1)
# 初始化机械臂位置
LeArm.runActionGroup('rest', 1)
while True:
    if GPIO.input(key) == 0:
        time.sleep(0.1)
        if GPIO.input(key) == 0:
            correction_flag = not correction_flag
            if correction_flag is False:
                LeArm.runActionGroup('rest', 1)
    if correction_flag is False:
        run_corr_one = 0
        if minFrame is not None and get_image_ok:
            t1 = cv2.getTickCount()
            frame = minFrame
            img_h, img_w = frame.shape[:2]
            # 画图像中心点
            cv2.line(frame, ((img_w / 2) - 20, (img_h / 2)), ((img_w / 2) + 20, (img_h / 2)), (0, 0, 255), 1)
            cv2.line(frame, ((img_w / 2), (img_h / 2) - 20), ((img_w / 2), (img_h / 2) + 20), (0, 0, 255), 1)
            cv2.imshow('image', frame)
            cv2.waitKey(1)
            get_image_ok = False
            t2 = cv2.getTickCount()
            time_r = (t2 - t1) / cv2.getTickFrequency() * 1000
            # print("%sms" % time_r)
        else:
            time.sleep(0.01)
    else:
        if correction_flag and run_corr_one == 0:
            run_corr_one += 1
            Arm_Pos_Corr()
        else:
            time.sleep(0.01)


