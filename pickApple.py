#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# 搬运色块_1

import cv2
import numpy as np
import time
import urllib
import threading
import signal
import LeArm
import kinematics_apple as kin
import RPi.GPIO as GPIO
import sys
import os
#import down
#import up

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

# 要识别的颜色字典
color_dist = {'red': {'Lower': np.array([0, 160, 160]), 'Upper': np.array([6, 255, 255])},
              #'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([35, 50, 40]), 'Upper': np.array([90, 255, 255])},
              }
# 色块颜色，位置列表
position_color_list = []
# 识别到色块标志
cv_blocks_ok = False
# 搬运步骤
step = 0
num_random = None
# 识别次数
cv_count = 0
# 用于判读色块是否稳定
last_blocks = []
last_x = 0
stable = False
# 存储色块， 用于判读色块 Y轴的远近， 机械臂先取近的
storage_blocks = []


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
		    cropped = orgFrame[0:330,100:540]
                    minFrame = cv2.resize(cropped, (320, 240), interpolation=cv2.INTER_LINEAR)  # 将图片缩放到 320*240
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


# 数值映射
# 将一个数从一个范围映射到另一个范围
def leMap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def move_blocks():
    global cv_blocks_ok, position_color_list, step
    while True:
        if cv_blocks_ok is True:
            if len(position_color_list) == 1:    #
                # 数据处理
                print position_color_list, 'pos'
                if step == 0:
                    x_pix_cm = position_color_list[0][1]
                    y_pix_cm = position_color_list[0][2]
                    angle = position_color_list[0][3]
                    # 数据映射-1250 1250
                    n_x = int(leMap(x_pix_cm, 0.0, 320.0, -2050.0, 2050.0)) * 1.0
                    n_y = int(leMap(240 - y_pix_cm, 0.0, 240.0, 1050, 3550.0)) * 1.0 #1250 3250
		    print("n_x,n_y:",n_x,n_y)
                    # 需要根据实际情况调整，偏差主要来自舵机的虚位
                    #if n_x < -100:
                    #    n_x -= 120  # 偏差
		    #if n_x < -500:
			#n_x -= 120
	            if n_x < -1250:
			n_x = 1250
		    if n_x > 1250:
  			n_x =1250
	            if n_y > 1600:
			n_y += 300
                    #if n_y > 2400:
			#n_y +=300
                    if n_y < 1250:
                        n_y = 1250
		    if n_y > 3250:
			n_y = 3250
                    print("Adjusted n_x,n_y",n_x,n_y) 
                    LeArm.setServo(1, 700, 500)
                    time.sleep(1.5)
                    step = 1
                elif step == 1:
                    # 机械臂下去
                    if kin.ki_move(n_x, n_y, 900.0, 1500):
                        step = 2
                    else:
                        step = 6
                elif step == 2:
                    # 根据方块的角度转动爪子
                    '''if angle <= -45:
                        angle = -(90 + angle)
                    n_angle = leMap(angle, 0.0, -45.0, 1500.0, 1750.0)
                    if n_x > 0:
                        LeArm.setServo(2, 3000 - n_angle, 500)
                    else:
                        LeArm.setServo(2, n_angle, 500)
                    time.sleep(0.5)'''
                    step = 3
                elif step == 3:
                    # 抓取
                    print '3 ok'
                    LeArm.setServo(1, 1510, 500)
                    time.sleep(1.5)
                    step = 4
                elif step == 4:     # 将方块提起
                    print '4 ok'
                    kin.ki_move(n_x, n_y, 1000.0, 1000)
                    step = 5
                elif step == 5:     # 抓取成功，放置方块
                    print '5 ok'
                    if position_color_list[0][0] == 'red':
                        LeArm.runActionGroup('hold', 1)
                    #elif position_color_list[0][0] == 'blue':
                        #LeArm.runActionGroup('blue', 1)
                    elif position_color_list[0][0] == 'green':
                        LeArm.runActionGroup('hold', 1)
                    #step = 6
		    #sys.exit()
                    #raise Exception('I know Python!')
		    #LeArm.setServo(7,600,500)
	            #up.main() 
		    #time.sleep(0.5)
                    os._exit(1)
                elif step == 6:     # 复位机械臂
                    print '6 ok'
                    LeArm.runActionGroup('rest', 1)
                    threadLock.acquire()
                    position_color_list = []
                    cv_blocks_ok = False
                    threadLock.release()
                    step = 0
        else:
            time.sleep(0.01)


# 启动动作在运行线程
th2 = threading.Thread(target=move_blocks)
#th2.setDaemon(True)
th2.start()

# 线程锁
threadLock = threading.Lock()


# 镜头畸变系数
lens_mtx = np.array([
                [993.17745922, 0., 347.76412756],
                [0., 992.6210587, 198.08924031],
                [0., 0., 1.],
           ])
lens_dist = np.array([[-2.22696961e-01, 3.34897836e-01, 1.43573965e-03, -5.99140365e-03, -2.03168813e+00]])


# 镜头畸变调整
def lens_distortion_adjustment(image):
    global lens_mtx, lens_dist
    h, w = image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(lens_mtx, lens_dist, (w, h), 0, (w, h))  # 自由比例参数
    dst = cv2.undistort(image, lens_mtx, lens_dist, None, newcameramtx)
    return dst


# 机械臂位置校准
def Arm_Pos_Corr():
    LeArm.setServo(1, 1200, 500)
    time.sleep(0.5)
    kin.ki_move(0, 2250, 800.0, 1500)


# 运行程序前按下KEY2,进入校准机械臂位置， 校准完成后，再按下KEY退出
run_corr_one = 0
cv_continue(1, 1)
time.sleep(0.1)
#down.main()
#LeArm.setServo(7,700,600)
time.sleep(0.5)
# 初始化机械臂位置
LeArm.runActionGroup('rest', 1)
timeout = time.time() + 440 #Timeout Exit

while True:
    if time.time() > timeout:
	os._exit(1)
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
            frame = lens_distortion_adjustment(minFrame)
            img_h, img_w = frame.shape[:2]
            # 获取图像中心点坐标x, y
            img_center_x = img_w / 2
            img_center_y = img_h / 2
            if cv_blocks_ok is False:
                # 高斯模糊
                gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)
                # 转换颜色空间
                hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)
                print("HSV at center:",hsv[img_center_y,img_center_x])
                for i in color_dist:
                    # 查找字典颜色
                    mask = cv2.inRange(hsv, color_dist[i]['Lower'], color_dist[i]['Upper'])
                    # 腐蚀
                    mask = cv2.erode(mask, None, iterations=2)
                    # 膨胀
                    kernel = np.ones((5, 5), np.uint8)
                    mask = cv2.dilate(mask, kernel, iterations=2)
                    # 查找轮廓
                    # cv2.imshow('mask', mask)
                    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                    if len(cnts) > 0:
                        # 找出最大的区域
                        c = max(cnts, key=cv2.contourArea)
                        # 返回的值 中心坐标（x, y）,（w，h）,角度
                        rect = cv2.minAreaRect(c)
                        # 获取最小外接矩形的4个顶点
                        box = cv2.cv.BoxPoints(rect)
                        # 数据类型转换
                        # 绘制轮廓
                        cv2.drawContours(frame, [np.int0(box)], -1, (0, 255, 255), 2)
                        # 找色块中心点
                        c_x, c_y = rect[0]
                        h, w = rect[1]
                        c_angle = rect[2]
                        if h * w >= 1350:   # 色块面积限制
                            # 绘制中心点
                            cv2.circle(frame, (int(c_x), int(c_y)), 3, (216, 0, 255), -1)
                            # print 'nnn', int(c_x), int(c_y)
                            # print c_angle
                            # 存储用于判读X是否稳定的列表
                            last_blocks.append([int(c_x), i])
                            if stable:
                                # 存储 稳定后的数据
                                storage_blocks.append((int(c_y), int(c_x), i, int(c_angle)))
                '''
                    检测色块位置步骤：
                        1、 判读色块 X 轴数据是否稳定
                        2、 稳定后，存储数据
                        3、 判读稳定后的数据， 找出Y轴色块 距离机械臂最近的
                '''
                stable = False
                if len(last_blocks) > 0:
                    if -10 <= int(last_blocks[len(last_blocks) - 1][0] - last_x) <= 10:    # 只判读最后一个方块是否稳定
                        print cv_count
                        cv_count += 1
                    else:
                        cv_count = 0
                    last_x = int(last_blocks[len(last_blocks) - 1][0])
                    last_blocks = []
                    if cv_count >= 5:
                        cv_count = 0
                        stable = True   # 数据稳定后，开始取数据
                    # 稳定后的数据发送给搬运进程
                    if len(storage_blocks) > 0:
                        max_y = storage_blocks.index(max(storage_blocks))
                        # 存储稳定后的数据， 颜色， X, Y, 色块角度
                        position_color_list.append((storage_blocks[max_y][2], storage_blocks[max_y][1],
                                                    storage_blocks[max_y][0], storage_blocks[max_y][3]))
                        storage_blocks = []
                        cv_blocks_ok = True  # 开始搬运
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
            frame = lens_distortion_adjustment(minFrame)
            img_h, img_w = frame.shape[:2]
            # 获取图像中心点坐标x, y
            img_center_x = img_w / 2
            img_center_y = img_h / 2
            cv2.line(frame, ((img_w / 2) - 20, (img_h / 2)), ((img_w / 2) + 20, (img_h / 2)), (0, 0, 255), 1)
            cv2.line(frame, ((img_w / 2), (img_h / 2) - 20), ((img_w / 2), (img_h / 2) + 20), (0, 0, 255), 1)
            cv2.imshow('image', frame)
            cv2.waitKey(1)
            time.sleep(0.01)


