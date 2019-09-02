# coding=utf8
# 机械臂运动学：给定相应的坐标（X,Y,Z），机械臂运行到相应的位置
#
from math import *
import LeArm
import time
import matplotlib.pyplot as plt
import LeConf


d = []
for i in range(0, len(LeConf.Deviation), 1):
    if LeConf.Deviation[i] > 1800 or LeConf.Deviation < 1200:
        print("偏差值超出范围1200-1800")
    else:
        d.append(LeConf.Deviation[i] - 1500)

LeArm.initLeArm(tuple(d))
targetX = 0
targetY = 0
targetZ = 4000
lastX = 0
lastY = 0
lastZ = 4000

l0 = 960    # 底盘到第二个舵机中心轴的距离9.6cm
l1 = 1050   # 第二个舵机到第三 个舵机的距离10.5cm
l2 = 880    # 第三个舵机到第四 个舵机的距离8.8cm
l3 = 2150   # 第四个舵机到机械臂(张开后)最高点的距离16.5cm


alpha = 0   # 第四个舵机的仰角
# theta3 = 0  # ID3舵机角度值
# theta4 = 0  # ID4舵机角度值
# theta5 = 0  # ID5舵机角度值
# theta6 = 0  # ID6舵机角度值


def kinematic_analysis(x, y, z, Alpha):
    global alpha
    global l0, l1, l2, l3
    if x == 0:
        theta6 = 90.0
    elif x < 0:
        # a = y/x
        # print a
        theta6 = atan(y / x)
        # print theta6
        # print '123', theta6 * 180.0 / pi
        theta6 = 180 + (theta6 * 180.0/pi)
        # print '123', theta6
    else:
        theta6 = atan(y / x)
        theta6 = theta6 * 180.0 / pi
        # print '-123', theta6
    # print('theta6:', theta6)
    y = sqrt(x*x + y*y)     # x,y坐标的斜边
    # print('y1 + y2 + y3 = ', y)
    y = y-l3 * cos(Alpha*pi/180.0)    # 求出 y总 - y3 = y2 + y1
    # print('y1 + y2 = ', y)
    z = z-l0-l3*sin(Alpha*pi/180.0)   # 求出z1 + z2
    # print z
    if z < -l0:
        return False
    # print('z1 + z2', z)
    # print(-(y*y+z*z-l1*l1-l2*l2)/(2*l1*l2))
    if sqrt(y*y + z*z) > (l1 + l2):
        return False
    aaa = -(y*y+z*z-l1*l1-l2*l2)/(2*l1*l2)
    if aaa > 1 or aaa < -1:
        return False
    theta4 = acos(aaa)  # 算出弧度
    theta4 = 180.0 - theta4 * 180.0 / pi    # 转化角度
    # print('theta4:', theta4)
    if theta4 > 135.0 or theta4 < -135.0:
        return False
    alpha = acos(y / sqrt(y * y + z * z))
    bbb = (y*y+z*z+l1*l1-l2*l2)/(2*l1*sqrt(y*y+z*z))
    if bbb > 1 or bbb < -1:
        return False
    if z < 0:
        zf_flag = -1
    else:
        zf_flag = 1
    theta5 = alpha * zf_flag + acos(bbb)
    theta5 = theta5 * 180.0 / pi
    # print('theta5:', theta5)
    if theta5 > 180.0 or theta5 < 0:
        return False

    theta3 = Alpha - theta5 + theta4
    # print('theta3:', theta3)
    if theta3 > 90.0 or theta3 < -90.0:
        return False
    return theta3, theta4, theta5, theta6     # 运行成功返回数据


# 计算平均数
def averagenum(num):
    nsum = 0
    for i in range(len(num)):
        nsum += num[i]
    return nsum / len(num)


def ki_move(x, y, z, movetime):
    alpha_list = []
    for alp in range(-25, -85, -1):
        if kinematic_analysis(x, y, z, alp):
            alpha_list.append(alp)
    if len(alpha_list) > 0:
        # print 'y', y
        if y > 2150:
            best_alpha = min(alpha_list) #max
        else:
            best_alpha = min(alpha_list) #min
        # print best_alpha
        theta3, theta4, theta5, theta6 = kinematic_analysis(x, y, z, best_alpha)
        # print theta3, theta4, theta5, theta6
        pwm_6 = int(2000.0 * theta6 / 180.0 + 500.0)
        pwm_5 = int(2000.0 * (90.0 - theta5) / 180.0 + 1500.0)
        pwm_4 = int(2000.0 * (135.0 - theta4) / 270.0 + 500.0)
        pwm_3 = int(2000.0 * theta3 / 180.0 + 1500.0)
        # print 'pwm', pwm_3, pwm_4, pwm_5, pwm_6
        LeArm.setServo(3, pwm_3, movetime)
        LeArm.setServo(4, pwm_4, movetime)
        LeArm.setServo(5, pwm_5, movetime)
        LeArm.setServo(6, pwm_6, movetime)
        time.sleep(movetime / 1000.0)
        return True
    else:
        return False


def plt_image(x_list, y_list):
    plt.title('Scatter Plot')

    plt.xlabel('x-value')
    plt.ylabel('y-label')
    # plt.scatter(x, y, s, c, marker)
    # x: x轴坐标
    # y：y轴坐标
    # s：点的大小/粗细 标量或array_like 默认是 rcParams['lines.markersize'] ** 2
    # c: 点的颜色
    # marker: 标记的样式 默认是 'o'
    plt.legend()

    plt.scatter(x_list, y_list, s=10, c="#ff1212", marker='o')


if __name__ == '__main__':
     try:
         # 画出机械臂可以到达的点
         for y in range(1250, 3250, 50):
             for x in range(1500, -1500, -50):
                 ok = kinematic_analysis(x, y, 200, 0)
                 if ok:
                     #print(x, y)
                     plt_image(x, y)
                 else:
                     for a in range(0, -60, -1):
                         ok = kinematic_analysis(x, y, 200, a)
                         if ok:
                             #print('a', (x, y))
                             plt_image(x, y)
                             # xx_list.append(x)
                             # yy_list.append(y)
                             #print('a', a)
                             break
                 # time.sleep(0.1)
         plt.show()
         # plt_image(xx_list, yy_list)
     except Exception as e:
         print(e)
