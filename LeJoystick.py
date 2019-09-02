#!/usr/bin/python3

import pygame
import time
import os
from socket import *

HOST = "127.0.0.1"
PORT = 8947

step_width = 40
key_map = {"PSB_CROSS":2, "PSB_CIRCLE":1, "PSB_SQUARE":3, "PSB_TRIANGLE":0,
        "PSB_L1": 4, "PSB_R1":5, "PSB_L2":6, "PSB_R2":7,
        "PSB_SELECT":8, "PSB_START":9, "PSB_L3":10, "PSB_R3":11};
action_map = ["CROSS", "CIRCLE", "", "SQUARE", "TRIANGLE", "L1", "R1", "L2", "R2", "SELECT", "START", "", "L3", "R3"]

pygame.init()
os.environ["SDL_VIDEODRIVER"] = "dummy"
pygame.display.init()
pygame.joystick.init()
if pygame.joystick.get_count() > 0:
    js=pygame.joystick.Joystick(0)
    js.init()
    jsName = js.get_name()
    print("Name of the joystick:", jsName)
    jsAxes=js.get_numaxes()
    print("Number of axis:",jsAxes)
    jsButtons=js.get_numbuttons()
    print("Number of buttons:", jsButtons);
    jsBall=js.get_numballs()
    print("Numbe of balls:", jsBall)
    jsHat= js.get_numhats()
    print("Number of hats:", jsHat)

pygame.joystick.init()
connected = False
count = 0
while True:
    if os.path.exists("/dev/input/js0") is True :
        if connected is False:
            pygame.joystick.quit()
            pygame.joystick.init()
            jscount =  pygame.joystick.get_count()
            if jscount > 0:
                try:
                    js=pygame.joystick.Joystick(0)
                    js.init()
                    js.get_hat(0)
                    client = socket(AF_INET, SOCK_STREAM)
                    client.connect((HOST,PORT))
                    connected = True
                except Exception as e:
                    print(e)
            else:
                pygame.joystick.quit()
        else:
            pass
    else:
        if connected is True:
            connected = False
            js.quit();
            pygame.joystick.quit()
            client.close()
        else:
            pass
    if connected is True:
        pygame.event.pump()
        change = [0,0,0,0,0,0]
        try:
            if js.get_button(key_map["PSB_R1"]) :
                change[0] -= step_width
            if js.get_button(key_map["PSB_R2"])  :
                change[0] += step_width
            if js.get_button(key_map["PSB_SQUARE"]) :
                change[1] -= step_width
            if js.get_button(key_map["PSB_CIRCLE"]) :
                change[1] += step_width
            if js.get_button(key_map["PSB_TRIANGLE"]) :
                change[2] -= step_width
            if js.get_button(key_map["PSB_CROSS"]) :
                change[2] += step_width
            if js.get_button(key_map["PSB_L1"]) :
                change[3] -= step_width
            if js.get_button(key_map["PSB_L2"]) :
                change[3] += step_width
            hat = js.get_hat(0)
            if hat[0] > 0 :
                change[5] += step_width
            elif hat[0] < 0:
                change[5] -= step_width
            else:
                pass
            if hat[1] > 0 :
                change[4] += step_width
            elif hat[1] < 0:
                change[4] -= step_width
            else:
                pass
            lx = js.get_axis(0)
            ly = js.get_axis(1)
            rx = js.get_axis(2)
            ry = js.get_axis(3)
            if lx > 0.5 :
                change[5] += step_width
            elif lx < -0.5:
                change[5] -= step_width
            else:
                pass
            l3_state = js.get_button(key_map["PSB_L3"])
            if ly > 0.5 :
                if l3_state:
                    change[4] -= step_width
                else:
                    change[3] += step_width
            elif ly < -0.5:
                if l3_state:
                    change[4] += step_width
                else:
                    change[3] -= step_width
            else:
                pass
            if rx > 0.5 :
                change[1] += step_width
            elif rx < -0.5:
                change[1] -= step_width
            else:
                pass
            if ry > 0.5 :
                change[2] += step_width
            elif ry < -0.5:
                change[2] -= step_width
            else:
                pass
            cmd = ""
            cu = 0;
            for i in range(6):
                if not change[i] is 0:
                    cu += 1
                    change[i] += 10000
                    cmd += '-' + str(i+1) + '-' + str(change[i])
            if cu > 0 and not js.get_button(key_map["PSB_START"]):
                cmd = "I007-" + "50-" + str(cu) + cmd + "\r\n"
                print(cmd)
                client.send(cmd.encode())
            elif js.get_button(key_map["PSB_START"]):
                cmd = "I001-1000-6-1-1500-2-1500-3-1500-4-1500-5-1500-6-1500\r\n"
                client.send(cmd.encode())
            else:
                pass
        except Exception as e:
            print(e)
            connected = False
            client.close()
            
    time.sleep(0.06)
