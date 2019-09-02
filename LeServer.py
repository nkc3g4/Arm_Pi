#!/usr/bin/python
# encoding: utf-8

import SocketServer
import threading
import time
import re
import LeArm #舵机转动
import LeConf #偏差
from LeCmd import LeError 
import LeCmd

class ServoServer(SocketServer.BaseRequestHandler):
    def handle(self):
        print("已连接")
        conn = self.request
        Flag = True
        recv_data = ""
        while Flag:
            try:
                recv_data += conn.recv(1024)
                if not recv_data:
                    Flag = False
                    print("break")
                    break
                recv_data = recv_data.replace(' ', '')
                
                cp = re.compile(r'\r\n')
                test = cp.search(recv_data)

                if test:
                    rdata = recv_data.split("\r\n")  #分割

                    recv_data = recv_data[len(rdata[0])+2:] #在缓存中去除将要执行的指令
                    rdata = [rdata[0]]
                    for data in rdata:  #因为rdata中只有一个元素，所以其实只会执行一次
                        if data:
                            #print("Cmd:%s"%(data))
                            rex = re.compile(r'^(I[0-9]{3}).*')  # 判断收到的指令是否符合规则
                            match = data
                            match = rex.match(match)
                            if match:
                                if not 0 == match.start() or not len(data) == match.end():
                                    print("错误指令 1")
                                else:
                                    data = data.split('-')
                                    cmd = data[0][1:5]
                                    del data[0]
                                    par = []
                                    try:
                                        cmd = int(cmd)
                                        if cmd >= 3 and cmd <= 7:
                                            #print(rdata)
                                            LeCmd.cmd_list[cmd](conn, data)
                                        else:
                                            for p in data:
                                                par.append(int(p))
                                            LeCmd.cmd_list[cmd](par)
                                    except LeError as err:
                                        print(err.msg)
                                        print(err.data)
                                    except:
                                        print("指令执行错误")
                            else:
                                print(data)
                                print("错误指令 2")
                else:
                    pass
                if not Flag:
                    print("break1")
                    break
            except Exception as e:
                print(e)
                break

    def finish(self):
        print("已断开")


class LeServer(SocketServer.ThreadingTCPServer):
    allow_reuse_address = True


if __name__ == "__main__":
    if not len(LeConf.Deviation) == 6:
        print("偏差数量错误")
        sys.exit()
    else:
        d = []
        for i in range(0,len(LeConf.Deviation), 1):
            if LeConf.Deviation[i] > 1600 or LeConf.Deviation < 1400:
                print("偏差值超出范围1200-1800")
                sys.exit()
            else:
                d.append(LeConf.Deviation[i] - 1500)

    LeArm.initLeArm(tuple(d))
    LeCmd.cmd_i001([1000, 6, 1, 1500, 2, 1500, 3, 1500, 4, 1500, 5, 1500, 6, 1500])
    server = LeServer(("", 8947), ServoServer)
    try:
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.start()
        while True:
            time.sleep(0.1)
    except:
        server.shutdown()
        server.server_close()
        LeArm.stopLeArm()
