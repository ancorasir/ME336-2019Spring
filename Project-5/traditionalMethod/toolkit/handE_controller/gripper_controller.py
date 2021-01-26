# creat by lxb on 2019.05.06
import socket
import time
import os
#control robotiq hand-E
class HandEController:
    #get the current path
    __currentPath = os.path.dirname(os.path.abspath(__file__))
    __closePath = __currentPath + '/' + 'gripper_close.script'
    __openPath = __currentPath + '/' + 'gripper_open.script'
    __activePath = __currentPath + '/' + 'gripper_active.script'
    def __init__(self,robot_ip = "192.168.1.10",port = 30003):
        self.__robot_ip = robot_ip
        self.__port = port

    def closeGripper(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.__robot_ip, self.__port))
        with open(self.__closePath, "rb") as f:
            l = f.read(1024)
            while(l):
                s.send(l)
                l = f.read(1024)
        s.close()

    def openGripper(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.__robot_ip, self.__port))
        with open(self.__openPath, "rb") as f:
            l = f.read(1024)
            while(l):
                s.send(l)
                l = f.read(1024)
        s.close()

    def activeGripper(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.__robot_ip, self.__port))
        with open(self.__activePath, "rb") as f:
            l = f.read(1024)
            while(l):
                s.send(l)
                l = f.read(1024)
        s.close()

#   #test
# if __name__ == '__main__':
#     ip = "192.168.1.10"
#     port = 30003
#     gripper_controller = HandEController(ip,port)
#     gripper_controller.activeGripper()
#     time.sleep(2)
#     gripper_controller.closeGripper()
#     time.sleep(2)
#     gripper_controller.openGripper()
