#!/usr/bin/python3
#coding=utf-8

import cv2
import numpy as np
import time
import VisionCaptureApi
import PX4MavCtrlV4 as PX4MavCtrl
import math

# 启用ROS发布模式
VisionCaptureApi.isEnableRosTrans =True
vis = VisionCaptureApi.VisionCaptureApi()

# VisionCaptureApi 中的配置函数
vis.jsonLoad()  # 加载Config.json中的传感器配置文件
vis.startImgCap()  # 开启取图循环，执行本语句之后，已经可以通过vis.Img[i]读取到图片了
print('Start Image Reciver')

vis.RemotSendIP = "192.168.1.152"

vis.sendImuReqCopterSim()
# vis.sendImuReqServe()


# 控制飞机起飞到一定高度
VehilceNum = 1
MavList = []
# Create MAV instance
for i in range(VehilceNum):
    MavList = MavList+[PX4MavCtrl.PX4MavCtrler(20100+i*2, '255.255.255.255')]

time.sleep(2)
# Start MAV loop with UDP mode: MAVLINK_FULL
for i in range(VehilceNum):
    MavList[i].InitMavLoop()
    MavList[i].InitTrueDataLoop()

# Enter Offboard mode to start vehicle control
time.sleep(2)
for i in range(VehilceNum):
    MavList[i].initOffboard()

# Get the takeoff position of each vehicle to the UE4 Map
# this can be adopted to obtain the global position of a vehicle in swarm simulation
time.sleep(2)
Error2UE4Map = []
for i in range(VehilceNum):
    mav = MavList[i]
    Error2UE4Map = Error2UE4Map+[-np.array([mav.uavGlobalPos[0]-mav.uavPosNED[0],
                                           mav.uavGlobalPos[1]-mav.uavPosNED[1], mav.uavGlobalPos[2]-mav.uavPosNED[2]])]

# fly to 10m high above its takeoff position
for i in range(VehilceNum):
    MavList[i].SendPosNED(-5, 5, -1, 0)
    MavList[i].SendCopterSpeed(3)
time.sleep(10)

