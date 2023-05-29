#!/usr/bin/python3
#coding=utf-8

import rospy
import rospkg
import os
import json
import numpy as np
import math
import time
import threading
from geometry_msgs.msg import TwistStamped, Quaternion, PoseStamped
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import SetMavFrame
from mavros_msgs.msg import State, RCIn, HomePosition, PositionTarget
from mavros_msgs.msg import Thrust
from visualization_msgs.msg import MarkerArray, Marker
from utils_obs import Utils
from assemble_cmd import Px4Controller


# Simulation of RealFlight
current_state = State()
ch5, ch6, ch7, ch8, ch9, ch11, ch14 = 0, 0, 0, 0, 1, 1, 1
is_initialize_mav, is_initialize_vel, is_initialize_rc, is_initialize_img = False, False, False, False

ch20 = 0
mav_pos = [0, 0, 0]
mav_original_angle = [0, 0, 0]
# mav_vel = [0, 0, 0]
mav_vel = np.array([0, 0, 0])
mav_yaw = 0
mav_R = np.zeros((3,3))
Initial_pos = [0, 0, 0]
pos_i = [0, 0, 0, 0, 0]
pos_i_raw = [0, 0, 0, 0, 0]
pos_i_ekf = [0, 0, 0, 0, 0]
image_failed_cnt = 0
state_name = "InitializeState"
idle_command = TwistStamped()
command = PositionTarget()
command.coordinate_frame = PositionTarget.FRAME_LOCAL_NED 
command.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                  + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                  + PositionTarget.IGNORE_YAW

desired_pos = np.array([0., 0., 0.])
desired_vel_norm = np.array([0., 0., 0.])


def spin():
    rospy.spin()

def state_cb(msg):
    global current_state
    current_state = msg

def mav_pose_cb(msg):
    global mav_pos, mav_yaw, mav_R, is_initialize_mav, mav_pitch, mav_roll
    is_initialize_mav = True
    mav_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
    mav_yaw = math.atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
    mav_pitch = math.asin(2*(q0*q2 - q1*q3))
    mav_roll = math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1 + q2*q2))
    R_ae = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
                      [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
                      [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])
    # R_ba = np.array([[0,1,0], [-1,0,0], [0,0,1]]) #mavros_coordinate to body_coordinate
    R_ba = np.array([[0,1,0], [-1,0,0], [0,0,1]]) #mavros_coordinate to baselink_coordinate  // body to enu  # body: right-front-up3rpos_est_body)
    mav_R = R_ae.dot(R_ba)
    # mav_R = R_ae

def mav_vel_cb(msg):
    global mav_vel, is_initialize_vel
    is_initialize_vel = True
    # mav_vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
    mav_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

def flight_corridor_cb(msg):
    if len(msg.markers) == 0: 
        print("return 0")
        return
    global desired_pos, desired_vel_norm
    p0 = msg.markers[0].pose.position
    p1 = msg.markers[1].pose.position
    if p0.x == 0 and p1.x == 0:
        print("return 1")
        return
    desired_pos  = np.array([p0.x, p0.y, p0.z])
    desired_vel_norm  = np.array([p1.x, p1.y, p1.z]) - np.array([p0.x, p0.y, p0.z])
    desired_vel_norm /= np.linalg.norm(desired_vel_norm)
    print("desired_pos: {}\ndesired_vel_norm: {}".format(desired_pos, desired_vel_norm))

def rcin_cb(msg):
    global ch5, ch6, ch7, ch8, ch9, ch11, ch14, is_initialize_rc
    is_initialize_rc = True
    last_ch5, last_ch6, last_ch7, last_ch8, last_ch9, last_ch11, last_ch14 = ch5, ch6, ch7, ch8, ch9, ch11, ch14
    chs = msg.channels
    ch5 = 2 if chs[4] < 1300 else 1 if chs[4] < 1700 else 0
    ch6 = 2 if chs[5] < 1300 else 1 if chs[5] < 1700 else 0
    ch7 = 0 if chs[6] < 1300 else 1 if chs[6] < 1700 else 2
    ch8 = 0 if chs[7] < 1300 else 1 if chs[7] < 1700 else 2
    ch9 = 2 if chs[8] < 1300 else 1 if chs[8] < 1700 else 0
    ch11 = 1 if chs[10] < 1500 else 0
    ch14 = 1 if chs[10] < 1500 else 0
    if ch5!=last_ch5 or ch6!=last_ch6 or ch7!=last_ch7 or ch8!=last_ch8 or ch9!=last_ch9 or ch11!=last_ch11 or ch14!=last_ch14:
        print("ch5: {}, ch6: {}, ch7: {}, ch8: {}, ch9: {}, ch11: {}, ch14: {}".format(ch5, ch6, ch7, ch8, ch9, ch11, ch14))

def call(event):
    global ch5, ch6, ch7, ch8, ch9, ch20
    k = event.keysym
    if k == "m":
        ch6 = 0
    elif k == "h":
        ch20 = 1
    elif k == "o":
        ch8 = 1
    elif k == "p":
        ch8 = 0
    elif k == "c":
        ch9 = (ch9 + 1) % 2
    elif k == "a":
        ch7 = 1
    elif k == "b":
        ch7 = 0
    time.sleep(0.02)

def read_kbd_input():
    import tkinter
    global is_initialize_rc
    is_initialize_rc = True
    win = tkinter.Tk()
    frame = tkinter.Frame(win,width=100,height=60)
    frame.bind("<Key>",call)
    frame.focus_set()
    frame.pack()
    win.mainloop()

def pos_image_cb(msg):
    global is_initialize_img, pos_i_raw, pos_i, image_failed_cnt
    is_initialize_img = True
    # print("msg_data: {}".format(msg.data))
    if msg.data[0] <= 0:
        image_failed_cnt += 1
    else:
        image_failed_cnt = 0
    if image_failed_cnt <= 20 and image_failed_cnt > 0:
        pass
    else:
        pos_i_raw = msg.data
        pos_i = pos_i_raw
    # print("pos_i_raw: {}".format(pos_i_raw))

def pos_image_ekf_cb(msg):
    global pos_i_ekf, pos_i_raw, pos_i
    pos_i_ekf = msg.data
    # If we don't consider the safety of the aircraft when the target is lost, use pos_i_ekf when pos_i_raw[0]<0.
    if abs(pos_i_ekf[0] - pos_i_raw[0]) < 10 and abs(pos_i_ekf[1] - pos_i_raw[1]) < 10:
        pos_i = pos_i_ekf
    else:
        pos_i = pos_i_raw
    # print("pos_i_ekf: {}".format(pos_i_ekf))
    # print("pos_i: {}".format(pos_i))


def minAngleDiff(a, b):
    diff = a - b
    if diff < 0:
        diff += 2*np.pi
    if diff < np.pi:
        return diff
    else:
        return diff - 2*np.pi

def angleLimiting(a):
    if a > np.pi:
        return a - 2*np.pi
    if a < -np.pi:
        return a + 2*np.pi
    return a



if __name__=="__main__":
    src_path = os.path.join(rospkg.RosPack().get_path("offboard_pkg"), "..")
    setting_file = open(os.path.join(src_path, "settings.json"))
    setting = json.load(setting_file)
    # print(json.dumps(setting, indent=4))

    MODE = setting["MODE"]
    car_velocity = setting["car_velocity"]
    # follow_mode: 0, ll=follow_distance; 1, ll=norm(car_home, mav_home)
    follow_mode = setting["follow_mode"]
    follow_distance = setting["follow_distance"]
    FLIGHT_H = setting["FLIGHT_H"]
    if MODE == "RealFlight":
        u = Utils(setting["Utils"])
    elif MODE == "Simulation":
        u = Utils(setting["Simulation"])

    rospy.init_node('offb_node', anonymous=True)
    spin_thread = threading.Thread(target = spin)
    spin_thread.start()

    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, mav_pose_cb)
    rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, mav_vel_cb)
    rospy.Subscriber("sim_planning_demo/flight_corridor", MarkerArray, flight_corridor_cb)

    #HIL使用遥控器进行控制
    IsRC = setting["IsRC"]
    if MODE == "RealFlight":
        rospy.Subscriber("mavros/rc/in", RCIn, rcin_cb)
        image_center = [setting["Utils"]["WIDTH"] / 2.0, setting["Utils"]["HEIGHT"] / 2.0]
    elif MODE == "Simulation":
        image_center = [setting["Simulation"]["WIDTH"] / 2.0, setting["Simulation"]["HEIGHT"] / 2.0]

        if IsRC == True:
            rospy.Subscriber("mavros/rc/in", RCIn, rcin_cb)
        # else:
        #     inputThread = threading.Thread(target=read_kbd_input)
        #     inputThread.start()
    else:
        raise Exception("Invalid MODE!", MODE)
    rospy.Subscriber("tracker/pos_image", Float32MultiArray, pos_image_cb)
    rospy.Subscriber("tracker/pos_image_ekf", Float32MultiArray, pos_image_ekf_cb)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    local_acc_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    # print("Publisher and Subscriber Created")

    rospy.wait_for_service("mavros/cmd/arming")
    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    rospy.wait_for_service("mavros/set_mode")
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    # print("Clients Created")
    rate = rospy.Rate(50)#50
    
    # ensure the connection 
    while(not current_state.connected):
        # print("connected: {}".format(current_state.connected))
        rate.sleep()

    px4 = Px4Controller()
    px4.start()

    last_request = rospy.Time.now()

    # start
    cnt = -1
    controller_reset = True
    while not rospy.is_shutdown():
        # print("time: {}".format(rospy.Time.now().to_sec() - last_request.to_sec()))
        cnt += 1

        pos_info = {"mav_pos": mav_pos, "mav_vel": mav_vel, "mav_R": mav_R, "R_bc": np.array([[0,0,1], [1,0,0], [0,1,0]]), 
                    "desired_pos": desired_pos, "desired_vel_norm": desired_vel_norm,
                    "mav_original_angle": mav_original_angle, "Initial_pos": Initial_pos}

        # cmd = u.RotateAttackAccelerationController(pos_info, pos_i, controller_reset)
        cmd = u.RotateAvoidancePDAccelerationController(pos_info, pos_i)
        controller_reset = False

        command.acceleration_or_force.x = cmd[0]
        command.acceleration_or_force.y = cmd[1]
        command.acceleration_or_force.z = cmd[2]
        command.yaw_rate = cmd[3]
        # print("cmd: {}".format(cmd))
        local_acc_pub.publish(command)

        rate.sleep()
    rospy.spin()