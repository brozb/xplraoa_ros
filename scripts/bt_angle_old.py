#!/usr/bin/env python3

# +UUDF:6C1DEBA5F60C,-53,8,2,0,37,"6C1DEBA09862","",1437347
# instance ID | RSSI | azimuth | elevation | reserved | channel | anchor ID | user string | timestamp


import rospy

import serial
import numpy as np
from collections import deque
from sensor_tests.msg import Angles


Nan = "NAN"


def read_serial(ser):
    if ser.is_open:
        raw_data = ser.readline()
        data = raw_data.split(b",")
        return data
    else:
        return Nan


rospy.init_node("bt_angle")
pub1 = rospy.Publisher("angles_raw", Angles, queue_size=1)
pub2 = rospy.Publisher("angles_avg", Angles, queue_size=1)

q = deque()

ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.5)
r = rospy.Rate(100)
while ser.is_open and not rospy.is_shutdown():
    data = read_serial(ser)
    if len(data) > 1:
        az = float(data[2])
        el = float(data[3])
        rssi = float(data[1])
        az_r = -np.deg2rad(az)
        el_r = np.deg2rad(el)

        # add to queue
        if len(q) >= 15:
            q.popleft()
        q.append([az_r, el_r])

        # average
        sum_az = 0
        sum_el = 0
        for m in q:
            sum_az += m[0]
            sum_el += m[1]
        az_av = sum_az / len(q)
        el_av = sum_el / len(q)
        # print(
        # "az: %.1f (%.3f)  el: %.1f (%.3f)  RSSI: %.1f \r"
        # % (np.rad2deg(az_av), az_av, np.rad2deg(el_av), el_av, rssi),
        # end="",
        # )

        # publish angles
        a1 = Angles(az_r, el_r, rssi)
        pub1.publish(a1)
        if len(q) == 15:
            a2 = Angles(az_av, el_av, rssi)
            pub2.publish(a2)
    r.sleep()
