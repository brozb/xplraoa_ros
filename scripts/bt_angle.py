#!/usr/bin/env python

# +UUDF:6C1DEBA5F60C,-53,8,2,0,37,"6C1DEBA09862","",1437347
# instance ID | RSSI | azimuth | elevation | reserved | channel | anchor ID | user string | timestamp


import rospy
import sys
import serial
import numpy as np
from collections import deque

from xplraoa1_ros.msg import Angles


class BT_angle:
    def __init__(self, usb_port, freq, n_avg):
        rospy.init_node("bt_angle")
        self.pub_raw = rospy.Publisher("angles_raw", Angles, queue_size=1)
        self.pub_avg = rospy.Publisher("angles_avg", Angles, queue_size=1)

        self.queue = deque()
        self.n_avg = n_avg

        self.ser = serial.Serial(usb_port, 115200, timeout=0.5)

        self.tim = rospy.Timer(rospy.Duration(1 / freq), self.read_data)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.tim.shutdown()
        rospy.sleep(0.2)
        rospy.loginfo("Shutting down")

    def read_serial(self):
        if self.ser.is_open:
            try:
                raw_data = self.ser.readline()
            except serial.SerialException as ex:
                rospy.logfatal("Serial exception [%s]" % ex)
                rospy.signal_shutdown("Connection failed")
            data = raw_data.split(b",")
            return data
        else:
            return None

    def read_data(self, _):
        data = self.read_serial()
        if data is None:
            rospy.logerror("Communication compromised")
        if len(data) > 1 and data[0][:4] == "+UUDF":
            az = float(data[2])
            el = float(data[3])
            rssi = float(data[1])
            az_r = -np.deg2rad(az)
            el_r = np.deg2rad(el)

            # add to queue
            if len(self.queue) > self.n_avg:
                self.queue.popleft()
            self.queue.append([az_r, el_r])

            # average
            sum_az = 0
            sum_el = 0
            for m in self.queue:
                sum_az += m[0]
                sum_el += m[1]
            az_av = sum_az / len(self.queue)
            el_av = sum_el / len(self.queue)

            # publish angles
            a1 = Angles(az_r, el_r, rssi)
            self.pub_raw.publish(a1)
            a2 = Angles(az_av, el_av, rssi)
            self.pub_avg.publish(a2)
        else:
            rospy.logerr("Unknown message [%s]" % (data))


if __name__ == "__main__":
    arg = rospy.myargv(argv=sys.argv)
    if len(arg) != 4:
        print("ERROR: wrong number of arguments")
        print(
            "expected three: usb_port, read_frequency, number of samples for averaging"
        )
        print("got:", arg[1:])
        quit()
    b = BT_angle(arg[1], float(arg[2]), arg[3])
    rospy.spin()
