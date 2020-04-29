#!/usr/bin/env python
import numpy as np
import rospy
import rospkg
import os.path
import sys
import tf
import datetime
import csv
from geometry_msgs.msg import PoseStamped


class csv_save():
    def __init__(self):
        self._f = []
        self._t = []
        ds = datetime.datetime.now()
        ds = str(ds)
        date = ds.split(" ")[0]
        time = ds.split(" ")[1].split(".")[0]
        homedir = os.path.expanduser("~")
        self.filename = homedir + "/2020-self-driving-car-project/result/" + \
            str(ds).split(".")[0].replace(":", "_").replace(" ", "_") + ".csv"
        self.result = rospy.Subscriber(
            "/result", PoseStamped, self.callback, queue_size=10)

    def callback(self, msg):
        quaternion = [0, 0, 0, 0]
        quaternion[0] = msg.pose.orientation.x
        quaternion[1] = msg.pose.orientation.y
        quaternion[2] = msg.pose.orientation.z
        quaternion[3] = msg.pose.orientation.w
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        self._t.append("%.9f" % (msg.header.stamp.to_nsec()*1e-9))
        self._t.append(str(msg.pose.position.x))
        self._t.append(str(msg.pose.position.y))
        self._t.append(str(msg.pose.position.z))
        self._t.append(str(yaw))
        self._t.append(str(pitch))
        self._t.append(str(roll))
        print self._t[0]

        self._f.append(self._t)
        self._t = []

    def onShutdown(self):
        with open(self.filename, mode='w') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerows(self._f)
            print(len(self._f))
        rospy.loginfo("csv saved.")


if __name__ == '__main__':
    rospy.init_node('generate_csv', anonymous=False)
    s = csv_save()
    rospy.on_shutdown(s.onShutdown)
    rospy.spin()
