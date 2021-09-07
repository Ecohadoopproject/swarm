#! /home/michael/.local/share/virtualenvs/DeepLearningProject-WE2UAgWf/bin/python

import rospy
import message_filters
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import sys

def callback(data):
    
    global target
    target = np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z])

def callback2(data):
    global drone
    drone = np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z])
    


if __name__ == "__main__":
    x = sys.argv[1]
    target = 0
    drone = 0
    message = "uav"+x+"/ground_truth_to_tf/pose"

    rospy.init_node("Error", anonymous=True)
    rospy.Subscriber("target", PoseStamped,callback)
    rospy.Subscriber(message, PoseStamped,callback2)
    robot_namespace = "uav"+x+"/cmd_vel"
    
    pub = rospy.Publisher(robot_namespace,Twist,queue_size=10)
    while True:

        error = target-drone
        print(error)
        vel = Twist()
        kp = 0.25
        try:
            vel.linear.x = kp*error[0]
            vel.linear.y = kp*error[1]
            vel.linear.z = kp*error[2]
            pub.publish(vel)
        except TypeError:
            pass



