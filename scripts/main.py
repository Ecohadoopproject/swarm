#! /home/michael/.local/share/virtualenvs/DeepLearningProject-WE2UAgWf/bin/python


import rospy
import message_filters
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import sys

average_pub = None
keybindings={"w":(0,1,0),"s":(0,-1,0),"a":(1,0,0),"d":(-1,0,0),"z":(0,0,1),"x":(0,0,-1)}

def evaluate(data1,data2,data3,data4,data5):
    global average_pose_x
    global average_pose_y
    global average_pose_z
    global average_heading_x
    global average_heading_y
    global average_heading_z

    poses_x = (data1.pose.position.x,data2.pose.position.x,data3.pose.position.x,data4.pose.position.x,data5.pose.position.x)
    poses_y = (data1.pose.position.y,data2.pose.position.y,data3.pose.position.y,data4.pose.position.y,data5.pose.position.y)
    poses_z = (data1.pose.position.z,data2.pose.position.z,data3.pose.position.z,data4.pose.position.z,data5.pose.position.z)
    heading_x = (data1.pose.orientation.x,data2.pose.orientation.x,data3.pose.orientation.x,data4.pose.orientation.x,data5.pose.orientation.x)
    heading_y = (data1.pose.orientation.y,data2.pose.orientation.y,data3.pose.orientation.y,data4.pose.orientation.y,data5.pose.orientation.y)
    heading_z = (data1.pose.orientation.z,data2.pose.orientation.z,data3.pose.orientation.z,data4.pose.orientation.z,data5.pose.orientation.z)

    average_pose_x = sum(poses_x)/5
    average_pose_y = sum(poses_y)/5
    average_pose_z = sum(poses_z)/5
    average_heading_x = sum(heading_x)/5
    average_heading_y = sum(heading_y)/5
    average_heading_z = sum(heading_z)/5
   
   
if __name__=="__main__":
    rospy.init_node("average_position",anonymous=True)
    average_pose_x = 0
    average_pose_y = 0
    average_pose_z = 0
    average_heading_x = 0
    average_heading_y = 0
    average_heading_z = 0

    drone1=message_filters.Subscriber("/uav1/ground_truth_to_tf/pose", PoseStamped)
    drone2=message_filters.Subscriber("/uav2/ground_truth_to_tf/pose", PoseStamped)
    drone3=message_filters.Subscriber("/uav3/ground_truth_to_tf/pose", PoseStamped)
    drone4=message_filters.Subscriber("/uav4/ground_truth_to_tf/pose", PoseStamped)
    drone5=message_filters.Subscriber("/uav5/ground_truth_to_tf/pose", PoseStamped)
    ts = message_filters.ApproximateTimeSynchronizer([drone1,drone2,drone3,drone4,drone5], queue_size=5, slop=0.1, allow_headerless=True)
    ts.registerCallback(evaluate)
   
    average_pub = rospy.Publisher("target",PoseStamped,queue_size=1)

    while True:
        pub_msg = PoseStamped()
        pub_msg.header =Header()
        try:
            value = input("Please enter w,a,s,d,z or x:\n")
        
            pub_msg.pose.position.x = average_pose_x + keybindings[value][0]
            pub_msg.pose.position.y = average_pose_y + keybindings[value][1]
            pub_msg.pose.position.z = average_pose_z + keybindings[value][2]
            pub_msg.pose.orientation.x = average_heading_x
            pub_msg.pose.orientation.y = average_heading_y
            pub_msg.pose.orientation.z = average_heading_z
            
            print(keybindings[value])
            print(pub_msg)
            average_pub.publish(pub_msg)
        except KeyError:
            pub_msg.pose.position.x = average_pose_x 
            pub_msg.pose.position.y = average_pose_y
            pub_msg.pose.position.z = average_pose_z
            pub_msg.pose.orientation.x = average_heading_x
            pub_msg.pose.orientation.y = average_heading_y
            pub_msg.pose.orientation.z = average_heading_z
            print(pub_msg)
            average_pub.publish(pub_msg)

    rospy.spin()

