#! /home/michael/.local/share/virtualenvs/DeepLearningProject-WE2UAgWf/bin/python


import rospy
import message_filters
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import sys


def magnitude(vector):
    absolute_difference = math.sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)
    return absolute_difference

def callback(data):    
    global target
    target = np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z,data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z])

def callback2(data):
    global drone
    drone = np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z,data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z])
    
def evaluate(data1,data2,data3,data4,data5,value):
    global x_accel
    global y_accel
    global z_accel
    poses=[data1,data2,data3,data4,data5]
    matrix_seperation =[]
    for main_element in poses:    
        matrix_difference = []
        for element in poses:
            matrix_difference.append([main_element.pose.position.x - element.pose.position.x, main_element.pose.position.y - element.pose.position.y, main_element.pose.position.z - element.pose.position.z])
        matrix_seperation.append(matrix_difference)
    for differences in matrix_seperation[int(value)-1]:
        Seperation_kp = 0.45
        if magnitude(differences) < 1.5:
            x_accel = Seperation_kp*differences[0]
            y_accel = Seperation_kp*differences[1]
            z_accel = Seperation_kp*differences[2]
       

if __name__=="__main__":
    rospy.init_node("seperation_vector",anonymous=True)
    value = sys.argv[1]
    x_accel = 0
    y_accel = 0
    z_accel = 0
    target = 0
    drone = 0
    message = "uav"+value+"/ground_truth_to_tf/pose"
    rospy.Subscriber("target", PoseStamped,callback)
    rospy.Subscriber(message, PoseStamped,callback2)
    drone1=message_filters.Subscriber("/uav1/ground_truth_to_tf/pose", PoseStamped)
    drone2=message_filters.Subscriber("/uav2/ground_truth_to_tf/pose", PoseStamped)
    drone3=message_filters.Subscriber("/uav3/ground_truth_to_tf/pose", PoseStamped)
    drone4=message_filters.Subscriber("/uav4/ground_truth_to_tf/pose", PoseStamped)
    drone5=message_filters.Subscriber("/uav5/ground_truth_to_tf/pose", PoseStamped)


    ts = message_filters.ApproximateTimeSynchronizer([drone1,drone2,drone3,drone4,drone5], queue_size=5, slop=0.1, allow_headerless=True)
    ts.registerCallback(evaluate,value)
    

    robot_namespace = "uav"+value+"/cmd_vel"
    vel = Twist()
    kp = 0.25
    pub = rospy.Publisher(robot_namespace,Twist,queue_size =10)

    while True:
        error = target -drone
        print(error)
        try:
            vel.linear.x = x_accel+ kp*error[0]
            vel.linear.y = y_accel+ kp*error[1]
            vel.linear.z = z_accel+kp*error[2]
            vel.angular.x = 2*kp*error[3]
            vel.angular.y = 2*kp*error[4]
            vel.angular.z = 2*kp*error[5]
                
            pub.publish(vel)
        except TypeError:
            pass


