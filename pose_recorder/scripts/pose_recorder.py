#!/usr/bin/env python3
#! coding:utf-8

import matplotlib.animation as anm
import matplotlib.pyplot as plt
import math
import matplotlib.patches as patches
import matplotlib.lines as lines
import numpy as np
import itertools
from numpy.core.fromnumeric import repeat
import seaborn as sns
import random
import sys

import rospy
from nav_msgs.msg import Odometry
from rospy.client import init_node

class PoseRecorder:
    def __init__(self,is_visualization = False):
        rospy.init_node('pose_recorder')

        self.HZ = 1
        self.is_visualization = is_visualization
        self.odom_list = []
        self.x_list = []
        self.y_list = []

        self.odom_sub = rospy.Subscriber('/roomba/odometry',Odometry,self.odom_callback)
        self.init_graph()

    def init_graph(self):
        self.fig = plt.figure(figsize=(8,8))
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-20,20)
        self.ax.set_ylim(-20,20)
        self.ax.set_xlabel("X",fontsize=10)
        self.ax.set_ylabel("Y",fontsize=10)

    def draw(self):    
        plt.cla()
        self.init_graph()    
        self.ani = anm.FuncAnimation(self.fig,self.draw_trajectory,interval=0.1)
        plt.show()

    def draw_trajectory(self,i):
        self.ax.plot([x for x in self.x_list],[y for y in self.y_list],linewidth=1.5,color="red")



    def odom_callback(self,msg):
        #self.odom_list.append(msg)

        if len(self.x_list) == 1000: self.x_list.pop(0)
        if len(self.y_list) == 1000: self.y_list.pop(0)

        self.x_list.append(msg.pose.pose.position.x)
        self.y_list.append(msg.pose.pose.position.y)
        #rospy.loginfo("x: %f, y: %f", msg.pose.pose.position.x, msg.pose.pose.position.y)
        for x in self.x_list: rospy.loginfo("x: %f", x)

    def process(self):
        r = rospy.Rate(self.HZ)
        while not rospy.is_shutdown():
            self.draw()
            r.sleep()

if __name__ == '__main__':
    try:
        pose_recorder = PoseRecorder()
        pose_recorder.process()
    except rospy.ROSInterruptException: pass
