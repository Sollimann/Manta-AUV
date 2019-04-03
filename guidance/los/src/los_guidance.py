#!/usr/bin/env python
import rospy
from vortex_msgs.msg import PropulsionCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, PoseStamped, Pose
import numpy as np
import math
import actionlib
from waypoint_action_msgs.msg import WaypointAction
import time
from std_msgs.msg import Float64

class LOS:
    def __init__(self):
        self.waypoint_one = PoseStamped()
        self.waypoint_two = PoseStamped()
        #Position
        self.x = 0.0
        self.y = 0.0

        #Previous waypoint
        self.x_k = -5
        self.y_k = -5 

        #Next waypoint
        self.x_kp1 = 3.0
        self.y_kp1 = 3.0

        #LOS target
        self.x_los = 0
        self.y_los = 0

        self.R = 200
        
        #Heading
        self.heading = Float64()
        self.heading_d = Float64()


    def updatePosition(self, x, y, heading):
        #Position

        self.x = x
        self.y = y
        self.heading = heading

    def setWayPoints(self, x_k, y_k, x_kp1, y_kp1):
        #Previous waypoint
        self.x_k = x_k
        self.y_k = y_k

        #Next waypoint
        self.x_kp1 = x_kp1
        self.y_kp1 = y_kp1


    def LOSG(self): #current values = x,xk,xkp1,y,yk,ykp1
        self.y_delta = self.y_kp1 - self.y_k
        self.x_delta = self.x_kp1 - self.x_k

        if self.x_delta != 0:
            self.d = self.y_delta/self.x_delta
            self.e = self.x_k
            self.f = self.y_k
            self.g = self.f -self.d*self.e

            self.b = 2*(self.d*self.g-self.d*self.y-self.x)
            self.a = 1+self.d**2
            self.c = self.x**2+self.y**2+self.g**2-2*self.g*self.y-self.R**2

            if self.x_delta > 0:
                self.x_los = (-self.b + math.sqrt(self.b**2 -4*self.a*self.c))/(2*self.a)
            elif self.x_delta < 0:
                self.x_los = (-self.b - math.sqrt(self.b**2 -4*self.a*self.c))/(2*self.a)

            self.y_los = self.d*(self.x_los-self.x_k)+self.y_k

        elif self.x_delta == 0:
            self.x_los = self.x_k
            if self.y_delta > 0:
                self.y_los = self.y+math.sqrt(self.R**2-(self.x_los-self.x)**2)
            elif self.y_delta < 0:
                self.y_los = self.y-math.sqrt(self.R**2-(self.x_los-self.x)**2)
            else:
                self.y_los = self.y_k

        self.heading_d = math.atan2(self.y_los-self.y, self.x_los-self.x)

        return self.heading_d

class LosGuidanceNode(object):
    def __init__(self):
        rospy.init_node('los_guidance_node')
        self.sub = rospy.Subscriber('/odometry/filtered', Odometry, self.callback, queue_size=1)
        self.pub_ref = rospy.Publisher('yaw_reference', Float64, queue_size=1)

        self.los = LOS()
        self.action_server = actionlib.SimpleActionServer('waypoint_coordination', WaypointAction, self.waypoint_action_server, False)
        self.action_server.start()

    def waypoint_action_server(self, goal):
        if goal.is_first_wp:
            odom_msg = rospy.wait_for_message('/odometry/filtered', Odometry)
            self.los.x_k = odom_msg.pose.pose.position.x
            self.los.y_k = odom_msg.pose.pose.position.y

        else:
            self.los.x_k = self.los.x_kp1
            self.los.y_k = self.los.y_kp1

        self.los.x_kp1 = goal.pose.pose.position.x
        self.los.y_kp1 = goal.pose.pose.position.y
        
        #time.sleep(3)

        self.completed_path_segment()
        
        self.action_server.set_succeeded()


    def completed_path_segment(self):
        #Circle of acceptance
        #Making sure state estimate is updated
        rospy.wait_for_message('/odometry/filtered', Odometry)
        self.los.heading_d = self.los.LOSG()
        head_error = self.los.heading - self.los.heading_d

        while head_error > 0.01:
            head_error = self.los.heading - self.los.heading_d
            #print head_error

    def callback(self, msg):
        self.heading = msg.pose.pose.orientation.z
        self.los.updatePosition(msg.pose.pose.position.x, msg.pose.pose.position.y, self.heading)

        self.heading_d = self.los.LOSG()
        print(self.heading_d)
        
        """
        self.error = self.heading-self.heading_d
        self.norm_error = self.error/(abs(self.error)+1)

        motion_msg = Wrench()
        motion_msg.torque.z = self.norm_error
        print(motion_msg)
        """
        heading_ref_msg = Float64()
        heading_ref_msg.data = self.heading_d
        print "HER"
        self.pub_ref.publish(heading_ref_msg)
        #motion_msg.force
  


if __name__ == '__main__':
    try:
        los_guidance_node = LosGuidanceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
