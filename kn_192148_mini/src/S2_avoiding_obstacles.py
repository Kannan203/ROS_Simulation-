#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,Point,Pose
from sensor_msgs.msg import LaserScan
from std_srvs.srv import *
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from math import sqrt,pow
import ipdb


class Wall_avoidance(object):

    def __init__(self):
        self.state_ = 0
        self.active_ = False

        rate = rospy.Rate(20)
        self.regions_ ={
        'fright': 0,
        'front' : 0,
        'fleft' : 0,
        }

        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.laser = rospy.Subscriber('/scan',LaserScan,self.scanner)
        self.srv = rospy.Service('wall_follower_switch',SetBool,self.wall_follower_switch)

        while not rospy.is_shutdown():
            if not self.active_:
                continue
            else:
                if self.state_ == 0:
                    self.Find_wall()
                elif self.state_ == 1:
                    self.Turn_Left()
                elif self.state_ == 2:
                    self.Follow_wall()
            rate.sleep()
    def scanner(self,msg):
        self.regions_ = {
        'fright': min(min(msg.ranges[310:320]),10),
        'front' : min(min(msg.ranges[0:6]+msg.ranges[354:]),10),
        'fleft': min(min(msg.ranges[50:60]),10),
        }
        self.take_action()
    def take_action(self):

        regions = self.regions_
        d = 0.33
        state_description = ''

        if regions['fleft'] > d and regions['front'] > d and regions['fright'] > d :
            state_description = 'Case 1-nothing'
            self.change_state(0)
        elif regions['fleft'] > d and regions['front'] > d and regions['fright'] < d :
            state_description = 'Case 2-fright'
            self.change_state(2)
        elif regions['fleft'] > d and regions['front'] < d and regions['fright'] > d :
            state_description = 'Case 3-front'
            self.change_state(1)
        elif regions['fleft'] > d and regions['front'] < d and regions['fright'] < d :
            state_description = 'Case 4-front and fright'
            self.change_state(1)
        elif regions['fleft'] < d and regions['front'] > d and regions['fright'] > d :
            state_description = 'Case 5-fleft'
            self.change_state(0)
        elif regions['fleft'] < d and regions['front'] > d and regions['fright'] < d :
            state_description = 'Case 6-fleft and fright'
            self.change_state(0)
        elif regions['fleft'] < d and regions['front'] < d and regions['fright'] > d :
            state_description = 'Case 7-fleft and front'
            self.change_state(1)
        elif regions['fleft'] < d and regions['front'] < d and regions['fright'] < d :
            state_description = 'Case 8-nothing'
            self.change_state(1)
        else:
            state_description = 'Unknown_State'

    def change_state(self,state):
        self.state_ = state

    def Find_wall(self):
        velocity = Twist()
        velocity.linear.x = 0.3
        velocity.angular.z = -0.3
        self.pub.publish(velocity)
    def Turn_Left(self):
        velocity = Twist()
        velocity.linear.x = 0
        velocity.angular.z = 0.4
        self.pub.publish(velocity)
    def Follow_wall(self):
        velocity = Twist()
        velocity.linear.x = 0.3
        self.pub.publish(velocity)
    def wall_follower_switch(self,msg):
        self.active_ = msg.data
        res = SetBoolResponse()
        res.success = True
        return res


if __name__ == '__main__':
    rospy.init_node('Avoid_Wall')
    Wall_avoidance()
