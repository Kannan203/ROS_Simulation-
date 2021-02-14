#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,Point,Pose
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from goal_publisher.msg import PointArray
from tf.transformations import euler_from_quaternion
from std_srvs.srv import *
from kn_192148_mini.srv import *
import math

class MasterProg(object):

    def __init__(self):
        rate = rospy.Rate(20)

        self.state_ = 0
        self.rewards = 0
        self.count_state_time_ = 0
        self.count_loop_ = 0
        self.position_ = Point()
        self.goal_pos_ = Point()
        self.init_position = Point()
        self.yaw_ = 0
        self.line_position_tol = 0.15
        self.region_front_Maxtol = 1
        self.region_front_Mintol = 0.15

        self.i = 8
        self.k = 0

        self.srv_client_go_to_point = None
        self.srv_client_wall_follower = None
        self.regions_ = None

        self.point = rospy.wait_for_message('/goals',PointArray)

        rospy.Subscriber('/scan',LaserScan,self.scanner)
        rospy.Subscriber('/gazebo/model_states',ModelStates,self.position)

        self.goal_pos_.x = self.point.goals[self.i].x
        print('goal_x',self.goal_pos_.x)
        self.goal_pos_.y = self.point.goals[self.i].y
        self.goal_pos_.z = self.point.goals[self.i].z

        self.init_position.x= 0
        self.init_position.y= 0
        self.init_position.z= 0

        rospy.wait_for_service('/go_to_point_switch')
        rospy.wait_for_service('/wall_follower_switch')

        self.srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch',ziel)
        self.srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch',SetBool)

        self.change_state(0)

        while not rospy.is_shutdown():

            self.position_from_line = self.distance_from_line(self.position_,self.goal_pos_.x,self.goal_pos_.y)

            if self.regions_ == None:
                continue
            if math.sqrt(pow(self.goal_pos_.y - self.position_.y, 2) + pow(self.goal_pos_.x - self.position_.x, 2)) <= 0.4:

                self.init_position.x= self.point.goals[self.i].x
                self.init_position.y= self.point.goals[self.i].y
                self.init_position.z= self.point.goals[self.i].z

                self.rewards = self.rewards + self.point.goals[self.i].reward
                print('Rewards Attained = {}'.format(self.rewards))

                self.i=self.i+1
                self.k=self.k+1
                print('No of Goal Points reached: {}'.format(self.i))
                
               
                if self.i == len(self.point.goals):
                    quit()

                self.goal_pos_.x = self.point.goals[self.i].x
                self.goal_pos_.y = self.point.goals[self.i].y
                self.goal_pos_.z = self.point.goals[self.i].z
                
                self.change_state(0)
            

            if self.state_ == 0:
                if self.regions_['front'] > 0.05 and self.regions_['front'] < 0.33: ### 0.15 & 0.5 ###
                    self.change_state(1)
            elif self.state_ == 1:
                if self.count_state_time_ > 5 and self.position_from_line < 1: #### 1 ###
                    self.change_state(0)
                self.count_loop_ += 1
                if self.count_loop_ == 20:
                    self.count_state_time_ += 1
                    self.count_loop_ = 0
                rate.sleep()
    def position(self,msg):
        self.position_ = msg.pose[1].position
        rot = msg.pose[1].orientation
        (r,p,self.yaw_) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
    def scanner(self,msg):
        self.regions_ = {
        'fright': min(min(msg.ranges[310:320]),10),
        'front' : min(min(msg.ranges[0:6]+msg.ranges[354:]),10),
        'fleft': min(min(msg.ranges[50:60]),10),
        }
    def distance_from_line(self,p0,x,y):#x and y are the goal position
        p1 = self.init_position #p1 is the initial position & p0 is the current position of bot
        equ_1 = math.fabs((y - p1.y) * p0.x - (x - p1.x) * p0.y + (x * p1.y) - (y * p1.x))
        equ_2 = math.sqrt(pow(y - p1.y, 2) + pow(x - p1.x, 2))
        distance = equ_1/equ_2
        return distance
    def change_state(self,state):
        self.state_ = state
        print('state1',self.state_)
        self.count_state_time_ = 0
        if self.state_ == 0:
            req = zielRequest()
            req.data = True
            req.target.position.x = self.goal_pos_.x
            req.target.position.y = self.goal_pos_.y

            res = self.srv_client_go_to_point_(req)
            res = self.srv_client_wall_follower_(False)
        if self.state_ == 1:
            req = zielRequest()
            req.data = False
            req.target.position.x = self.goal_pos_.x
            req.target.position.y = self.goal_pos_.y

            res = self.srv_client_go_to_point_(False,None)
            res = self.srv_client_wall_follower_(True)
if __name__ == '__main__':
    rospy.init_node('robot_initiation')
    MasterProg()
