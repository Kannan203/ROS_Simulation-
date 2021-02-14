#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,Point,Pose
from tf.transformations import euler_from_quaternion
from goal_publisher.msg import PointArray
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import *
from kn_192148_mini.srv import *
import math

class To_Goal(object):

    def __init__(self):
        self.active_ = False

        self.goal_pos_ = Pose()
        self.position_ = Point()
        self.yaw_ = 0
        self.state_ = 0
        self.yaw_precision_ = math.pi/90
        self.dist_precision_ = 0.3


        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size = 1)
        self.point = rospy.wait_for_message('/goals',PointArray)
        self.srv = rospy.Service('/go_to_point_switch',ziel,self.go_to_point_switch)
        self.pos = rospy.Subscriber('/gazebo/model_states',ModelStates,self.call_pos)
        
        #rospy.sleep(2)
        #self.i = 0
        rate = rospy.Rate(15)
        #
        # self.goal_pos_.x = self.point.goals[self.i].x
        # self.goal_pos_.y = self.point.goals[self.i].y
        # self.goal_pos_.z = self.point.goals[self.i].z
        while not rospy.is_shutdown():
             if not self.active_:
                continue
             else:

                 if self.state_ == 0:
                     self.fix_yaw(self.goal_pos_.position.x,self.goal_pos_.position.y)
                 elif self.state_ == 1:
                     self.go_straight(self.goal_pos_.position.x,self.goal_pos_.position.y)
                 elif self.state_ == 2:
                     velocity = Twist()
                     velocity.linear.x = 0
                     velocity.angular.z = 0
                     self.pub.publish(velocity)
                     self.change_state(0)
                 else:
                     rospy.loginfo('Unknown_State')
             rate.sleep()

    def change_state(self,state):
        self.state_ = state
        #print ('State Changed to [%s]'% self.state_)

    def call_pos(self,msg):
        self.position_ = msg.pose[1].position

        rot = msg.pose[1].orientation
        (r,p,self.yaw_) = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])

    def normalised_yaw(self,angle):
        if (math.fabs(angle)>math.pi):
            angle = angle - (2*math.pi*angle)/(math.fabs(angle))
        return angle

    def fix_yaw(self,x,y):

        desired_yaw_ = math.atan2(y - self.position_.y,x - self.position_.x )
        err_yaw = self.normalised_yaw(desired_yaw_ - self.yaw_)

        velocity = Twist()
        if math.fabs(err_yaw)>self.yaw_precision_:

            velocity.angular.z = 0.4 if err_yaw > 0 else -0.4
        self.pub.publish(velocity)
        if math.fabs(err_yaw) <= self.yaw_precision_:
            self.change_state(1)

    def go_straight(self,x,y):

        desired_yaw_ = math.atan2(y - self.position_.y,x - self.position_.x )
        err_yaw = desired_yaw_ - self.yaw_
        err_dist =math.sqrt(pow(y - self.position_.y,2)+ pow(x - self.position_.x,2))
        print('err_dist',err_dist)


        if err_dist > self.dist_precision_:
            velocity = Twist()
            velocity.linear.x = 0.3 #### 0.7 ###
            self.pub.publish(velocity)
        else:
            self.change_state(2)
        if math.fabs(err_yaw) > self.yaw_precision_:
            self.change_state(0)

    def go_to_point_switch(self,msg):
        self.active_ = msg.data
        self.goal_pos_ = msg.target
        res = zielResponse()
        res.success = True
        return res


if __name__ == '__main__':
    rospy.init_node('To_Goal_')
    To_Goal()
