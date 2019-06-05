#!/usr/bin/env python

""" This node publishes the position of a ball that moves with a random walk.
    It also projects the ball into two different camera coordinate frames.
    It publishes synthetic images of the ball (one for each camera)
    and these will then be used to identify the ball in each image, and then 
    do stereo reconstruction. the idea is to test the stereo reconstruction.
"""

import roslib
import rospy
from numpy import *
roslib.load_manifest('triangulation_test')
import tf
from geometry_msgs.msg import *
from visualization_msgs.msg import *


class Ball():
    def __init__(self):
        self.w = 1
        self.x = 0
        self.y = 0
        self.z = 0
        self.xd = 0
        self.yd = 0
        self.zd = 0
        self.max = .25

    def update(self,dt):
        self.xd+=random.randn(1)*self.w*dt
        self.yd+=random.randn(1)*self.w*dt
        self.zd+=random.randn(1)*self.w*dt
        if((abs(self.x)>self.max)and(sign(self.xd)*sign(self.x)>0)):
            self.xd=0
        if((abs(self.y)>self.max)and(sign(self.yd)*sign(self.y)>0)):
            self.yd=0
        if((abs(self.z)>self.max)and(sign(self.zd)*sign(self.z)>0)):
            self.zd=0

        self.x+=self.xd*dt
        self.y+=self.yd*dt
        self.z+=self.zd*dt
        return self.x,self.y,self.z

class BallSim():
    def __init__(self):
        rospy.init_node('ballSim')

        #create the ball simulation
        self.B = Ball()

        #create the publishers that will provide information about the ball
        #self.posepub = rospy.Publisher('/ballsim/ballpose',PoseStamped,queue_size=1)
        self.markerpub= rospy.Publisher('/ballsim/ballmarker',Marker,queue_size=1)

        self.dt = 0.01
        #create the timed loop that will run the simulation
        rospy.Timer(rospy.Duration(self.dt),self.simcallback,oneshot=False)

    def simcallback(self,data):
        self.timenow = rospy.Time.now()
        #update the ball simulation
        x,y,z=self.B.update(self.dt)

        #send the coordinate system transform to the transform stack
        br = tf.TransformBroadcaster()
        ballquat = tf.transformations.quaternion_from_euler(0,0,0)
        br.sendTransform((x,y,z),ballquat,self.timenow,'/ball','world')

        #prepare the marker message
        ballmarker = Marker()
        ballmarker.header.frame_id='/ball'
        ballmarker.header.stamp = self.timenow
        ballmarker.type = ballmarker.SPHERE
        ballmarker.action = ballmarker.MODIFY
        ballmarker.scale.x = .1
        ballmarker.scale.y = .1
        ballmarker.scale.z = .1
        ballmarker.pose.orientation.w=1
        ballmarker.pose.orientation.x = 0
        ballmarker.pose.orientation.y=0
        ballmarker.pose.orientation.z=0
        ballmarker.pose.position.x = 0
        ballmarker.pose.position.y=0
        ballmarker.pose.position.z=0
        ballmarker.color.r=1.0
        ballmarker.color.g=0
        ballmarker.color.b=0
        ballmarker.color.a=1.0

        self.markerpub.publish(ballmarker)


def main(args):
    posepub = BallSim()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"

if __name__ == '__main__':
    main(sys.argv)