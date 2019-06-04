#!/usr/bin/env python

#Alex Brown
#2019

import roslib
roslib.load_manifest('fishtracker')
import sys
import rospy
#from cv2 import cv
from std_msgs.msg import *
from geometry_msgs.msg import *
#from preview_filter.msg import * #this is very important! we have custom message types defined in this package!!
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker #we will use this message for the perceived fish. then pop it into Rviz
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy import *
import math
import cv2
import tf
import rospkg


def generateCamMats(x,y,z,roll,pitch,yaw,fx,fy,cx,cy):
    x,y,z = 0.5,-2,0
    roll,pitch,yaw = 0,pi/2,pi/2
    K = array([[fx,0,cx,0],[0,fy,cy,0],[0,0,1,0],[0,0,0,1]])
    Rx = array([[1,0,0,0],[0,cos(roll),sin(roll),0],[0,-sin(roll),cos(roll),0],[0,0,0,1]])
    Ry = array([[cos(pitch),0,-sin(pitch),0],[0,1,0,0],[sin(pitch),0,cos(pitch),0],[0,0,0,1]])
    Rz = array([[cos(yaw),sin(yaw),0,0],[-sin(yaw),cos(yaw),0,0],[0,0,1,0],[0,0,0,1]])
    R = dot(Rx,dot(Ry,Rz))
    T = array([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]])
    return K,R,T


class SimCam:
    def __init__(self,K,R,T,w,h):
        self.K=K
        self.R=R
        self.T=T
        self.E = dot(self.R,self.T)
        self.w=w
        self.h=h
        self.P=dot(self.K,self.E)

    def renderSphere(self,position,radius,depth):
        im = zeros((self.h,self.w,3),dtype=uint8)

        #position will come in as a list or array
        pos = vstack(position)
        pos = vstack((pos,1))
        centroid = dot(self.K,pos)
        #real copout on the radius... but for now... TODO
        prad = self.K[0,0]*radius/(depth+.00001)
        #draw our sphere at the correct location
        # cv2.circle(im,(int(centroid[0]),int(centroid[1])), int(prad), (0,0,255), -1)
        cv2.circle(im,(int(centroid[0]),int(centroid[1])), 50, (0,0,255), -1)

        return im

class Cams:
    def __init__(self):
        #images 720 rows 1280 columns

        #first camera info
        x1,y1,z1 = -3,.5,0
        roll1,pitch1,yaw1 = -pi/2,0,-pi/2
        fx1,fy1,cx1,cy1 = 500,500,640,360
        self.K1,self.R1,self.T1= generateCamMats(x1,y1,z1,roll1,pitch1,yaw1,fx1,fy1,cx1,cy1)
        self.pos1 = [x1,y1,z1]
        self.quat1 = tf.transformations.quaternion_from_euler(roll1,pitch1,yaw1)
        self.simcam1 = SimCam(self.K1,self.R1,self.T1,1280,720)

        #second camera info
        x2,y2,z2 = 0,0.5,3
        roll2,pitch2,yaw2 = -pi,0,-pi/2
        fx2,fy2,cx2,cy2 = 500,500,640,360
        self.K2,self.R2,self.T2= generateCamMats(x2,y2,z2,roll2,pitch2,yaw2,fx2,fy2,cx2,cy2)
        self.pos2 = [x2,y2,z2]
        self.quat2 = tf.transformations.quaternion_from_euler(roll2,pitch2,yaw2)
        self.simcam2 = SimCam(self.K1,self.R1,self.T1,1280,720)

        #timed loop that will generate two synced images
        rospy.Timer(rospy.Duration(0.033),self.camcallback,oneshot=False)

        #transform listener
        self.listener = tf.TransformListener()
        self.bridge = CvBridge()

        #set up image publishers for synthetic images
        self.im1_pub = rospy.Publisher("/cam1/image_raw",Image,queue_size=1)
        self.im2_pub = rospy.Publisher("/cam2/image_raw",Image,queue_size=1)

    def camcallback(self,data):
        now = rospy.Time.now()

        #send transforms to show each camera's coordinate system
        br = tf.TransformBroadcaster()
        br.sendTransform(self.pos1,self.quat1,now,'/cam1','world')
        br.sendTransform(self.pos2,self.quat2,now,'/cam2','world')


        #look up the transform to the ball
        try:
            (ballpos,ballrot) = self.listener.lookupTransform('/ball', 'world', rospy.Time(0))
            (ballpos1,ballrot1) = self.listener.lookupTransform('/ball','cam1',rospy.Time(0))
            (ballpos2,ballrot2) = self.listener.lookupTransform('/ball','cam2',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("exception!")
            ballpos,ballrot = [0,0,0],[0,0,0,1]
            ballpos1,ballrot1 = [0,0,0],[0,0,0,1]
            ballpos2,ballrot2 = [0,0,0],[0,0,0,1]
            #continue

        
        #generate the synthetic images
        im1 = self.simcam1.renderSphere(ballpos,0.05,ballpos1[2])
        im2 = self.simcam2.renderSphere(ballpos,0.05,ballpos2[2])

        #now publish the synthetic images
        im1out = self.bridge.cv2_to_imgmsg(im1,"bgr8")
        im1out.header.stamp = now
        im2out = self.bridge.cv2_to_imgmsg(im2,"bgr8")
        im2out.header.stamp = now

        #publish the two synthetic images
        self.im1_pub.publish(im1out)
        self.im2_pub.publish(im2out)

def main(args):
  
  rospy.init_node('cam_simulator', anonymous=True)
  cameras = Cams()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  #cv2.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)