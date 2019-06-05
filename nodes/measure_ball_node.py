#!/usr/bin/env python

#Alex Brown
#2018

import roslib
roslib.load_manifest('triangulation_test')
import sys
import rospy
#from cv2 import cv
from std_msgs.msg import *
from geometry_msgs.msg import *
#from preview_filter.msg import * #this is very important! we have custom message types defined in this package!!
from sensor_msgs.msg import Image,CameraInfo
from visualization_msgs.msg import Marker #we will use this message for the perceived fish. then pop it into Rviz
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy import *
import math
import cv2
import tf
import rospkg
from skimage import data, color, img_as_ubyte
from skimage.feature import canny
from skimage.transform import hough_ellipse
from skimage.draw import ellipse_perimeter

class MeasureBall:

    def __init__(self):
        #cv bridge for handling image transport
        self.bridge = CvBridge()
        
        #pub and sub for camera 1
        self.im1_sub = rospy.Subscriber("/cam1/image_raw",Image,self.im1callback,queue_size=1)#change this to proper name!
        self.CI1_sub = rospy.Subscriber("/cam1/camera_info",CameraInfo,self.CI1callback,queue_size=1)
        self.im1_pub = rospy.Publisher('/balltracker/overlay_im1',Image,queue_size=1)
        #placeholder for cam1 parameters
        self.CI1 = CameraInfo()
        #placeholder for the ball row and column
        self.balluv1 = array([0,0])

        #pub and sub for camera 2
        self.im2_sub = rospy.Subscriber("/cam2/image_raw",Image,self.im2callback,queue_size=1)#change this to proper name!
        self.CI2_sub = rospy.Subscriber("/cam2/camera_info",CameraInfo,self.CI2callback,queue_size=1)
        self.im2_pub = rospy.Publisher('/balltracker/overlay_im2',Image,queue_size=1)
        #placeholder for cam1 parameters
        self.CI2 = CameraInfo()
        #placeholder for the ball row and column
        self.balluv2 = array([0,0])

        #ball marker publisher
        self.markerpub = rospy.Publisher('/measured_ball',Marker,queue_size=1)

        #timed loop that will generate two synced images
        rospy.Timer(rospy.Duration(0.1),self.triangulate,oneshot=False)
    

    def cleanRects(self,rects):
        #gets rid of any rects that are fully contained within another
        rects = array(rects)
        rectsout=rects.copy()
        badrects = array([],dtype=int32)
        for rectnum in range(1,len(rects[:,0])):
            #what rect are we looking at?
            rect_tlx,rect_tly,rect_brx,rect_bry = rects[rectnum,0],rects[rectnum,1],rects[rectnum,0]+rects[rectnum,2],rects[rectnum,1]+rects[rectnum,3]
            #now see if any others are contained within it
            for testnum in range(1,len(rects[:,0])):
                testrect_tlx,testrect_tly,testrect_brx,testrect_bry = rects[testnum,0],rects[testnum,1],rects[testnum,0]+rects[testnum,2],rects[testnum,1]+rects[testnum,3]
                if ((rect_tlx-testrect_tlx)<=0 and (rect_tly-testrect_tly)<=0):
                    #this means that the TL corner is inside the rect
                    if ((rect_brx-testrect_brx)>=0 and (rect_bry-testrect_bry)>=0):
                        #this means that testrect is fully enclosed in rect, so delete it
                        badrects = append(badrects,testnum)
                        #print "found bad rect at index "+str(testnum)+" of "+str(len(rects[:,0]))
        rectsout=delete(rectsout,badrects,0)
        return rectsout

    def box(self,rects, img):
        #print rects.shape
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)
        #cv2.imwrite('one.jpg', img);

    def boxBW(self,rects, img):
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(img, (x1, y1), (x2, y2), ( 255,255,255), 2)
        #cv2.imwrite('one.jpg', img);

    def detect(self,img,thisname):
        frame = img.copy()
        rows,cols,depth = frame.shape

        rects = None
        if rows>0:
            #fgmask = self.fgbg.apply(frame)
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            canny = cv2.Canny(gray,100,200)
            # canny = cv2.dilate(canny,self.kernel,iterations=1)
            cannycolor = cv2.cvtColor(canny,cv2.COLOR_GRAY2BGR)
            im,contours,hier = cv2.findContours(canny.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            
            if(len(contours)>0):
                cv2.drawContours(cannycolor,contours,-1,(0,255,0),5)
                for k in range(0,len(contours)):
                    cnt = contours[k]
                    x,y,w,h = cv2.boundingRect(array(cnt))
                    #print x,y,w,h
                    if rects is not None:
                        rects = np.vstack((rects,np.array([x,y,w+x,h+y])))
                        #print rects
                    else:
                        rects = np.array([[x,y,w+x,h+y]])
            
            #print rects
            if rects is not None:
                rectsout = self.cleanRects(rects)
                #print rects.shape
                self.box(rectsout,frame)
            # cv2.imshow('frame',frame)
            #cv2.imshow(thisname,frame)

            #cv2.waitKey(1)

        if rects is not None:
            return frame,array([(rects[0,0]+rects[0,2])/2,(rects[0,1]+rects[0,3])/2])
        else:
            return frame,array([])



    def im1callback(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #if image topic is compressed, use following two lines instead
            # np_arr = fromstring(data.data,uint8)
            # frame = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
            if frame is not None:
                #print "detecting 1"
                frame1_out,self.balluv1 = self.detect(frame,'cam1')
                #print "im1: "+str(self.balluv1)
                img_out = self.bridge.cv2_to_imgmsg(frame1_out, "bgr8")
                img_out.header.stamp = rospy.Time.now()
                self.im1_pub.publish(img_out)

            else:
                print "frame is none"
        except CvBridgeError,e:
            print e

    def CI1callback(self,data):
        self.CI1 = data
        #print self.CI1.P

    def im2callback(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #if image topic is compressed, use following two lines instead
            # np_arr = fromstring(data.data,uint8)
            # frame = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
            if frame is not None:
                #print "detecting 1"
                frame2_out,self.balluv2 = self.detect(frame,'cam2')
                #print "im2: " +str(self.balluv2)
                img_out = self.bridge.cv2_to_imgmsg(frame2_out, "bgr8")
                img_out.header.stamp = rospy.Time.now()
                self.im2_pub.publish(img_out)

            else:
                print "frame is none"
        except CvBridgeError,e:
            print e

    def CI2callback(self,data):
        self.CI2 = data

    def triangulate(self,data):
        print "hello triangulate"
        P1 = array([self.CI1.P]).reshape(3,4)
        P2 = array([self.CI2.P]).reshape(3,4)
        x1 = vstack(self.balluv1)
        x1 = vstack((x1,1))
        x2 = vstack(self.balluv2)
        x2 = vstack((x2,1))
        #use the camera information and the current guess for the u,v
        #to triangulate the position of the ball.
        #print x1,x2
        #print P1,P2
        ballpos = cv2.triangulatePoints(P1, P2, self.balluv1.astype(float), self.balluv2.astype(float))
        ballpos/=ballpos[3]
        #print ballpos

        #send transforms to show ball's coordinate system
        br = tf.TransformBroadcaster()
        ballquat = tf.transformations.quaternion_from_euler(0,0,0)
        #I think the ball position needs to be inverted... why? Not sure but I think
        #it may be because there is a confusion in the T matrix between camera->object vs. object->camera
        br.sendTransform(-ballpos[:,0],ballquat,rospy.Time.now(),'/ballmeasured','world')

        #create a marker
        ballmarker = Marker()
        ballmarker.header.frame_id='/ballmeasured'
        ballmarker.header.stamp = rospy.Time.now()
        ballmarker.type = ballmarker.SPHERE
        ballmarker.action = ballmarker.MODIFY
        ballmarker.scale.x = .12
        ballmarker.scale.y = .12
        ballmarker.scale.z = .12
        ballmarker.pose.orientation.w=1
        ballmarker.pose.orientation.x = 0
        ballmarker.pose.orientation.y=0
        ballmarker.pose.orientation.z=0
        ballmarker.pose.position.x = 0
        ballmarker.pose.position.y=0
        ballmarker.pose.position.z=0
        ballmarker.color.r=0.0
        ballmarker.color.g=0
        ballmarker.color.b=1.0
        ballmarker.color.a=0.5

        self.markerpub.publish(ballmarker)


def main(args):
  
  rospy.init_node('measure_ball', anonymous=True)
  ic = MeasureBall()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)