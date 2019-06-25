#!/usr/bin/env python 
################################################################################# 
# Copyright 2018 ROBOTIS CO., LTD. 
# 
# Licensed under the Apache License, Version 2.0 (the "License"); 
# you may not use this file except in compliance with the License. 
# You may obtain a copy of the License at 
# 
#     http://www.apache.org/licenses/LICENSE-2.0 
# 
# Unless required by applicable law or agreed to in writing, software 
# distributed under the License is distributed on an "AS IS" BASIS, 
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
# See the License for the specific language governing permissions and 
# limitations under the License. 
################################################################################# 
# Authors: Gilbert
# Modified by team4


####For moving robot####
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
####FOR OPENCV########
import cv2 as cv
import numpy as np
import time
 
### for transformf####
import numpy as np
from numpy.linalg import inv
 
### for detecting trash using opencv ###
from collections import Counter

### for grabbing the trash ###
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import os

 
###############################################Detect the trash##################################################
#function to detect trash
# return 6 trashes poistions (white3+color3)
# should: check the camera number
# return form is '[(316, 60), (440, 54), (452, 40), (197, 45), (265, 43), (240, 39)]'
 
#cap = cv.VideoCapture(1)
 
def detect_trash():
    set2=[]
    set2x=[]
    set2y=[]
    set_mode2_list=[]
    result2=[]
    n=0
    N=0
    cap = cv.VideoCapture(1)#######!check camera number######
    cap.set(3, 1280)
    cap.set(4, 720)
 
    while(N<30):
        N=N+1
        ret, img_color = cap.read()
        img_color=img_color[70:650,200:1050]
        img_hsv = cv.cvtColor(img_color, cv.COLOR_BGR2HSV)
   
        lower_white = np.array([0,0,245], dtype=np.uint8)#0>250
        upper_white = np.array([255,255,255], dtype=np.uint8)

        lower_color2 = np.array([0,150,0], dtype=np.uint8)
        upper_color2 = np.array([90,255,240], dtype=np.uint8)
        lower_color1 = np.array([170,150,0], dtype=np.uint8)
        upper_color1 = np.array([180,255,255], dtype=np.uint8)

        lower_color3 = np.array([100,150,100], dtype=np.uint8)  #blue
        upper_color3 = np.array([110,255,255], dtype=np.uint8)
 
        img_mask1 = cv.inRange(img_hsv, lower_color1, upper_color1)
        img_mask2 = cv.inRange(img_hsv, lower_color2, upper_color2)
        img_mask3 = cv.inRange(img_hsv, lower_color3, upper_color3)

        img_maskw = cv.inRange(img_hsv, lower_white, upper_white)

        img_mask = img_mask1 | img_mask2 | img_maskw | img_mask3

        #kernel = np.ones((11,11),np.uint8)
        #img_mask = cv.morphologyEx(img_mask, cv.MORPH_OPEN, kernel)
        #img_mask = cv.morphologyEx(img_mask, cv.MORPH_CLOSE, kernel)
  
        img_result = cv.bitwise_and(img_color,img_color, mask= img_mask)
        num0fLabels, img_label, stats, centroids = cv.connectedComponentsWithStats(img_mask)
 
        for idx, centroid in enumerate(centroids):
            if stats[idx][0] == 0 and stats[idx][1] == 0:
                continue
            if np.any(np.isnan(centroid)):
                continue
            x,y,width,height,area = stats[idx]
            centerX,centerY = int(centroid[0]),int(centroid[1])
            #print(centerX,centerY)
            if 150>area>50: #nois detect #70,10
                if centerX<=725 or centerY>=95:
                    cv.circle(img_color,(centerX,centerY),10,(0,0,255),10)
                    cv.rectangle(img_color, (x,y), (x+width,y+height), (0,0,255))
                    cv.imshow('img_color',img_color)
                    cv.imshow('img_result',img_result)
                    #time.sleep(1)
                    #a2 = centroidsw[idxw]
                    set2.append([centerX,centerY]);

    for i, centroid in enumerate(set2):
        set2x.append(set2[i][0])
#        set2y.append(int(set2[iw][1]))
######
    cnt2=Counter(set2x)
    set_mode2=cnt2.most_common(6)

    for value in set_mode2:
        value=list(value)
        set_mode2_list.append(value)
    

    for i in range(len(set_mode2_list)):
        for j in range(len(set2)):
            if set_mode2_list[i][0]==int(set2[j][0]):
                result2.append([int(set2[j][0]),int(set2[j][1])])
    result=list(set(map(tuple,result2)))

    #cv.destroyAllWindows()
    print (result)
    return result
 
#################################################Move Robot####################################################
msg = """
 
control your Turtlebot3!
 
-----------------------
 
Insert xyz - coordinate.
 
x : position x (m)
 
y : position y (m)
 
z : orientation z (degree: -180 ~ 180)
 
If you want to close, insert 's'
 
-----------------------
 
"""
 
class GotoPoint():
 
    def __init__(self):
	 
	self.namespace = rospy.get_param("om_with_tb3")

        rospy.init_node('turtlebot3_pointop_key', anonymous=True)

        rospy.on_shutdown(self.shutdown)	
 
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
 
        self.position = Point()
 
        self.move_cmd = Twist()
 
        self.r = rospy.Rate(10)
 
        self.tf_listener = tf.TransformListener()
 
        self.odom_frame = 'odom'
 
 
 
        try:
 
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
 
            self.base_frame = 'base_footprint'

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
 
            try:
 
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
 
                self.base_frame = 'base_link'
 
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
 
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
 
                rospy.signal_shutdown("tf Exception")
 
 
 
        (self.position, self.rotation) = self.get_odom()
 
        global xposition
 
        global yposition
 
        global zposition
 
        global rotationval 
 
        xposition = self.position.x
 
        yposition = self.position.y
 
        zposition = self.position.z
 
        rotationval = self.rotation
 
        self.last_rotation = 0
 
        self.linear_speed = 1
 
        self.angular_speed = 1
 
 
    def goto(self,xval,yval,zval):
        self.goal_x, self.goal_y, self.goal_z = xval, yval, zval        
 
        if self.goal_z > 180 or self.goal_z < -180:
 
            print("you input worng z range.")
 
            self.shutdown()
 
        self.goal_z = np.deg2rad(self.goal_z)
 
        self.goal_distance = sqrt(pow(self.goal_x - self.position.x, 2) + pow(self.goal_y - self.position.y, 2))
 
        self.distance = self.goal_distance
 
 
        while self.distance > 0.15:
 
            (self.position, self.rotation) = self.get_odom()
 
            self.x_start = self.position.x
 
            self.y_start = self.position.y
 
            self.path_angle = atan2(self.goal_y - self.y_start, self.goal_x- self.x_start)
 
 
 
            if self.path_angle < -pi/4 or self.path_angle > pi/4:
 
                if self.goal_y < 0 and self.y_start < self.goal_y:
 
                    self.path_angle = -2*pi + self.path_angle
 
                elif self.goal_y >= 0 and self.y_start > self.goal_y:
 
                    self.path_angle = 2*pi + self.path_angle
 
            if self.last_rotation > pi-0.1 and self.rotation <= 0:
 
                self.rotation = 2*pi + self.rotation
 
            elif self.last_rotation < -pi+0.1 and self.rotation > 0:
 
                self.rotation = -2*pi + self.rotation
 
            self.move_cmd.angular.z = self.angular_speed * self.path_angle-self.rotation
 
 
 
            self.distance = sqrt(pow((self.goal_x - self.x_start), 2) + pow((self.goal_y - self.y_start), 2))
 
            self.move_cmd.linear.x = min(self.linear_speed * self.distance, 0.1)
 
 
 
            if self.move_cmd.angular.z > 0:
 
                self.move_cmd.angular.z = min(self.move_cmd.angular.z, 1.5)
 
            else:
 
                self.move_cmd.angular.z = max(self.move_cmd.angular.z, -1.5)
 
 
 
            self.last_rotation = self.rotation
 
            self.cmd_vel.publish(self.move_cmd)
 
            self.r.sleep()
 
        (self.position, self.rotation) = self.get_odom()
 
 
        
        while abs(self.rotation - self.goal_z) > 0.05:
 
            (self.position, self.rotation) = self.get_odom()
 
            if self.goal_z >= 0:
 
                if self.rotation <= self.goal_z and self.rotation >= self.goal_z - pi:
 
                    self.move_cmd.linear.x = 0.00
 
                    self.move_cmd.angular.z = 0.5
 
                else:
 
                    self.move_cmd.linear.x = 0.00
 
                    self.move_cmd.angular.z = -0.5
 
            else:
 
                if self.rotation <= self.goal_z + pi and self.rotation > self.goal_z:
 
                    self.move_cmd.linear.x = 0.00
 
                    self.move_cmd.angular.z = -0.5
 
                else:
 
                    self.move_cmd.linear.x = 0.00
 
                    self.move_cmd.angular.z = 0.5
 
            self.cmd_vel.publish(self.move_cmd)
 
            self.r.sleep()
 
        
 
 
 
        rospy.loginfo("Stopping the robot...")
 
        self.cmd_vel.publish(Twist())
 
    
 
    def get_odom(self):
 
        try:
 
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
 
            rotation = euler_from_quaternion(rot)
 
 
 
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
 
            rospy.loginfo("TF Exception")
 
            return
 
 
 
        return (Point(*trans), rotation[2])
 
 
 
 
 
    def shutdown(self):
 
        self.cmd_vel.publish(Twist())
 
        rospy.sleep(1)
 
####################################OPENCV ROBOT DETECT POSITION####################################
def robotcf():
    import cv2 as cv
    import numpy as np
    import time

    cap = cv.VideoCapture(1)
    cap.set(3, 1280)
    cap.set(4, 720)
    N=0
    while N<10:
        N=N+1

        ret, img_color = cap.read()
        img_color=img_color[70:650,200:1050]
        img_hsv = cv.cvtColor(img_color, cv.COLOR_BGR2HSV)
 
        # define range of white color in HSV
        lower_white = np.array([0,0,0], dtype=np.uint8)#black range in 17:00, 92,44,0
        upper_white = np.array([255,255,60], dtype=np.uint8)#105,70,60
 
    # Threshold the HSV image to get only white colors
        img_mask = cv.inRange(img_hsv, lower_white, upper_white)

        img_result = cv.bitwise_and(img_color,img_color, mask= img_mask)

  
 
        num0fLabels, img_label, stats, centroids = cv.connectedComponentsWithStats(img_mask)
 
        for idx, centroid in enumerate(centroids):
            if stats[idx][0] == 0 and stats[idx][1] == 0:
                continue #
            if np.any(np.isnan(centroid)):
                continue
            x,y,width,height,area = stats[idx] #
            centerX,centerY = int(centroid[0]),int(centroid[1]) #
            #time.sleep(3)
        #
      
 
        #print(centerX,centerY)
 
            if area>400: #nois detect #150>50
                if centerX<=725 or centerY>95:
                    cv.circle(img_color,(centerX,centerY),10,(0,0,255),10)
                    cv.rectangle(img_color, (x,y), (x+width,y+height), (0,0,255))
                    centerX=centerX
                    centerY=centerY
                #print(centroid) #array
 
                    cv.imshow('img_color',img_color)
                    cv.imshow('img_mask',img_mask)
                    cv.imshow('img_result',img_result)
                    #print(area)
                    #cv.destroyAllWindows()
                    break

 
    #if cv.waitKey(1) & 0xFF == 27:
     #   break
 

    print(centerX,centerY)
    return centerX, centerY

####################################transformf##################################################################
#input : 
#output : 
#function : 
 
 
 
def transformf(trashpixel, robotx1, roboty1, robotx2, roboty2, robotx3, roboty3):
    
    
    #p, q = input('pixelvalue:').split(',')
    #a, b = input('initrob:').split(',')
    #c, d = input('xmoverob:').split(',')
    #
    #
    #p,q = float(p), float(q)
    
    pixelvalue = np.asarray(trashpixel) #garbage pixel value
    initrob = np.asarray([robotx1,roboty1])  #initial robot pixel value
    xmoverob = np.asarray([robotx2,roboty2]) #robot pixel value when robot moves (1,0)
    ymoverob = np.asarray([robotx3,roboty3])    
    
    factor1 = pow(pow((robotx1-robotx2),2)+pow((roboty2-roboty1),2),0.5) # might be about 127~129
    factor2 = pow(pow((robotx3-robotx1),2)+pow((roboty3-roboty1),2),0.5)
    factor = (factor1+factor2) / 2
 
    robxbasis = (xmoverob-initrob)/factor
    robybasis = (ymoverob-initrob)/factor
    
    i = robxbasis[0]
    j = robxbasis[1]
    v = robybasis[0]
    w = robybasis[1]
    
    k_b = pixelvalue-initrob
    k = k_b / factor
    
    # k_x , k_y = x * i + v * i + y * j + y * w 
    # find x and y
    k = np.reshape(k,(2,1))
    k = np.asmatrix(k)
    determine= np.asmatrix([[i,j],[v,w]])
    print("fac:",factor)
    print("k:",k)
    print(type(determine))
    print(np.shape(k))
    print(np.shape(determine))

    #enterx = x[0][0]
    #print('goto trash:',x,y)
 
    trashx,trashy = inv(determine) * k
    print("trashx and trashy:",trashx,"&",trashy)
    return trashx, trashy
 
############################CHECK THE ROBOT COORDINATE SYSTEM###########################
 
def checkrobotcoor():
    #opencv robot detcting 1
    global robotx1
    global roboty1
    robotx1,roboty1 = robotcf()
    
    #moving 1m to x direction
    gotogoal = GotoPoint()
    gotogoal.goto(1,0,0)
    #opencv robot deecting 2
    global robotx2
    global roboty2
    robotx2, roboty2 = robotcf()
 
    gotogoal = GotoPoint()
    gotogoal.goto(0,1,0)
 
    global robotx3
    global roboty3
    robotx3, roboty3 = robotcf()
 
#############################Go to trash#######################################################
 
#gototrash
 
#input: trash, robot coordinate, trashpixel:list, so on: int value
#output: real trash coordinate
 
def gototrash(goalpositionx,goalpositiony):
    print("global coordinate:",goalpositionx,goalpositiony)
    gotogoal=GotoPoint()
    gotogoal.goto(goalpositionx,goalpositiony,0)

##############################################################################################
 
 
####################################Find the bin using pixel value#############################
 
#bin x and bin y should be pixel value
def findbin():
    global binpixel
    global binx
    global biny
    binpixel = [790,122] #this is pixel value
    binx ,biny = transformf(binpixel,robotx1,roboty1,robotx2,roboty2,robotx3,roboty3) #now we know where the bin is.
########################################################################################################
 
 
def gotobin():
    gotogoal=GotoPoint()
    gotogoal.goto(binx,biny,0)
 
#####################################################################################################################
 


###################################Grab the trash#############################################
class grapper():
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		#rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander("arm")
		#display_trajectory_publisher = rospy.Publisher('/move_group/					display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size = 10)
		rospy.sleep(5)	

	#####################################################################
	def grabtotrash(self):
    		waypoints = []
   
    		waypoints.append(self.group.get_current_pose().pose)
   
    		pose = geometry_msgs.msg.Pose()
    		pose.orientation.w = 0
    		pose.position.x = 0.6
    		pose.position.y = 0
    		pose.position.z = -0.1
    		waypoints.append(copy.deepcopy(pose))
   
   
    		(plan1,fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)        
                                  
    		rospy.sleep(5)
########################gripper#########################################

	def grab(self):
    		os.system('rosservice call /om_with_tb3/gripper \"planning_group: \'\' \n\
joint_position:\n\
  joint_name: \n\
  - \'\'\n\
  position:\n\
  - 0.03\n\
  max_accelerations_scaling_factor: 0.0\n\
  max_velocity_scaling_factor: 0.0\n\
path_time: 0.0\"')

##########################################################################
	def grabori(self):
		waypoints = []
   
    		waypoints.append(self.group.get_current_pose().pose)
   
    		pose = geometry_msgs.msg.Pose()
    		pose.orientation.w = 0
    		pose.position.x = 0.1
    		pose.position.y = 0
    		pose.position.z = 0.3
    		waypoints.append(copy.deepcopy(pose))
   
   
    		(plan2,fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)   
    		rospy.sleep(5)

#########################################################################33
	def throwaway(self):
    		waypoints = []
   
    		waypoints.append(self.group.get_current_pose().pose)
   
    		pose = geometry_msgs.msg.Pose()
    		pose.orientation.w = 0
    		pose.position.x = 1
    		pose.position.y = 0
    		pose.position.z = 0   
    		waypoints.append(copy.deepcopy(pose))
   
   
    		(plan3,fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0) 
    		rospy.sleep(5)
 
#################################################################################3
	def nograb(self):
    		os.system('rosservice call /om_with_tb3/gripper \"planning_group: \'\' \n\
joint_position:\n\
  joint_name: \n\
  - \'\'\n\
  position:\n\
  - 0.0\n\
  max_accelerations_scaling_factor: 0.0\n\
  max_velocity_scaling_factor: 0.0\n\
path_time: 0.0\"')
#########################################################################
	def movetoori(self):
    		waypoints = []
   
    		waypoints.append(self.group.get_current_pose().pose)
   
   		pose = geometry_msgs.msg.Pose()
    		pose.orientation.w = 0
    		pose.position.x = 0.05
    		pose.position.y = 0
    		pose.position.z = 0.05   
    		waypoints.append(copy.deepcopy(pose))
   
   
    		(plan4,fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0) 
    		rospy.sleep(5)
 


collision_object = moveit_msgs.msg.CollisionObject()




##############################################################################################

endeffector=grapper()

###########################             NOW        START        RUN!!!!         ####################################
 
while True:
    trashlist = detect_trash()   ## we get trash list [(1,1),(2,2), ...] -> 6 trash list
    print(type(trashlist))
    print(trashlist)
    checkrobotcoor()  ## we checked robot coordinate. It stored initial and x1 pixel value to global.
    findbin()
    for index in trashlist:
        #print('please:',trashlist[index])
        #print('type:',trashlist[index])
        #print('after:',trashlist[index])
        #trashpixel=trashlist[index]
        ### we should convert tuple to list
        trashpixel=index
        goalpositionx, goalpositiony = transformf(trashpixel,robotx1,roboty1,robotx2,roboty2,robotx3,roboty3)
        print('goalx:',goalpositionx)
        print('goaly:',goalpositiony)
        print('trashpixel:',trashpixel)
        gototrash(goalpositionx, goalpositiony)
	        
	endeffector.grabtotrash()
        endeffector.grab()
        endeffector.grabori()
        gotobin()
        endeffector.throwaway()
        endeffector.nograb()
        endeffector.grabori()
        time.sleep(3) 
