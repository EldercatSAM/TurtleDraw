#!/usr/bin/python2.7

import roslib
import rospy
import math
import tf
import geometry_msgs.msg  
import turtlesim.srv  
import turtlesim.msg

Letter_dict = {
     'A':[(0,0.5,2),(1,0,0),(0,0.5,2),(1,1,0),(0,0.25,1),(1,0.75,1)],
     'B':[(0,0,2),(1,0,0),(0,0,2),(1,0.5,2),(1,0.5,1),(1,0,1),(1,1,1),(1,1,0),(0,0,0)],
     'C':[(0,1,2),(1,0,2),(1,0,0),(1,1,0)],
     'D':[(0,0,2),(1,0,0),(0,0,2),(1,0.5,2),(1,1,1.4),(1,1,0.8),(1,0.5,0),(1,0,0)],
     'E':[(0,0,2),(1,1,2),(0,0,2),(1,0,0),(1,1,0),(0,0,1),(1,0.75,1)],
     'F':[(0,0,2),(1,1,2),(0,0,2),(1,0,0),(0,0,1),(1,0.75,1)],
     'H':[(0,0,2),(1,0,0),(0,0,1),(1,1,1),(1,1,2),(1,1,0)],
     'I':[(0,0.25,2),(1,0.75,2),(0,0.5,2),(1,0.5,0),(1,0.25,0),(1,0.75,0)],
     'L':[(0,0,2),(1,0,0),(1,1,0)],
     'M':[(0,0.4,2),(1,0,0),(0,0.4,2),(1,0.8,0),(1,1.2,2),(1,1.6,0)],
     'O':[(0,0,2),(1,0,0),(1,1,0),(1,1,2),(1,0,2)],
     'S':[(0,1,2),(1,0,2),(1,0,1),(1,1,1),(1,1,0),(1,0,0)],
     'V':[(0,0,2),(1,0.5,0),(1,1,2)],
     'Y':[(0,0,2),(1,0.5,1.2),(0,1,2),(1,0.5,1.2),(1,0.5,0)],
}

def spawn(turtleid, x, y, theta): #create new turtle
     rospy.wait_for_service('spawn')
     spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
     spawner(x,y,theta,'turtle'+str(turtleid))

def penUp(turtleid, r=255, g=255, b=255, lineWidth=3):
     rospy.wait_for_service('turtle'+str(turtleid)+'/set_pen')
     penSetter = rospy.ServiceProxy('turtle'+str(turtleid)+'/set_pen', turtlesim.srv.SetPen)
     penSetter(r,g,b,lineWidth,True)

def penDown(turtleid, r=255, g=255, b=255, lineWidth=3):
     rospy.wait_for_service('turtle'+str(turtleid)+'/set_pen')
     penSetter = rospy.ServiceProxy('turtle'+str(turtleid)+'/set_pen', turtlesim.srv.SetPen)
     penSetter(r,g,b,lineWidth,False)

def teleport(turtleid, x, y, theta): # move turtle without leaving trace
     rospy.wait_for_service('turtle'+str(turtleid)+'/teleport_absolute')
     turtle_abs = rospy.ServiceProxy('turtle'+str(turtleid)+'/teleport_absolute', turtlesim.srv.TeleportAbsolute)
     turtle_abs(x,y,theta)

def move(turtleid, src, dst):
     turtle_vel =rospy.Publisher('turtle'+str(turtleid)+'/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
     y = dst[1] - src[1]
     x = dst[0] - src[0]
     dist = math.sqrt(x ** 2 + y ** 2)
     angle = math.atan2(y,x)
     cmd = geometry_msgs.msg.Twist()
     if abs(angle) < 0.05:
          angle = 0
     if abs(angle-math.pi) < 0.05:
          angle = math.pi
     #turn
     cmd.linear.x = 0
     cmd.angular.z = angle
     if angle == 0:
          cmd.angular.z = math.pi/2
     bias = 0.06
     while True:
          pose = getPose(turtleid)
          #rospy.loginfo(pose.theta)
          if (angle > 0 and pose.theta >= angle*(1-bias) and pose.theta <= angle*(1+bias)) or (angle < 0 and pose.theta <= angle*(1-bias) and pose.theta >= angle*(1+bias)) or (angle == 0 or angle == math.pi and pose.theta >= -bias and pose.theta <= bias):
               cmd.angular.z = 0
               turtle_vel.publish(cmd)
               teleport(turtleid,pose.x,pose.y,angle)
               break
          turtle_vel.publish(cmd)

     bias = 0.08
     #move
     cmd.linear.x = dist
     while True:
          pose = getPose(turtleid)
          if pose.x <= dst[0]*(1+bias) and pose.x>=dst[0]*(1-bias) and pose.y <= dst[1]*(1+bias) and pose.y>=dst[1]*(1-bias):
               teleport(turtleid,dst[0],dst[1],angle)
               break
          turtle_vel.publish(cmd)

     cmd.linear.x = 0
     cmd.angular.z = 0
     turtle_vel.publish(cmd)


def writeLetter(turtleid, letter, x, y, w, h):
     for i,(p,xx,yy) in enumerate(Letter_dict[letter]):
          if p:
               penDown(turtleid)
          else:
               penUp(turtleid)
          dst = (x+xx*w, y+yy*h/2)
          if i == 0:
               teleport(turtleid,dst[0],dst[1],0)
               continue
          pose = getPose(turtleid)
          src = (pose.x,pose.y)
          #rospy.loginfo(i)
          move(turtleid,src,dst)

def getPose(turtleid):
     pose = rospy.wait_for_message('turtle'+str(turtleid)+'/pose', turtlesim.msg.Pose, timeout=10)
     return pose

def getwidth(character):
     wid = []
     for (p,x,y) in character:
          wid.append(x)
     return max(wid)

def write(string):
     raws = 1
     
     for i in range(len(string)):
          if string[i] == ' ':
               raws += 1

     totalHeight = raws  + (raws + 1) * 0.4
     standardHeight = 11 / totalHeight
     standardGapY = standardHeight * 0.4
     currentY = standardGapY + (raws-1) * (standardHeight + standardGapY)

     vocabs = string.split()
     
     turtle = 1
     for vocab in vocabs:
          rospy.loginfo(vocab)
          totalWidth = 0
          for i in range(len(vocab)):
               totalWidth += getwidth(Letter_dict[vocab[i]])
               
          totalWidth = totalWidth + (len(vocab)+1)*0.5
          standardWidth = 11 / totalWidth 

          standardWidth = min(standardWidth,standardHeight/2)
          standardGap = standardWidth/2
          
          currentX = standardGap
          

          for i in range(len(vocab)):
               if turtle > 1:
                    spawn(turtle,currentX,currentY,0)
               writeLetter(turtle,vocab[i],currentX,currentY,standardWidth,2*standardWidth)
               currentX += getwidth(Letter_dict[vocab[i]]) * standardWidth + standardGap
               turtle += 1
          currentY -= (standardHeight + standardGapY)

#def draw(turtleid, x)
if __name__ == '__main__':
     rospy.init_node('turtle_tf_listener') 
     listener = tf.TransformListener()  
 
     #penUp(1)
     #teleport(1,1,1,0)
     #penDown(1)
     msg = raw_input("Please input what you want turtles to write:")
     

     write(msg.upper())
     rate = rospy.Rate(10.0)
