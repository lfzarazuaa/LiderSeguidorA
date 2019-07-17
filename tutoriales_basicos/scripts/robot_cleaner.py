#!/usr/bin/env python
import sys
import rospy
import numpy as np

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class RobotCleaner:
    
    def __init__(self):
      self.pose_subscriber=rospy.Subscriber("/turtle1/pose", Pose,self.poseCallback,queue_size=1)
      self.velocity_publisher=rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
      self.turtlesim_pose=Pose()
      self.rate = rospy.Rate(10) # 10hz
      self.poseflag=False
      
    
    def poseCallback(self,data):
        #print('Recibido')
        self.turtlesim_pose.x=data.x
        self.turtlesim_pose.y=data.y
        self.turtlesim_pose.theta=data.theta
        self.poseflag=True
        #rospy.loginfo("new Pose data received: (%.2f, %.2f, %.2f)", 
        #self.turtlesim_pose.x,self.turtlesim_pose.y,self.turtlesim_pose.theta)

    def move(self,speed,distance,isForward):
       #distance=speed*time
        self.vel_msg=Twist()
        if isForward:
          self.vel_msg.linear.x=np.abs(speed)
        else:
          self.vel_msg.linear.x=-np.abs(speed)
        self.vel_msg.linear.y=0
        self.vel_msg.linear.z=0
        #set to zero angular velocities for a straigth line
        self.vel_msg.angular.x=0
        self.vel_msg.angular.y=0
        self.vel_msg.angular.z=0
        rospy.loginfo("speed distance isforward: (%.2f, %.2f, %d)", 
        speed,distance,isForward)
        #t0: current time
        #loop
        #publish velocity
        #estimate the distance = speed * (t1-t0)
        #current_distance_moved_by_robot <= distance
        self.rate = rospy.Rate(30)
        current_distance=0
        t0 = rospy.get_time()
        while True:
            self.velocity_publisher.publish(self.vel_msg)
            t1=rospy.get_time()
            current_distance = speed * (t1-t0)
            self.rate.sleep()
            if not(current_distance<distance):
                break
        self.vel_msg.linear.x=0
        self.velocity_publisher.publish(self.vel_msg)

    def rotate(self, angular_speed,angle,clockwise):
        self.vel_msg=Twist()
        #set a random linear velocity in the x-axis
        self.vel_msg.linear.x=0
        self.vel_msg.linear.y=0
        self.vel_msg.linear.z=0
        #set a random angular velocity in the z-axis
        self.vel_msg.angular.x=0
        self.vel_msg.angular.y=0
        if clockwise:
          self.vel_msg.angular.z=-np.abs(angular_speed)
        else:
          self.vel_msg.angular.z=np.abs(angular_speed)
        #t0: current time
        #loop
        #publish velocity
        #estimate the angle = angular_speed * (t1-t0)
        #current_angle_moved_by_robot <= angular_speed
        self.rate = rospy.Rate(30)
        current_angle=0
        t0 = rospy.get_time()
        while True:
            self.velocity_publisher.publish(self.vel_msg)
            t1=rospy.get_time()
            current_angle = angular_speed * (t1-t0)
            self.rate.sleep()
            if not(current_angle<angle):
                break
        self.vel_msg.angular.z=0
        self.velocity_publisher.publish(self.vel_msg)

    def rotateDeg(self, angular_speed,angle,clockwise):
        self.rotate(self.degrees2radians(angular_speed),self.degrees2radians(angle),clockwise)

    def degrees2radians(self,angle_in_degrees):
        return angle_in_degrees*np.pi/180.0

    def radians2degrees(self,angle_in_radians):
        return angle_in_radians*180.0/np.pi
    
    def setDesiredOrientation(self,desired_angle_degrees):
        desired_angle_radians=self.degrees2radians(desired_angle_degrees)
        while(not(self.poseflag)): #be secure that the first pose is recivied
            pass
        relative_angle_radians=desired_angle_radians-self.turtlesim_pose.theta
        if relative_angle_radians<0:
           clockwise=True
        else:
           clockwise=False
        self.rotate(np.abs(relative_angle_radians)/10,np.abs(relative_angle_radians),clockwise);
        print('rotation finished at: ', desired_angle_degrees)
    
    def setDesiredOrientation2(self,desired_angle_degrees):
        desired_angle_radians=self.degrees2radians(desired_angle_degrees)
        while(not(self.poseflag)): #be secure that the first pose is recivied
            pass
        
        for i in range(0,2,1):
         if self.turtlesim_pose.theta<0:
           theta=2*np.pi+self.turtlesim_pose.theta
         else:
           theta=self.turtlesim_pose.theta
         if theta>desired_angle_radians:
           relative_angle_radians1=theta-desired_angle_radians
           relative_angle_radians2=2*np.pi-theta+desired_angle_radians
           if relative_angle_radians2<relative_angle_radians1:
               relative_angle_radians=relative_angle_radians2
               clockwise=False
           else:
               relative_angle_radians=relative_angle_radians1
               clockwise=True
         else:
           relative_angle_radians1=desired_angle_radians-theta
           relative_angle_radians2=2*np.pi-desired_angle_radians+theta
           if relative_angle_radians1<relative_angle_radians2:
               relative_angle_radians=relative_angle_radians1
               clockwise=False
           else:
               relative_angle_radians=relative_angle_radians2
               clockwise=True
         self.rotate(np.abs(relative_angle_radians)/4,np.abs(relative_angle_radians),clockwise)
         if self.turtlesim_pose.theta<0:
           theta=2*np.pi+self.turtlesim_pose.theta
         else:
           theta=self.turtlesim_pose.theta
        print('rotation finished at: ', self.radians2degrees(theta))

    def getDistance(self,x1,y1,x2,y2):
	    return np.absolute(x2-x1+(y2-y1)*1j)
    
    def getDistance2(self,d1,d2):
	    #d1=Pose()
        #d2=Pose()
        #return np.abs(d2.x-d1.x+(d2.y-d1.y)*1j)
        return np.sqrt(np.power(d2.x-d1.x,2)+np.power(d2.y-d1.y,2))

    def getAngle(self,d1,d2):
	    #d1=Pose()
        #d2=Pose()
        return np.arctan2(d2.y-d1.y,d2.x-d1.x)

    def moveGoal(self,goal_pose,distance_tolerance):
        self.vel_msg=Twist()
        self.rate = rospy.Rate(15)
        #****** Proportional Controller ******#
		#    linear velocity in the x-axis
        while True:
              kl=1.5
              ka=8
              self.vel_msg.linear.x=kl*self.getDistance2(self.turtlesim_pose,goal_pose) 
              self.vel_msg.linear.y=0
              self.vel_msg.linear.z=0
              #set a ka proportional angular velocity in the z-axis
              self.vel_msg.angular.x=0
              self.vel_msg.angular.y=0
              self.vel_msg.angular.z =ka*(self.getAngle(self.turtlesim_pose,goal_pose)-self.turtlesim_pose.theta)
              self.velocity_publisher.publish(self.vel_msg)
              self.rate.sleep()
              if not(self.getDistance2(self.turtlesim_pose,goal_pose)>distance_tolerance):
                   break
        #make zero the linear and angular velocity
        self.vel_msg.linear.x=0
        self.vel_msg.angular.z=0
        self.velocity_publisher.publish(self.vel_msg)
        print 'x=',self.turtlesim_pose.x,'y=',self.turtlesim_pose.y
    
    def goalClean(self):
            goal_pose=Pose()
            goal_pose.x=2
            goal_pose.y=2
            goal_pose.theta=0
            self.moveGoal(goal_pose,0.01)
            goal_pose.x=3
            goal_pose.y=3
            goal_pose.theta=0
            self.moveGoal(goal_pose,0.01)
            goal_pose.x=4
            goal_pose.y=4
            goal_pose.theta=0
            self.moveGoal(goal_pose,0.01)
            goal_pose.x=5.544
            goal_pose.y=5.544
            goal_pose.theta=0
            self.moveGoal(goal_pose,0.01)
            goal_pose.x=10
            goal_pose.y=10
            goal_pose.theta=0
            self.moveGoal(goal_pose,0.01)
            goal_pose.x=9
            goal_pose.y=9
            goal_pose.theta=0
            self.moveGoal(goal_pose,0.01)
            goal_pose.x=7
            goal_pose.y=7
            goal_pose.theta=0
            self.moveGoal(goal_pose,0.01)

    def gridClean(self):
         rate = rospy.Rate(0.5)
         pose=Pose()
         pose.x=1
         pose.y=1
         pose.theta=0
         self.moveGoal(pose,0.01)
         rate.sleep()
         self.setDesiredOrientation2(0)
         rate.sleep()

         self.move(2,9,True)
         rate.sleep()
         self.rotateDeg(10,90,False)
         rate.sleep()
         self.move(2,9,True)

         self.rotateDeg(10,90,False)
         rate.sleep()
         self.move(2,1,True)
         self.rotateDeg(10,90,False)
         rate.sleep()
         self.move(2,9,True)

         self.rotateDeg(10,90,False)
         rate.sleep()
         self.move(2,1,True)
         self.rotateDeg(10,90,False)
         rate.sleep()
         self.move(2,9,True)

    def spiralClean(self):
        self.vel_msg=Twist()
        constant_speed=4
        rk=0.5
        self.rate = rospy.Rate(1)
        #****** Proportional Controller ******#
		#    linear velocity in the x-axis
        while True:
              rk=rk+0.25
              self.vel_msg.linear.x=rk 
              self.vel_msg.linear.y=0
              self.vel_msg.linear.z=0
              #set a ka proportional angular velocity in the z-axis
              self.vel_msg.angular.x=0
              self.vel_msg.angular.y=0
              self.vel_msg.angular.z=constant_speed
              self.velocity_publisher.publish(self.vel_msg)
              self.rate.sleep()
              if not((self.turtlesim_pose.x<10.5)and(self.turtlesim_pose.y<10.5)):
                   break
        #make zero the linear and angular velocity
        self.vel_msg.linear.x=0
        self.vel_msg.angular.z=0
        self.velocity_publisher.publish(self.vel_msg)

def main():
    if len(sys.argv) == 2 or len(sys.argv) == 3:
        rospy.init_node('RobotCleaner', anonymous=True)
        RC=RobotCleaner() # constructor creates publishers / subscribers
        option=int(sys.argv[1]) 
        if option==1:
           RC.move(1,4,True)
        elif option==2:
           RC.rotateDeg(5,90,True)
        elif option==3:
           RC.setDesiredOrientation2(float(int(sys.argv[2])))
        elif option==4:
            cont=True
            while(cont):
              x=float(input('Enter your x goal:'))
              y=float(input('Enter your y goal:'))
              if (x>0 and x<10.5)or(y>0 and y<10.5):
                  goal_pose=Pose()
                  goal_pose.x=x
                  goal_pose.y=y
                  goal_pose.theta=0
                  RC.moveGoal(goal_pose,0.5)
              else:
                  print 'invalid position'
              cont=bool(input('Repeat it? (1/0)'))
        elif option==5:
            RC.goalClean()
        elif option==6:
            RC.gridClean()
        elif option==7:
            RC.spiralClean()
        else:
            print('invalid option')
    else:
        print("In %s put only option number"%sys.argv[0])   

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass