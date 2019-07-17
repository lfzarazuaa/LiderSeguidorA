
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
using namespace std;

const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 11.0;
const double y_max = 11.0;

const double PI=3.14159265359;

void move(double speed,double distance,bool isForward);
void rotate(double angular_speed,double angle,bool clockwise);
double degrees2radians(double angle_in_degrees);
double setDesiredOrientation(double desired_angle_degrees);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
double getDistance(double x1, double y1, double x2, double y2);
void moveGoal(turtlesim::Pose  goal_pose, double distance_tolerance);
void gridClean();
void spiralClean();

int main(int argc, char **argv)
{
        // Initiate new ROS node named "robot_cleaner"
        ros::init(argc, argv, "robot_cleaner");
        ros::NodeHandle n;
        ros::Rate loop_rate(10);
        double speed,angular_speed;
        double distance,angle;
        bool isForward,clockwise;
        velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
        pose_subscriber = n.subscribe("/turtle1/pose",1000,poseCallback);
        /* cout << "enter speed: ";
        cin >> speed;
        cout << "enter distance: ";
        cin >> distance;
        cout << "enter isForward(0/1): ";
        cin >> isForward;
        move(speed, distance, isForward);
        //Angular
        cout << "enter angular speed: ";
        cin >> angular_speed;
        cout << "enter angle: ";
        cin >> angle;
        cout << "enter clockwise(0/1): ";
        cin >> clockwise;
        rotate(angular_speed, angle, clockwise);
        cout << "rotation finished"; */
        //ros::Rate loop_rate(0.5);
        
        /* setDesiredOrientation(120);
             loop_rate.sleep();
        setDesiredOrientation(270);
             loop_rate.sleep();
        setDesiredOrientation(0); */

        /* turtlesim::Pose goal_pose;
        goal_pose.x=1;
        goal_pose.y=1;
        goal_pose.theta=0;
        moveGoal(goal_pose,0.5);
        goal_pose.x=4;
        goal_pose.y=8;
        goal_pose.theta=0;
        moveGoal(goal_pose,0.5);
        goal_pose.x=8;
        goal_pose.y=10;
        goal_pose.theta=0;
        moveGoal(goal_pose,0.5);
        goal_pose.x=5.544;
        goal_pose.y=5.544;
        goal_pose.theta=0;
        moveGoal(goal_pose,0.2);
        loop_rate.sleep(); */
        
        //gridClean();
        
        spiralClean();
        ros::spin();
        return 0;
}
/**
 * @brief move- makes the robot move with a certain linear velocity in a certain distance
 * @param speed- linear velocity
 * @param distance- relative distance to move
 * @param isForward- direction of the move
 */
void move(double speed,double distance,bool isForward){
  //distance=speed*time
  geometry_msgs::Twist vel_msg;
  if (isForward)
     vel_msg.linear.x=abs(speed);
  else
     vel_msg.linear.x=-abs(speed);
  vel_msg.linear.y=0;
  vel_msg.linear.z=0;
  //set to zero angular velocities for a straigth line
  vel_msg.angular.x=0;
  vel_msg.angular.y=0;
  vel_msg.angular.z=0;
  //t0: current time
  //loop
  //publish velocity
  // estimate the distance = speed * (t1-t0)
  //current_distance_moved_by_robot <= distance
  double t0 = ros::Time::now().toSec();
  double current_distance=0;
  ros::Rate loop_rate(100);
  do{
      velocity_publisher.publish(vel_msg);
      double t1=ros::Time::now().toSec();
      current_distance = speed * (t1-t0);
      ros::spinOnce();
  }while(current_distance<distance);
  vel_msg.linear.x=0;
  velocity_publisher.publish(vel_msg);
}


void rotate(double angular_speed,double angle,bool clockwise){
    //angular_speed=degrees2radians(angular_speed);
    //angle=degrees2radians(angle);
    //cout<<"angular_speed= "<<angular_speed<<endl;
    //cout<<"angle= "<<angle<<endl;
    geometry_msgs::Twist vel_msg;
    //set a random linear velocity in the x-axis
    vel_msg.linear.x=0;
    vel_msg.linear.y=0;
    vel_msg.linear.z=0;
    //set a random angular velocity in the z-axis
    vel_msg.angular.x=0;
    vel_msg.angular.y=0;
    if (clockwise){
       vel_msg.angular.z=-abs(angular_speed);
    }
    else{
       vel_msg.angular.z=abs(angular_speed);
    }
    double current_angle=0;
    double t0=ros::Time::now().toSec(),t1;
    ros::Rate loop_rate(100);
    do
    {
       velocity_publisher.publish(vel_msg);
       t1=ros::Time::now().toSec();
       current_angle=angular_speed*(t1-t0);
       ros::spinOnce();
       loop_rate.sleep();
    } while (current_angle<angle);
      vel_msg.angular.z=0;
      velocity_publisher.publish(vel_msg);
}

double degrees2radians(double angle_in_degrees){
    return angle_in_degrees*PI/180.0;
}

double setDesiredOrientation(double desired_angle_degrees){
   double desired_angle_radians=degrees2radians(desired_angle_degrees);
   double relative_angle_radians=desired_angle_radians-turtlesim_pose.theta;
   bool clockwise=((relative_angle_radians<0)?true:false);
   cout<<desired_angle_radians<<" ,"<<turtlesim_pose.theta<<" ,"<<relative_angle_radians<<endl;
   rotate (abs(relative_angle_radians)/10,abs(relative_angle_radians),clockwise);
   cout<<"rotation finished at: "<<desired_angle_degrees<<endl;
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
     turtlesim_pose.x=pose_message->x;
     turtlesim_pose.y=pose_message->y;
     turtlesim_pose.theta=pose_message->theta;
}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

void moveGoal(turtlesim::Pose goal_pose,double distance_tolerance){

	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(20);

	do{
		/****** Proportional Controller ******/
		//linear velocity in the x-axis
		vel_msg.linear.x = 1.5*getDistance(turtlesim_pose.x,turtlesim_pose.y,goal_pose.x,goal_pose.y);
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		//angular velocity in the z-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =4*(atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x-turtlesim_pose.x)-turtlesim_pose.theta);

		velocity_publisher.publish(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>distance_tolerance);
	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
   cout<<"end move goal"<<endl;
}

void gridClean(){
  ros::Rate loop(0.5);
  turtlesim::Pose pose;
  pose.x=1;
  pose.y=1;
  pose.theta=0;
  moveGoal(pose,0.01);
  loop.sleep();
  setDesiredOrientation(0);
  loop.sleep();

  move(2,9,true);
  loop.sleep();
  rotate (degrees2radians(10),degrees2radians(90),false);
  loop.sleep();
  move(2,9,true);

  rotate(degrees2radians(10),degrees2radians(90),false);
  loop.sleep();
  move(2,1,true);
  rotate(degrees2radians(10),degrees2radians(90),false);
  loop.sleep();
  move(2,9,true);

  rotate(degrees2radians(10),degrees2radians(90),false);
  loop.sleep();
  move(2,1,true);
  rotate(degrees2radians(10),degrees2radians(90),false);
  loop.sleep();
  move(2,9,true);
}

void spiralClean(){
   geometry_msgs::Twist vel_msg;
   double count=0;

   double constant_speed=4;
   double vk=1, wk=2, rk=0.5;
   ros::Rate loop(1);
   do{
      rk=rk+0.5;
		//linear velocity in the x-axis
		vel_msg.linear.x = rk;
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		//angular velocity in the z-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =constant_speed;
      cout<<"vel_msg.linear.x = "<<vel_msg.linear.x<<endl;
      cout<<"vel_msg.linear.z = "<<vel_msg.angular.z<<endl;

		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop.sleep();
   }while((turtlesim_pose.x<10.5)&&(turtlesim_pose.y<10.5));
   vel_msg.linear.x =0;
   vel_msg.angular.z =0;
   velocity_publisher.publish(vel_msg);
}

