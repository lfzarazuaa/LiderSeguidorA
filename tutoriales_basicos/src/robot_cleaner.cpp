#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>
ros::Publisher velocity_publisher;
using namespace std;

void move(double speed,double distance,bool isForward);

/* argc = number of arguements.
 * argv = strings of all arguments argcv[0]=location.
 */

int main(int argc, char **argv)
{
        // Initiate new ROS node named "robot_cleaner"
        ros::init(argc, argv, "robot_cleaner");
        ros::NodeHandle n;
        double speed;
        double distance;
        bool isForward;
        if (argc == 3){
            speed=atof(argv[1]);
            distance=atof(argv[2]);
            isForward=true;}
        else if(argc == 4){
            speed=atof(argv[1]);
            distance=atof(argv[2]);
            isForward=atoi(argv[3]);}
        else{
            ROS_INFO("usage: move speed distance isForward");
            return 1;}
        cout<<"isForward="<<isForward<<endl;
        velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
        move(speed,distance,isForward);
}
/**
 * @brief move- makes the robot move with a certain linear velocity in a certain distance.
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
  ros::Rate loop_rate(10);
  do{
      velocity_publisher.publish(vel_msg);
      double t1=ros::Time::now().toSec();
      current_distance = speed * (t1-t0);
      ros::spinOnce();
  }while(current_distance<distance);
  vel_msg.linear.x=0;
  velocity_publisher.publish(vel_msg);
}
