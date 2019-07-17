#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
//#include </usr/include/x86_64-linux-gnu/qt5/QtWidgets/QApplication>
//#include <QApplication>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "nodo_gui");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
