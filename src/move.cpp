#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <sensor_msgs/Joy.h>






geometry_msgs::Twist mes;
int linear_, angular_;
double l_scale_, a_scale_;
ros::NodeHandle n;
ros::Subscriber subLaser;
ros::Publisher robot1_pub;


void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    ROS_INFO("hello");
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*joy->axes[angular_];
    twist.linear.x = l_scale_*joy->axes[linear_];
    robot1_pub.publish(twist);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move");


ROS_INFO("hello");


  n.param("axis_linear", linear_, linear_);
  n.param("axis_angular", angular_, angular_);
  n.param("scale_angular", a_scale_, a_scale_);
  n.param("scale_linear", l_scale_, l_scale_);

  ros::Publisher robot1_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_raw", 1);
  ros::Subscriber subLaser =n.subscribe<sensor_msgs::Joy>("joy",10, joyCallback);



  ros::Rate loop_rate(10);
  mes.linear.x=0.5;

  int count = 0;
  while (ros::ok())
  {


    robot1_pub.publish(mes);


    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }



  return 0;
}

