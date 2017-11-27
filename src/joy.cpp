#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>



class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Publisher palet_pub;

  double scale;

  std_msgs::Float32MultiArray msg;


};


TeleopTurtle::TeleopTurtle()
{

scale=1;
// set up dimensions
std::vector<double> vec1 = { 0, 0};
msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
msg.layout.dim[0].size = vec1.size();
msg.layout.dim[0].stride = 1;
msg.layout.dim[0].label = "x";

// copy in the data
msg.data.clear();
msg.data.insert(msg.data.end(), vec1.begin(), vec1.end());


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_raw", 1);

  palet_pub = nh_.advertise<std_msgs::Float32MultiArray>("/box/acilar", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    if(joy->buttons[4]==1)
    {
        if(msg.data.at(0)<1.5)
        {
            msg.data.at(0)+=0.1;
        }

    }
    if(joy->buttons[6]==1)
    {
        if(msg.data.at(0)>-1.5)
        {
            msg.data.at(0)-=0.1;
        }

    }
    if(joy->buttons[5]==1)
    {
        if(msg.data.at(1)<1.5)
        {
            msg.data.at(1)+=0.1;
        }

    }
    if(joy->buttons[7]==1)
    {
        if(msg.data.at(1)>-1.5)
        {
            msg.data.at(1)-=0.1;
        }

    }

    if(joy->buttons[0]==1 && scale<3)
    {
        scale+=0.2;
    }
    if(joy->buttons[2]==1 && scale>0.1)
    {
        scale-=0.2;
    }

  geometry_msgs::Twist twist;
  twist.linear.x=scale*joy->axes[1];
  twist.angular.z=scale*joy->axes[0];
  vel_pub_.publish(twist);
  palet_pub.publish(msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
