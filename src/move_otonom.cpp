#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32MultiArray.h>


class MoveOtonom
{
public:
  MoveOtonom();

private:
  void tipCallback(const std_msgs::Int8::ConstPtr& tip);

  ros::NodeHandle nh_;

  ros::Publisher vel_pub_;
  ros::Publisher palet_pub;
  ros::Subscriber tip_sub;

  double scale;

  std_msgs::Float32MultiArray msg;

};


MoveOtonom::MoveOtonom()
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

  tip_sub = nh_.subscribe<std_msgs::Int8>("tip", 10, &MoveOtonom::tipCallback, this);

}

void MoveOtonom::tipCallback(const std_msgs::Int8::ConstPtr& tip)
{


    // TODO: Otonom hareket komutlari burada hazirlanacaktir!

    /*
     * Return değerleri:
     *
     * 0 - Zemin
     * 1 - Duvar
     * 2 - Çukur
     * 3 - Kaldırım
     * 4 - Rampa
     * 5 - Merdiven
     */

    std::cout << std::endl << "Move Otonom" << std::endl;

    switch (tip->data)  {
        case 0:
            std::cout << "Zemin" << std::endl;
            break;
        case 1:
            std::cout << "Duvar" << std::endl;
            break;
        case 2:
            std::cout << "Cukur" << std::endl;
            break;
        case 3:
            std::cout << "Kaldirim" << std::endl;
            break;
        case 4:
            std::cout << "Rampa" << std::endl;
            break;
        case 5:
            std::cout << "Merdiven" << std::endl;
            break;
        case 6:
        default:
            std::cout << "Tanınmayan ortam" << std::endl;
    }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_otonom");

  MoveOtonom move_otonom;

  ros::spin();
}
