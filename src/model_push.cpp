#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/callback_queue.h>
#include <thread>
#include <geometry_msgs/Twist.h>
namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
            this->model = _parent;
          gazebo::common::PID PID = gazebo::common::PID(800,1,20);

          this->updateConnection = event::Events::ConnectWorldUpdateBegin(
              boost::bind(&ModelPush::OnUpdate, this, _1));

                std::vector<physics::JointPtr> jointler= this->model->GetJoints();

                for (int i=0;i<jointler.size();i++){
                    this->model->GetJointController()->SetPositionPID(jointler[i]->GetScopedName() ,PID);
                    std::cout<<jointler[i]->GetScopedName()<<std::endl;
                }

    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    ros::SubscribeOptions so =
            ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
                "/" + this->model->GetName() + "/acilar",
                1,
                boost::bind(&ModelPush::OnRosMsg, this, _1),
                ros::VoidPtr(), &this->rosQueue);

    ros::SubscribeOptions so_cmd =
            ros::SubscribeOptions::create<geometry_msgs::Twist>(
                "/cmd_vel_raw",
                1,
                boost::bind(&ModelPush::CmdCallBack, this, _1),
                ros::VoidPtr(), &this->rosQueue);

    this->rosSub_cmd = this->rosNode->subscribe(so_cmd);
    this->rosSub = this->rosNode->subscribe(so);
    this->rosQueueThread = std::thread(std::bind(&ModelPush::QueueThread, this));
    ROS_INFO("Hello World!......");


        for (int i=0;i<2;i++){
            this->acilar[i]=0;
        }
    }

  public: void CmdCallBack(const geometry_msgs::TwistConstPtr& _msg)
      {
          this->model->SetLinearVel(math::Vector3(_msg->linear.x*cos(this->model->GetRelativePose().rot.GetYaw()), _msg->linear.x*sin(this->model->GetRelativePose().rot.GetYaw()), 0));
          this->model->SetAngularVel(math::Vector3(0, 0, _msg->angular.z));
      }

  public: void OnRosMsg(const std_msgs::Float32MultiArrayConstPtr &_msg)
      {
          for(int i=0;i<2;i++)
          {
              this->acilar[i]=_msg->data[i];
              //std::cout << this->acilar[i] << " " ;
          }
          //std::cout << std::endl;
      }


      /// \brief ROS helper function that processes messages
  private: void QueueThread()
      {
          static const double timeout = 0.01;
          while (this->rosNode->ok())
          {
              this->rosQueue.callAvailable(ros::WallDuration(timeout));
          }
      }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {

          this->model->GetJointController()->SetPositionTarget("box::front_flipper_joint",-acilar[0]);
          this->model->GetJointController()->SetPositionTarget("box::rear_flipper_joint",acilar[1]);

          this->pose_main = this->model->GetWorldPose();

          this->model->GetJointController()->SetPositionTarget("box::tepsi_x",-this->pose_main.rot.GetRoll());
          this->model->GetJointController()->SetPositionTarget("box::tepsi_y",-this->pose_main.rot.GetPitch());
/*
          this->pose_main = this->model->GetWorldPose();
          this->pose_tepsi = math::Pose(0,0,0,-this->pose_main.rot.GetRoll(), -this->pose_main.rot.GetPitch(),0);
          this->model->GetLink("tepsi")->SetRelativePose(this->pose_tepsi,true);*/


    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

      /// \brief A node use for ROS transport
  private: std::unique_ptr<ros::NodeHandle> rosNode;

      /// \brief A ROS subscriber
  private: ros::Subscriber rosSub;

      /// \brief A ROS callbackqueue that helps process messages
  private: ros::CallbackQueue rosQueue;

      /// \brief A thread the keeps running the rosQueue
  private: std::thread rosQueueThread;

  private: ros::Subscriber rosSub_cmd;

  private: math::Pose pose_main, pose_tepsi;
  private: double acilar[2];
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
