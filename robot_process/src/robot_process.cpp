#include "robot_process/robot_process.h"

namespace robot_process {

  RobotProcess::RobotProcess(int argc, char* argv[])
  {
    ros::init(argc, argv, "robot_process", ros::init_options::AnonymousName);
    node_name_ = ros::this_node::getName();
  }

  RobotProcess::RobotProcess(int argc, char* argv[], std::string name)
    : node_name_(name)
  {
    ros::init(argc, argv, name);
  }

  RobotProcess::~RobotProcess()
  {
    sleep(4);
  }

  void RobotProcess::run(bool autostart)
  {
    create();
  }

  void RobotProcess::create()
  {
    node_handle_ = ros::NodeHandlePtr(new ros::NodeHandle);
    node_handle_private_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    float heartbeat_rate;
    ros::param::param<float>("~heartbeat_rate", heartbeat_rate, 1.0f);
    ROS_INFO("heartbeat_rate = %.3f [Hz]", heartbeat_rate);

    heartbeat_thread_ = std::thread([&heartbeat_rate]()
    {
      ros::Rate rate(heartbeat_rate);
      while (ros::ok())
      {
        ROS_INFO("heartbeat");



        rate.sleep();
      }
    });

  }

  void RobotProcess::configure()
  {
    ros::param::param<bool>("~autostart", autostart_, false);
    ROS_INFO("autostart = %d", autostart_);


  }

  void RobotProcess::start()
  {

  }

  void RobotProcess::stop()
  {

  }

  void RobotProcess::resume()
  {

  }

  void RobotProcess::pause()
  {

  }

} // namespace robot_process
