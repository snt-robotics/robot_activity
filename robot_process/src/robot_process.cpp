#include "robot_process/robot_process.h"

#define PRINT_DEBUG_STATE(state) \
  ROS_DEBUG("[%s] [#state] METHOD INVOKED", node_name_.c_str())


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
    ROS_INFO("2");
  }

  void RobotProcess::run(bool autostart)
  {
    ROS_DEBUG("[%s] [RUN] METHOD INVOKED", node_name_.c_str());

    create();

    sleep(2);

  }

  void RobotProcess::create()
  {
    PRINT_DEBUG_STATE("TERMINATE");

    node_handle_ = ros::NodeHandlePtr(new ros::NodeHandle);
    node_handle_private_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    process_state_pub_ = node_handle_->advertise<robot_process_msgs::State>("heartbeat", 1);

    float heartbeat_rate;
    ros::param::param<float>("~heartbeat_rate", heartbeat_rate, 1.0f);
    ROS_INFO("heartbeat_rate = %.3f [Hz]", heartbeat_rate);

    boost::function<void(const ros::TimerEvent& e)> heartbeat_callback =
    [this](const ros::TimerEvent& e)
    {
      notifyState();
      ROS_INFO("Sent heartbeat");
    };

    heartbeat_timer_ = std::make_shared<robot_process::IsolatedAsyncTimer>(*node_handle_, heartbeat_callback, 1.0f);



  }

  void RobotProcess::terminate()
  {
    PRINT_DEBUG_STATE("TERMINATE");

  }

  void RobotProcess::configure()
  {
    ROS_DEBUG("[%s] [CONFIGURE] METHOD INVOKED", node_name_.c_str());

    ros::param::param<bool>("~autostart", autostart_, false);
    ROS_INFO("autostart = %d", autostart_);

  }

  void RobotProcess::unconfigure()
  {
    ROS_DEBUG("[%s] [UNCONFIGURE] METHOD INVOKED", node_name_.c_str());

  }

  void RobotProcess::start()
  {
    ROS_DEBUG("[%s] [START] METHOD INVOKED", node_name_.c_str());
  }

  void RobotProcess::stop()
  {
    ROS_DEBUG("[%s] [STOP] METHOD INVOKED", node_name_.c_str());
  }

  void RobotProcess::resume()
  {
    ROS_DEBUG("[%s] [RESUME] METHOD INVOKED", node_name_.c_str());

  }

  void RobotProcess::pause()
  {
    ROS_DEBUG("[%s] [PAUSE] METHOD INVOKED", node_name_.c_str());

  }

  void RobotProcess::notifyState()
  {
    robot_process_msgs::State state_msg;
    state_msg.header.stamp = ros::Time::now();
    state_msg.node_name = node_name_;
    state_msg.state = node_state_;
    process_state_pub_.publish(state_msg);
  }


} // namespace robot_process
