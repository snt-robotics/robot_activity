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

    boost::function<void(void)> heartbeat_callback = [this]() { notifyState(); };
    heartbeat_timer_ = std::make_shared<robot_process::IsolatedAsyncTimer>(
      *node_handle_,
      heartbeat_callback,
      1.0f);

    //terminate_service_server_ = node_handle_->advertiseService("terminate", )

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

  void RobotProcess::notifyState() const
  {
    robot_process_msgs::State state_msg;
    state_msg.header.stamp = ros::Time::now();
    state_msg.node_name = node_name_;
    state_msg.state =  static_cast<uint8_t>(current_state_);
    process_state_pub_.publish(state_msg);
  }

  void RobotProcess::transitionToState(const State& new_state)
  {
    while (current_state_ != new_state)
    {
      /* code */
    }
  }

  void RobotProcess::changeState(const State& new_state)
  {
    switch (new_state) {
      case State:: :
    }
  }

  std::ostream& operator<<(std::ostream& os, State state)
  {
    switch(state)
    {
      case State::INVALID      : os << "INVALID";      break;
      case State::LAUNCHING    : os << "LAUNCHING";    break;
      case State::UNCONFIGURED : os << "UNCONFIGURED"; break;
      case State::STOPPED      : os << "STOPPED";      break;
      case State::PAUSED       : os << "PAUSED";       break;
      case State::RUNNING      : os << "RUNNING";      break;
      case State::TERMINATED   : os << "TERMINATED";   break;
      default                  : os.setstate(std::ios_base::failbit);
    }
    return os;
  }

  const static StateTransitionPaths STATE_TRANSITIONS_PATHS =
  {
    {
      /* State::INVALID to other states */
      State::INVALID, State::INVALID, State::INVALID, State::INVALID,
      State::INVALID, State::INVALID, State::INVALID
    },
    {
      /* State::LAUNCHING to other states */
      State::INVALID, State::LAUNCHING, State::UNCONFIGURED,
      State::UNCONFIGURED, State::UNCONFIGURED, State::UNCONFIGURED,
      State::UNCONFIGURED
    },
    {
      /* State::UNCONFIGURED to other states */
      State::INVALID, State::INVALID, State::UNCONFIGURED,
      State::STOPPED, State::STOPPED, State::STOPPED,
      State::TERMINATED
    },
    {
      /* State::STOPPED to other states */
      State::INVALID, State::INVALID, State::UNCONFIGURED,
      State::STOPPED, State::PAUSED, State::PAUSED,
      State::UNCONFIGURED
    },
    {
      /* State::PAUSED to other states */
      State::INVALID, State::INVALID, State::STOPPED,
      State::STOPPED, State::PAUSED, State::RUNNING,
      State::STOPPED
    },
    {
      /* State::RUNNING to other states */
      State::INVALID, State::INVALID, State::PAUSED,
      State::PAUSED, State::PAUSED, State::PAUSED,
      State::PAUSED
    },
    {
      /* State::TERMINATED to other states */
      State::INVALID, State::INVALID, State::INVALID,
      State::INVALID, State::INVALID, State::INVALID,
      State::INVALID
    },
  };

} // namespace robot_process
