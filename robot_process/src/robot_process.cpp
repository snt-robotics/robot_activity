#include "robot_process/robot_process.h"

#define PRINT_DEBUG_STATE(state) \
  ROS_INFO("[%s] ["#state"] METHOD INVOKED", node_name_.c_str())


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
    ROS_INFO("RobotProcess destructor");
  }

  void RobotProcess::run(bool autostart)
  {
    ROS_INFO("[%s] [RUN] METHOD INVOKED", node_name_.c_str());

    transitionToState(State::RUNNING);

  }

  void RobotProcess::create()
  {
    PRINT_DEBUG_STATE("CREATE");

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

    //changeState(State::UNCON)
    //terminate_service_server_ = node_handle_->advertiseService("terminate", )

  }

  void RobotProcess::terminate()
  {
    PRINT_DEBUG_STATE("TERMINATE");

  }

  void RobotProcess::configure()
  {
    ROS_INFO("[%s] [CONFIGURE] METHOD INVOKED", node_name_.c_str());

    ros::param::param<bool>("~autostart", autostart_, false);
    ROS_INFO("autostart = %d", autostart_);

  }

  void RobotProcess::unconfigure()
  {
    ROS_INFO("[%s] [UNCONFIGURE] METHOD INVOKED", node_name_.c_str());

  }

  void RobotProcess::start()
  {
    ROS_INFO("[%s] [START] METHOD INVOKED", node_name_.c_str());
  }

  void RobotProcess::stop()
  {
    ROS_INFO("[%s] [STOP] METHOD INVOKED", node_name_.c_str());
  }

  void RobotProcess::resume()
  {
    ROS_INFO("[%s] [RESUME] METHOD INVOKED", node_name_.c_str());

  }

  void RobotProcess::pause()
  {
    ROS_INFO("[%s] [PAUSE] METHOD INVOKED", node_name_.c_str());

  }

  void RobotProcess::notifyState() const
  {
    robot_process_msgs::State state_msg;
    state_msg.header.stamp = ros::Time::now();
    state_msg.node_name = node_name_;
    state_msg.state =  static_cast<uint8_t>(current_state_);
    process_state_pub_.publish(state_msg);
  }

  void RobotProcess::transitionToState(const State& goal_state)
  {
    while (current_state_ != goal_state)
    {
      uint8_t from_state = static_cast<uint8_t>(current_state_);
      uint8_t to_state = static_cast<uint8_t>(goal_state);
      State next_state = STATE_TRANSITIONS_PATHS[from_state][to_state];
      ROS_INFO_STREAM( current_state_ << " " << goal_state << " " << next_state );
      if (next_state == State::INVALID)
      {
        ROS_FATAL_STREAM( "There is no transition path from [" << current_state_
          << "] to [" << goal_state << "]" );
        return;
      }
      changeState(next_state);
      ROS_INFO("1");
    }
  }

  void RobotProcess::changeState(const State& new_state)
  {
    uint8_t from_state = static_cast<uint8_t>(current_state_);
    uint8_t to_state = static_cast<uint8_t>(new_state);
    TransitionCallback callback = STATE_TRANSITIONS[from_state][to_state];
    if (callback == nullptr)
    {
      ROS_FATAL_STREAM( "Tried changing state from [" << current_state_
        << "] to [" << new_state << "]. Transition does NOT exist!" );
      return;
    }
    ROS_INFO_STREAM( "Changing state from [" << current_state_
      << "] to [" << new_state << "]");
    current_state_ = new_state;
    //callback(*this);
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

  const RobotProcess::StateTransitions RobotProcess::STATE_TRANSITIONS =
  {
    {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr}, // State::INVALID
    {nullptr, nullptr, &RobotProcess::create, nullptr, nullptr, nullptr, nullptr}, // State::LAUNCHING
    {nullptr, nullptr, nullptr, &RobotProcess::configure, nullptr, nullptr, &RobotProcess::terminate}, // State::UNCONFIGURED
    {nullptr, nullptr, &RobotProcess::unconfigure, nullptr, &RobotProcess::start, nullptr, nullptr}, // State::STOPPED
    {nullptr, nullptr, nullptr, &RobotProcess::stop, nullptr, &RobotProcess::resume, nullptr}, // State::PAUSED
    {nullptr, nullptr, nullptr, nullptr, nullptr, &RobotProcess::pause, nullptr}, // State::RUNNING
    {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr} // State::TERMINATED
  };


  const RobotProcess::StateTransitionPaths RobotProcess::STATE_TRANSITIONS_PATHS =
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
