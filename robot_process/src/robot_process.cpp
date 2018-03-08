#include "robot_process/robot_process.h"

#define PRINT_FUNC_CALL(state) \
  ROS_DEBUG_STREAM(#state << "() method called")

namespace robot_process {

  RobotProcess::RobotProcess(int argc, char* argv[])
    : state_request_callback_queue_()
  {
    ros::init(argc, argv, "robot_process", ros::init_options::AnonymousName);
    node_name_ = ros::this_node::getName();
  }

  RobotProcess::RobotProcess(int argc, char* argv[], std::string name)
    : node_name_(name),
      state_request_callback_queue_()
  {
    ros::init(argc, argv, name);
  }

  RobotProcess::~RobotProcess()
  {
    ROS_DEBUG("RobotProcess destructor");
  }

  void RobotProcess::run(bool autostart)
  {
    autostart_ = autostart;
    transitionToState(State::UNCONFIGURED);
  }

  void RobotProcess::create()
  {
    PRINT_FUNC_CALL("create");

    node_handle_ = ros::NodeHandlePtr(new ros::NodeHandle);
    node_handle_private_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    process_state_pub_ = node_handle_->advertise<robot_process_msgs::State>("heartbeat", 1);

    float heartbeat_rate;
    ros::param::param<float>("~heartbeat_rate", heartbeat_rate, 1.0f);
    ROS_DEBUG("heartbeat_rate = %.3f [Hz]", heartbeat_rate);

    boost::function<void(void)> heartbeat_callback = [this]() { notifyState(); };
    heartbeat_timer_ = std::make_shared<robot_process::IsolatedAsyncTimer>(
      *node_handle_,
      heartbeat_callback,
      1.0f);

    bool autostart;
    ros::param::param<bool>("~autostart", autostart, false);
    autostart_ = autostart_ || autostart;
    ROS_DEBUG("autostart = %d", autostart_);

    registerStateChangeRequest("terminate", std::vector<State> {State::TERMINATED});

    if (autostart_)
      transitionToState(State::RUNNING);
    else
      transitionToState(State::STOPPED);

  }

  void RobotProcess::terminate()
  {
    PRINT_FUNC_CALL("terminate");
    onTerminate();
  }

  void RobotProcess::configure()
  {
    PRINT_FUNC_CALL("configure");
    onConfigure();
  }

  void RobotProcess::unconfigure()
  {
    PRINT_FUNC_CALL("unconfigure");
    onUnconfigure();
  }

  void RobotProcess::start()
  {
    PRINT_FUNC_CALL("start");
    onStart();
  }

  void RobotProcess::stop()
  {
    PRINT_FUNC_CALL("stop");
    onStop();
  }

  void RobotProcess::resume()
  {
    PRINT_FUNC_CALL("resume");
    onResume();
  }

  void RobotProcess::pause()
  {
    PRINT_FUNC_CALL("pause");
    onPause();
  }

  void RobotProcess::registerStateChangeRequest(
    const std::string& service_name,
    const std::vector<State>& states)
  {
    using namespace std_srvs;
    EmptyServiceCallback callback = [&](Empty::Request& req, Empty::Response& res)
    {
      bool success = true;
      for (const auto& s : states)
      {
        success = success && transitionToState(s);
      }
      return success;
    };

    auto options = ros::AdvertiseServiceOptions::create<Empty>(
      service_name,
      callback,
      ros::VoidConstPtr(),
      &state_request_callback_queue_
    );

    node_handle_->advertiseService(options);
  }

  void RobotProcess::notifyState() const
  {
    ROS_DEBUG("Heartbeat sent!");
    robot_process_msgs::State state_msg;
    state_msg.header.stamp = ros::Time::now();
    state_msg.node_name = node_name_;
    state_msg.state = static_cast<uint8_t>(current_state_);
    process_state_pub_.publish(state_msg);
  }

  bool RobotProcess::transitionToState(const State& goal_state)
  {
    if (current_state_ == goal_state)
    {
      ROS_WARN_STREAM( "Node is already at state " << goal_state );
      return false;
    }

    while (current_state_ != goal_state)
    {
      auto from_state = static_cast<uint8_t>(current_state_);
      auto to_state = static_cast<uint8_t>(goal_state);
      State next_state = STATE_TRANSITIONS_PATHS[from_state][to_state];
      if (next_state == State::INVALID)
      {
        ROS_WARN_STREAM( "There is no transition path from [" << current_state_
          << "] to [" << goal_state << "]" );
        return false;
      }
      changeState(next_state);
    }
    return true;
  }

  void RobotProcess::changeState(const State& new_state)
  {
    uint8_t from_state = static_cast<uint8_t>(current_state_);
    uint8_t to_state = static_cast<uint8_t>(new_state);
    TransitionCallback callback = STATE_TRANSITIONS[from_state][to_state];
    if (callback == nullptr)
    {
      ROS_FATAL_STREAM(
        "Tried changing state from [" << current_state_
        << "] to [" << new_state << "]. Transition does NOT exist!");
      return;
    }
    ROS_DEBUG_STREAM(
      "Changing state from [" << current_state_ << "] to ["
      << new_state << "]");
    current_state_ = new_state;
    (this->*callback)();
    notifyState();
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
