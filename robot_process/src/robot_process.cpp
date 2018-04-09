#include <robot_process/robot_process.h>

#define PRINT_FUNC_CALL(state) \
  ROS_DEBUG_STREAM(#state << "() method called")

namespace robot_process {

RobotProcess::RobotProcess(int argc, char* argv[],
                           const std::string& name_space,
                           const std::string& name)
  : node_namespace_(name_space),
    node_name_(name),
    state_request_callback_queue_()
{
  for (int i = 0; i < argc; ++i)
  {
    std::cout << argv[i] << std::endl;
  }
  std::cout << std::endl;


  if (ros::isInitialized())
  {
    node_name_ = ros::this_node::getName();
    return;
  }

  if (node_name_.empty())
  {
    ros::init(argc, argv, "robot_process", ros::init_options::AnonymousName);
    node_name_ = ros::this_node::getName();
  }
  else
    ros::init(argc, argv, name);
}

RobotProcess::~RobotProcess()
{
  ROS_DEBUG_STREAM("RobotProcess dtor [" << getNamespace() << "]");
}

RobotProcess& RobotProcess::init(bool autostart)
{

  node_handle_ = ros::NodeHandlePtr(new ros::NodeHandle(node_namespace_));
  node_handle_private_ = ros::NodeHandlePtr(new ros::NodeHandle("~" + node_namespace_));

  ros::param::param<bool>("~wait_for_supervisor", wait_for_supervisor_, true);
  ROS_INFO_STREAM("wait_for_supervisor = "
    << std::boolalpha << wait_for_supervisor_);

  process_state_pub_ = node_handle_private_->advertise<robot_process_msgs::State>("/heartbeat", 0, true);
  process_error_pub_ = node_handle_private_->advertise<robot_process_msgs::Error>("/error", 0, true);

  if (wait_for_supervisor_)
  {
    ros::Rate poll_rate(100);
    while (process_state_pub_.getNumSubscribers() == 0 && ros::ok())
      poll_rate.sleep();
  }

  notifyState();

  terminate_server_ = registerStateChangeRequest("terminate", {State::TERMINATED});
  reconfigure_server_ = registerStateChangeRequest("reconfigure", {
    State::UNCONFIGURED,
    autostart_after_reconfigure_ ? State::RUNNING : State::STOPPED
  });

  restart_server_ = registerStateChangeRequest("restart", {State::STOPPED, State::RUNNING});
  start_server_   = registerStateChangeRequest("start", {State::RUNNING});
  stop_server_    = registerStateChangeRequest("stop",  {State::STOPPED});
  pause_server_   = registerStateChangeRequest("pause", {State::PAUSED});

  float heartbeat_rate;
  ros::param::param<float>("~heartbeat_rate", heartbeat_rate, 1.0f);
  ROS_INFO("heartbeat_rate = %.3f [Hz]", heartbeat_rate);

  IsolatedAsyncTimer::LambdaCallback heartbeat_callback = [this]() { notifyState(); };
  heartbeat_timer_ = std::make_shared<IsolatedAsyncTimer>(
    *node_handle_private_,
    heartbeat_callback,
    heartbeat_rate,
    false);

  ros::param::param<bool>("~autostart", autostart_, false);

  state_request_spinner_ = std::make_shared<ros::AsyncSpinner>(1, &state_request_callback_queue_);
  state_request_spinner_->start();

  autostart_ = autostart_ || autostart;
  ROS_INFO_STREAM("autostart = " << std::boolalpha << autostart_);

  if (autostart_)
    transitionToState(State::RUNNING);
  else
    transitionToState(State::STOPPED);

  return *this;
}

void RobotProcess::run(uint8_t threads)
{
  runAsync(threads);
  ros::waitForShutdown();
}

void RobotProcess::runAsync(uint8_t threads)
{
  global_callback_queue_spinner_ = std::make_shared<ros::AsyncSpinner>(threads);
  global_callback_queue_spinner_->start();
}

State RobotProcess::getState()
{
  return current_state_;
}

void RobotProcess::notifyError(uint8_t error_type,
                 const std::string& function,
                 const std::string& description)
{
  ROS_DEBUG_STREAM("Publishing error msg with code: "
    << error_type << " function: " << function
    << " description: " << description);
  robot_process_msgs::Error error_msg;
  error_msg.header.stamp = ros::Time::now();
  error_msg.node_name = getNamespace();
  error_msg.error_type = error_type;
  error_msg.function = function;
  error_msg.description = description;
  process_error_pub_.publish(error_msg);
}

void RobotProcess::registerIsolatedTimer(
  const IsolatedAsyncTimer::LambdaCallback& callback,
  const float& frequency,
  bool stoppable)
{
  process_timers_.emplace_back(
    std::make_shared<IsolatedAsyncTimer>(
      *node_handle_private_,
      callback,
      frequency,
      stoppable,
      false,
      false
    ));
}

std::string RobotProcess::getNamespace() const
{
  if (node_handle_private_)
    return node_handle_private_->getNamespace();
  else
    return std::string();
}


void RobotProcess::create()
{
  PRINT_FUNC_CALL("create");
  onCreate();
}

void RobotProcess::terminate()
{
  PRINT_FUNC_CALL("terminate");
  onTerminate();
  ros::Rate(2).sleep();
  ros::shutdown();
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
  for(const auto& timer : process_timers_)
  {
    ROS_DEBUG("Starting timer");
    timer->start();
  }
  onStart();
}

void RobotProcess::stop()
{
  PRINT_FUNC_CALL("stop");
  for(const auto& timer : process_timers_)
  {
    ROS_DEBUG("Stopping timer");
    timer->stop();
  }
  onStop();
}

void RobotProcess::resume()
{
  PRINT_FUNC_CALL("resume");
  for(const auto& timer : process_timers_)
  {
    ROS_DEBUG("Resuming timer");
    timer->resume();
  }
  onResume();
}

void RobotProcess::pause()
{
  PRINT_FUNC_CALL("pause");
  for(const auto& timer : process_timers_)
  {
    ROS_DEBUG("Pausing timer");
    timer->pause();
  }
  onPause();
}

ros::ServiceServer RobotProcess::registerStateChangeRequest(
  const std::string& service_name,
  const std::vector<State>& states)
{
  ROS_DEBUG_STREAM(
    "Registering state transition request for state " << service_name);
  using namespace std_srvs;
  EmptyServiceCallback callback = [=](Empty::Request& req, Empty::Response& res)
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

  return node_handle_private_->advertiseService(options);
}

void RobotProcess::notifyState() const
{
  ROS_DEBUG("Heartbeat sent!");
  robot_process_msgs::State state_msg;
  state_msg.header.stamp = ros::Time::now();
  state_msg.node_name = getNamespace();
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
  MemberLambdaCallback callback = STATE_TRANSITIONS[from_state][to_state];
  if (callback == nullptr)
  {
    ROS_FATAL_STREAM_ONCE(
      "Tried changing state from [" << current_state_
      << "] to [" << new_state << "]. Transition does NOT exist!");
    return;
  }
  ROS_DEBUG_STREAM(
    "Changing state from [" << current_state_ << "] to ["
    << new_state << "]");
  current_state_ = new_state;

  boost::bind(callback, this)();
  //cb();
  //(this->*callback)();
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
  {nullptr, nullptr, nullptr, nullptr, &RobotProcess::pause, nullptr, nullptr}, // State::RUNNING
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
