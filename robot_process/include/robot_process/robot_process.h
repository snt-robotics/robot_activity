/*!
   \file robot_process.h
   \brief RobotProcess class implements ROS node lifecycle
   \author Maciej Marcin ZURAD
   \date 01/03/2018
*/
#ifndef ROBOT_PROCESS_H
#define ROBOT_PROCESS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <std_srvs/Empty.h>
#include <robot_process_msgs/State.h>
#include <robot_process_msgs/Error.h>

#include <robot_process/isolated_async_timer.h>

namespace robot_process {

enum class State : std::uint8_t {
  INVALID      = robot_process_msgs::State::INVALID,
  LAUNCHING    = robot_process_msgs::State::LAUNCHING,
  UNCONFIGURED = robot_process_msgs::State::UNCONFIGURED,
  STOPPED      = robot_process_msgs::State::STOPPED,
  PAUSED       = robot_process_msgs::State::PAUSED,
  RUNNING      = robot_process_msgs::State::RUNNING,
  TERMINATED   = robot_process_msgs::State::TERMINATED,
  Count = 7
};

std::ostream& operator<<(std::ostream& os, State state);

class RobotProcess
{
public:

  RobotProcess() = delete;

  RobotProcess(int argc, char* argv[],
    const std::string& name_space = {},
    const std::string& name = {});
  virtual ~RobotProcess();

  RobotProcess& init(bool autostart = false);
  void run(uint8_t threads = 0) const;
  void runAsync(uint8_t threads = 0) const;

protected:

  ros::NodeHandlePtr node_handle_;
  ros::NodeHandlePtr node_handle_private_;

  void notifyError(uint8_t error_type,
                   const std::string& function,
                   const std::string& description);

  void registerIsolatedTimer(const IsolatedAsyncTimer::LambdaCallback& callback,
                             const float& frequency,
                             bool stoppable = true);

  const std::string& getNamespace() const;

private:

  std::string node_namespace_;
  std::string node_name_;

  bool wait_for_supervisor_ = true;

  bool autostart_ = false;
  bool autostart_after_reconfigure_ = false;

  ros::CallbackQueue state_request_callback_queue_;
  std::shared_ptr<ros::AsyncSpinner> state_request_spinner_;

  ros::ServiceServer terminate_server_;
  ros::ServiceServer reconfigure_server_;
  ros::ServiceServer restart_server_;
  ros::ServiceServer start_server_;
  ros::ServiceServer stop_server_;
  ros::ServiceServer pause_server_;

  ros::Publisher process_state_pub_;
  ros::Publisher process_error_pub_;

  State current_state_ = State::LAUNCHING;

  std::shared_ptr<robot_process::IsolatedAsyncTimer> heartbeat_timer_;

  std::vector<std::shared_ptr<robot_process::IsolatedAsyncTimer>> process_timers_;

  virtual void onCreate() = 0;
  virtual void onTerminate() = 0;

  virtual void onConfigure() = 0;
  virtual void onUnconfigure() = 0;

  virtual void onStart() = 0;
  virtual void onStop() = 0;

  virtual void onPause() = 0;
  virtual void onResume() = 0;

  void create();
  void terminate();

  void configure();
  void unconfigure();

  void start();
  void stop();

  void resume();
  void pause();

  void notifyState() const;
  void changeState(const State& new_state);
  bool transitionToState(const State& new_state);

  ros::ServiceServer registerStateChangeRequest(
    const std::string& service_name,
    const std::vector<State>& states);

  typedef void (RobotProcess::*MemberLambdaCallback)();

  typedef boost::function<bool(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)> EmptyServiceCallback;
  typedef MemberLambdaCallback StateTransitions
    [static_cast<uint8_t>(State::Count)]
    [static_cast<uint8_t>(State::Count)];

  typedef State StateTransitionPaths
    [static_cast<uint8_t>(State::Count)]
    [static_cast<uint8_t>(State::Count)];

  const static StateTransitions STATE_TRANSITIONS;
  const static StateTransitionPaths STATE_TRANSITIONS_PATHS;

};

} // namespace robot_process

#endif
