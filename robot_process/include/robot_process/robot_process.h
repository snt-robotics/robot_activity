/*!
   \file robot_process.h
   \brief RobotProcess class implements ROS node lifecycle
   \author Maciej Marcin ZURAD
   \date 01/03/2018
*/
#ifndef ROBOT_PROCESS_H
#define ROBOT_PROCESS_H

#include <string>
#include <thread>

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <robot_process_msgs/State.h>
#include <robot_process_msgs/Error.h>

#include "robot_process/isolated_async_timer.h"

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

  RobotProcess(int argc, char* argv[]);
  RobotProcess(int argc, char* argv[], std::string name);
  ~RobotProcess();

  void run(bool autostart = false);

protected:

  ros::NodeHandlePtr node_handle_;
  ros::NodeHandlePtr node_handle_private_;

  virtual void onCreate() {};
  virtual void onTerminate() {};

  virtual void onConfigure() {};
  virtual void onUnconfigure() {};

  virtual void onStart() {};
  virtual void onStop() {};

  virtual void onPause() {};
  virtual void onResume() {};

private:

  bool autostart_ = false;

  std::string node_name_;

  ros::ServiceServer terminate_service_server_;
  bool terminate_service_callback(std_srvs::Empty::Request& req,
                                  std_srvs::Empty::Response& res);

  ros::ServiceServer reconfigure_service_server_;
  bool reconfigure_service_callback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);

  ros::ServiceServer restart_service_server_;

  ros::ServiceServer start_service_server_;
  ros::ServiceServer stop_service_server_;

  ros::ServiceServer resume_service_server_;
  ros::ServiceServer pause_service_server_;

  ros::Publisher process_state_pub_;
  ros::Publisher process_error_pub_;

  State current_state_ = State::LAUNCHING;

  std::shared_ptr<robot_process::IsolatedAsyncTimer> heartbeat_timer_;

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
  void transitionToState(const State& new_state);


  //typedef void (RobotProcess::*TransitionCallback)();
  typedef std::function<void(const RobotProcess)> TransitionCallback;
  typedef TransitionCallback StateTransitions
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
