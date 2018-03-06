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
#include <ros/callback_queue.h>

#include <std_srvs/Empty.h>
#include <robot_process_msgs/State.h>
#include <robot_process_msgs/Error.h>

#include "robot_process/isolated_async_timer.h"

namespace robot_process {

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
  ros::ServiceServer reconfigure_service_server_;
  ros::ServiceServer restart_service_server_;
  ros::ServiceServer start_service_server_;
  ros::ServiceServer stop_service_server_;
  ros::ServiceServer resume_service_server_;
  ros::ServiceServer pause_service_server_;

  ros::Publisher process_state_pub_;
  ros::Publisher process_error_pub_;

  uint8_t node_state_ = 0;

  std::shared_ptr<robot_process::IsolatedAsyncTimer> heartbeat_timer_;

  void create();
  void terminate();

  void configure();
  void unconfigure();

  void start();
  void stop();

  void resume();
  void pause();

  void notifyState();

};

} // namespace robot_process

#endif
