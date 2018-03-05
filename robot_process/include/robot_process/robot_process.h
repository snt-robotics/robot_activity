/*!
   \file robot_process.h
   \brief RobotProcess class implements ROS node lifecycle
   \author Maciej Marcin ZURAD
   \date 01/03/2018
*/
#ifndef DRONE_PROCESS_H
#define DRONE_PROCESS_H

#include <string>
#include <thread>

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <robot_process_msgs/State.h>
#include <robot_process_msgs/Error.h>

namespace robot_process {

class RobotProcess
{
public:

  RobotProcess();
  ~RobotProcess();

  void run(bool autostart=false);

protected:

  ros::NodeHandle node_handle_;

  virtual void onCreate() { };
  virtual void onConfigure() { };

  virtual void onStart() { };
  virtual void onStop() { };

  virtual void onPause() { };
  virtual void onResume() { };

private:

  std::thread heartbeat_thread_;

  ros::ServiceServer terminate_service_server_;
  ros::ServiceServer reconfigure_service_server_;
  ros::ServiceServer restart_service_server_;
  ros::ServiceServer start_service_server_;
  ros::ServiceServer stop_service_server_;
  ros::ServiceServer resume_service_server_;
  ros::ServiceServer pause_service_server_;

  ros::Publisher process_state_pub_;
  ros::Publisher process_error_pub_;

  std::string node_name_;

  void create();
  void configure();

  void start();
  void stop();

  void resume();
  void pause();

};

} // namespace robot_process

#endif
