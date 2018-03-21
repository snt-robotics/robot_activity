/*!
   \file managed_robot_process.h
   \brief ManagedRobotProcess class implements ROS node lifecycle with
    managed subscriptions and services
   \author Maciej Marcin ZURAD
   \date 01/03/2018
*/
#ifndef MANAGED_ROBOT_PROCESS_H
#define MANAGED_ROBOT_PROCESS_H

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <std_srvs/Empty.h>
#include <robot_process_msgs/State.h>
#include <robot_process_msgs/Error.h>

#include <robot_process/robot_process.h>

namespace robot_process {

class ManagedRobotProcess : public RobotProcess
{
public:

  using RobotProcess::RobotProcess;

private:

  void onCreate() override final;
  void onTerminate() override final;

  void onConfigure() override final;
  void onUnconfigure() override final;

  void onStart() override final;
  void onStop() override final;

  void onPause() override final;
  void onResume() override final;

  virtual void onManagedCreate() = 0;
  virtual void onManagedTerminate() = 0;

  virtual void onManagedConfigure() = 0;
  virtual void onManagedUnconfigure() = 0;

  virtual void onManagedStart() = 0;
  virtual void onManagedStop() = 0;

  virtual void onManagedPause() = 0;
  virtual void onManagedResume() = 0;

};

} // namespace robot_process

#endif
