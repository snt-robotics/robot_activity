#ifndef ROBOT_PROCESS_TUTORIALS_H
#define ROBOT_PROCESS_TUTORIALS_H

#include <ros/ros.h>
#include <robot_process/robot_process.h>
#include <robot_process/managed_robot_process.h>
#include <robot_process/subscription_manager.h>

#include <robot_process_msgs/State.h>


namespace robot_process_tutorials {

class RobotProcessTutorials : public robot_process::ManagedRobotProcess
{
private:

  using ManagedRobotProcess::ManagedRobotProcess;

  void onManagedCreate() override;
  void onManagedTerminate() override;

  void onManagedConfigure() override;
  void onManagedUnconfigure() override;

  void onManagedStart() override;
  void onManagedStop() override;

  void onManagedResume() override;
  void onManagedPause() override;

private:

  void timerCallback();
  void msgCallback(boost::shared_ptr<robot_process_msgs::State> i);

  int counter = 0;
};

} // namespace robot_process_tutorials

#endif
