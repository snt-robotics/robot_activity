#ifndef ROBOT_PROCESS_TUTORIALS_H
#define ROBOT_PROCESS_TUTORIALS_H

#include <ros/ros.h>
#include <robot_process/robot_process.h>
#include <robot_process/managed_robot_process.h>

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

  void onManagedPause() override;
  void onManagedResume() override;

private:

  void timerCallback();

  int counter = 0;
};

} // namespace robot_process_tutorials

#endif
