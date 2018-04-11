#ifndef ROBOT_PROCESS_TUTORIALS_H
#define ROBOT_PROCESS_TUTORIALS_H

#include <ros/ros.h>
#include <robot_process/managed_robot_process.h>

#include <robot_process_msgs/State.h>
#include <std_srvs/Empty.h>


namespace robot_process_tutorials
{

class RobotProcessTutorials : public robot_process::ManagedRobotProcess
{
public:
  using ManagedRobotProcess::ManagedRobotProcess;
  ~RobotProcessTutorials() { }

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
  void heartbeatCallback(boost::shared_ptr<robot_process_msgs::State const> msg);

  bool serviceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  int counter = 0;
};

} // namespace robot_process_tutorials

#endif
