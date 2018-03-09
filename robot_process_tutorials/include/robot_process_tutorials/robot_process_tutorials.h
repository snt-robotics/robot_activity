#ifndef ROBOT_PROCESS_TUTORIALS_H
#define ROBOT_PROCESS_TUTORIALS_H

#include <ros/ros.h>
#include <robot_process/robot_process.h>

namespace robot_process_tutorials {

class RobotProcessTutorials : public robot_process::RobotProcess
{

private:

  using RobotProcess::RobotProcess;

  void onCreate() override;
  void onTerminate() override;

  void onConfigure() override;
  void onUnconfigure() override;

  void onStart() override;
  void onStop() override;

  void onPause() override;
  void onResume() override;

  void timerCallback();

  int counter = 0;

};

} // namespace robot_process_tutorials

#endif
