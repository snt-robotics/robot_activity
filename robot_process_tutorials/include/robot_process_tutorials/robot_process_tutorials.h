#ifndef ROBOT_PROCESS_TUTORIALS_H
#define ROBOT_PROCESS_TUTORIALS_H

#include <ros/ros.h>
#include <robot_process/robot_process.h>

namespace robot_process_tutorials {

class RobotProcessTutorials : public robot_process::RobotProcess
{
public:

  using RobotProcess::RobotProcess;

  void onCreate();
  void onConfigure();

  void onStart();
  void onStop();

  void onPause();
  void onResume();

};

} // namespace robot_process_tutorials

#endif
