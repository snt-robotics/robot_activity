#include "robot_process_tutorials/robot_process_tutorials.h"

namespace robot_process_tutorials {

  void RobotProcessTutorials::onCreate()
  {
    ROS_INFO("onCreate");
  }

  void RobotProcessTutorials::onTerminate()
  {
    ROS_INFO("onTerminate");
  };

  void RobotProcessTutorials::onConfigure()
  {
    ROS_INFO("onConfigure");
  }

  void RobotProcessTutorials::onUnconfigure()
  {
    ROS_INFO("onUnconfigure");
  }

  void RobotProcessTutorials::onStart()
  {
    ROS_INFO("onStart");
  }

  void RobotProcessTutorials::onStop()
  {
    ROS_INFO("onStop");
  }

  void RobotProcessTutorials::onPause()
  {
    ROS_INFO("onPause");
  }

  void RobotProcessTutorials::onResume()
  {
    ROS_INFO("onResume");
  }

} // namespace robot_process_tutorials
