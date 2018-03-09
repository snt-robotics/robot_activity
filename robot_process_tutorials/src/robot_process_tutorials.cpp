#include "robot_process_tutorials/robot_process_tutorials.h"

namespace robot_process_tutorials {

  void RobotProcessTutorials::timerCallback()
  {
    ROS_INFO("Non-stoppable %d", counter++);
  }


  void RobotProcessTutorials::onCreate()
  {
    ROS_INFO("onCreate");

    // robot_process::IsolatedAsyncTimer::LambdaCallback cb =
    //   [this]() mutable { ROS_INFO("Stoppable %d", counter++); };
    // registerIsolatedTimer(cb, 4.0, true);

    //registerIsolatedTimer(&RobotProcessTutorials::timerCallback, 4.0, false);
    registerIsolatedTimer(std::bind(&RobotProcessTutorials::timerCallback, this), 4.0, false);

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
