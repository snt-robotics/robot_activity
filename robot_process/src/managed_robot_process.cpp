#include <robot_process/managed_robot_process.h>

namespace robot_process {

  void ManagedRobotProcess::onCreate()
  {
    ROS_INFO("onCreate");
    onManagedCreate();
  }

  void ManagedRobotProcess::onTerminate()
  {
    ROS_INFO("onTerminate");
    onManagedTerminate();
  }

  void ManagedRobotProcess::onConfigure()
  {
    ROS_INFO("onConfigure");
    onManagedConfigure();
  }
  void ManagedRobotProcess::onUnconfigure()
  {
    ROS_INFO("onUnconfigure");
    onManagedUnconfigure();
  }

  void ManagedRobotProcess::onStart()
  {
    ROS_INFO("onStart");
    onManagedStart();
  }
  void ManagedRobotProcess::onStop()
  {
    ROS_INFO("onStop");
    onManagedStop();
  }

  void ManagedRobotProcess::onPause()
  {
    ROS_INFO("onPause");
    onManagedPause();
  }
  void ManagedRobotProcess::onResume()
  {
    ROS_INFO("onResume");
    onManagedResume();
  }

} // namespace robot_process
