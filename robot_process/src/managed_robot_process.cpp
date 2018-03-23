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
  subscription_manager.subscribeAll(node_handle_private_);
  onManagedStart();
}

void ManagedRobotProcess::onStop()
{
  ROS_INFO("onStop");
  subscription_manager.unsubscribeAll();
  onManagedStop();
}

void ManagedRobotProcess::onPause()
{
  ROS_INFO("onPause");
  subscription_manager.pauseAll();
  onManagedPause();
}

void ManagedRobotProcess::onResume()
{
  ROS_INFO("onResume");
  subscription_manager.resumeAll();
  onManagedResume();
}

} // namespace robot_process
