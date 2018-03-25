#include <robot_process/managed_robot_process.h>

namespace robot_process {

void ManagedRobotProcess::onCreate()
{
  ROS_DEBUG("onCreate");
  onManagedCreate();
}

void ManagedRobotProcess::onTerminate()
{
  ROS_DEBUG("onTerminate");
  onManagedTerminate();
}

void ManagedRobotProcess::onConfigure()
{
  ROS_DEBUG("onConfigure");
  onManagedConfigure();
}

void ManagedRobotProcess::onUnconfigure()
{
  ROS_DEBUG("onUnconfigure");
  onManagedUnconfigure();
}

void ManagedRobotProcess::onStart()
{
  ROS_DEBUG("onStart");
  service_manager.acquireAll(node_handle_private_);
  subscriber_manager.acquireAll(node_handle_private_);
  onManagedStart();
}

void ManagedRobotProcess::onStop()
{
  ROS_DEBUG("onStop");
  service_manager.releaseAll();
  subscriber_manager.releaseAll();
  onManagedStop();
}

void ManagedRobotProcess::onPause()
{
  ROS_DEBUG("onPause");
  service_manager.pauseAll();
  subscriber_manager.pauseAll();
  onManagedPause();
}

void ManagedRobotProcess::onResume()
{
  ROS_DEBUG("onResume");
  service_manager.resumeAll();
  subscriber_manager.resumeAll();
  onManagedResume();
}

} // namespace robot_process
