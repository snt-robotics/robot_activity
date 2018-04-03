#ifndef ROBOT_PROCESS_TUTORIALS_MINIMAL_H
#define ROBOT_PROCESS_TUTORIALS_MINIMAL_H

#include <ros/ros.h>
#include <robot_process/managed_robot_process.h>

#include <robot_process_msgs/State.h>
#include <std_srvs/Empty.h>

namespace robot_process_tutorials {

class MyRobotProcess : public robot_process::ManagedRobotProcess
{
public:
  /* Important to inherit the constructor */
  using ManagedRobotProcess::ManagedRobotProcess;
  ~MyRobotProcess()
  { 
    ROS_DEBUG_STREAM(getNamespace() << " destructed!");
  }

private:
  void onManagedCreate() override
  {
    subscriber_manager.subscribe("/heartbeat", 1, 
      &MyRobotProcess::mySubscriberCallback, this);

    service_manager.advertiseService("test", 
      &MyRobotProcess::myServiceCallback, this);

    registerIsolatedTimer(std::bind(&MyRobotProcess::myTimerCallback, this),
      0.5, true);
  };
  void onManagedTerminate() override {};

  void onManagedConfigure() override {};
  void onManagedUnconfigure() override {};

  void onManagedStart() override {};
  void onManagedStop() override {};

  void onManagedResume() override {};
  void onManagedPause() override {};

  void myTimerCallback()
  {
    ROS_INFO_STREAM(getNamespace() << " Timer Counter: " << counter);
    counter++;
    float r2 = rand() / (RAND_MAX/0.10);
    ros::Duration(2.05 - r2).sleep();
  };

  void mySubscriberCallback(boost::shared_ptr<robot_process_msgs::State const> msg)
  {
    ROS_INFO_STREAM(getNamespace() << " " 
      << msg->node_name << " is in " << unsigned(msg->state));
  };

  bool myServiceCallback(
    std_srvs::Empty::Request& request, 
    std_srvs::Empty::Response& response)
  {
    ROS_INFO_STREAM(getNamespace() << " Service called, returning true");
    return true;
  };

  int counter = 0;

};

} // namespace robot_process_tutorials


int main(int argc, char *argv[]) {

  using namespace robot_process_tutorials;

  MyRobotProcess node1(argc, argv, "first");
  node1.init().runAsync();

  MyRobotProcess node2(argc, argv, "second");
  node2.init().runAsync();

  ros::waitForShutdown();
  return 0;
}

#endif
