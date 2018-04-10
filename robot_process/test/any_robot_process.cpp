#include <gtest/gtest.h>

#include <ros/ros.h>
#include <robot_process/robot_process.h>
#include <robot_process_msgs/State.h>

#include <std_srvs/Empty.h>

using namespace robot_process;

class AnyRobotProcess : public RobotProcess {
public:
  using RobotProcess::RobotProcess;
  ~AnyRobotProcess() {};
private:
  void onCreate() override {};
  void onTerminate() override {};

  void onConfigure() override {};
  void onUnconfigure() override {};

  void onStart() override {};
  void onStop() override {};

  void onResume() override {};
  void onPause() override {};
};

int main(int argc, char **argv){
  AnyRobotProcess erb(argc, argv);
  erb.init().run();
  return 0;
}
