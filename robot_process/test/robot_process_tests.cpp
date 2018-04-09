#include <gtest/gtest.h>

#include <iostream>

#include <ros/ros.h>
#include <robot_process/robot_process.h>

using namespace robot_process;

class EmptyRobotProcess : public RobotProcess {
public:
  using RobotProcess::RobotProcess;
  ~EmptyRobotProcess() {};
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

TEST(RobotProcessTests, UninitializedStateAndNamespace)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remmaped_name";

  EmptyRobotProcess test(argc, (char**)argv);
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  EXPECT_EQ(test.getNamespace(), std::string(""));
}

TEST(RobotProcessTests, InitializedStateAndNamespace)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remmaped_name";

  EmptyRobotProcess test(argc, (char**)argv);
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  EXPECT_EQ(test.getNamespace(), std::string(""));


  // test.init();
  // EXPECT_EQ(test.getState(), State::STOPPED);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}