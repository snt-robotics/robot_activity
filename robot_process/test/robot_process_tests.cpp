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

class AnyRobotProcessWithTimer : public AnyRobotProcess
{
public:
  using AnyRobotProcess::AnyRobotProcess;
  int context = 0;
  bool stoppable = true;
private:
  void onCreate() override
  {
    IsolatedAsyncTimer::LambdaCallback cb = [this]() { context++; };
    registerIsolatedTimer(cb, 1, stoppable);
  }
};

TEST(RobotProcessTests, RemappedNameInitializedStateAndNamespace)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  AnyRobotProcess test(argc, (char**)argv);
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  EXPECT_EQ(test.getNamespace(), std::string(""));

  boost::function<void(const robot_process_msgs::State::ConstPtr&)> empty_cb =
    [](const robot_process_msgs::State::ConstPtr &s){};

  ros::NodeHandle nh;
  auto sub = nh.subscribe("/heartbeat", 1, empty_cb);

  test.init();
  EXPECT_EQ(test.getState(), State::STOPPED);
  EXPECT_EQ(test.getNamespace(), std::string("/remapped_name"));
}

TEST(RobotProcessTests, InitializedNonWaitingStateAndNamespace)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);

  AnyRobotProcess test(argc, (char**)argv);
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  EXPECT_EQ(test.getNamespace(), std::string(""));
  test.init();
  EXPECT_EQ(test.getState(), State::STOPPED);
  EXPECT_EQ(test.getNamespace(), std::string("/remapped_name"));
}

TEST(RobotProcessTests, AutostartNonWaitingStateAndNamespace)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  AnyRobotProcess test(argc, (char**)argv);
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  EXPECT_EQ(test.getNamespace(), std::string(""));
  test.init();
  EXPECT_EQ(test.getState(), State::RUNNING);
  EXPECT_EQ(test.getNamespace(), std::string("/remapped_name"));
}

TEST(RobotProcessTests, StartStopServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", false);

  AnyRobotProcess test(argc, (char**)argv);
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::STOPPED);

  auto start = nh.serviceClient<std_srvs::Empty>("/remapped_name/start");
  std_srvs::Empty start_empty;
  EXPECT_EQ(start.call(start_empty), true);
  EXPECT_EQ(test.getState(), State::RUNNING);

  auto stop = nh.serviceClient<std_srvs::Empty>("/remapped_name/stop");
  std_srvs::Empty stop_empty;
  EXPECT_EQ(stop.call(stop_empty), true);
  EXPECT_EQ(test.getState(), State::STOPPED);
}

TEST(RobotProcessTests, PauseResumeServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  AnyRobotProcess test(argc, (char**)argv);
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::RUNNING);

  auto pause = nh.serviceClient<std_srvs::Empty>("/remapped_name/pause");
  std_srvs::Empty pause_empty;
  EXPECT_EQ(pause.call(pause_empty), true);
  EXPECT_EQ(test.getState(), State::PAUSED);

  auto start = nh.serviceClient<std_srvs::Empty>("/remapped_name/start");
  std_srvs::Empty start_empty;
  EXPECT_EQ(start.call(start_empty), true);
  EXPECT_EQ(test.getState(), State::RUNNING);
}

TEST(RobotProcessTests, RestartServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  AnyRobotProcess test(argc, (char**)argv);
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::RUNNING);


  int counter = 0;
  std::vector<State> expected_transitions = {
    State::RUNNING,
    State::PAUSED,
    State::STOPPED,
    State::PAUSED,
    State::RUNNING
  };

  boost::function<void(const robot_process_msgs::State::ConstPtr&)> check_transitions_cb =
  [&](const robot_process_msgs::State::ConstPtr &s)
  {
    EXPECT_EQ(s->state, static_cast<uint8_t>(expected_transitions[counter++]));
  };

  auto sub = nh.subscribe("/heartbeat", 0, check_transitions_cb);
  auto restart = nh.serviceClient<std_srvs::Empty>("/remapped_name/restart");
  std_srvs::Empty restart_empty;
  EXPECT_EQ(restart.call(restart_empty), true);
  EXPECT_EQ(test.getState(), State::RUNNING);
}

TEST(RobotProcessTests, ReconfigureServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/autostart_after_reconfigure", true);
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  AnyRobotProcess test(argc, (char**)argv);
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::RUNNING);

  int counter = 0;
  std::vector<State> expected_transitions = {
    State::RUNNING,
    State::PAUSED,
    State::STOPPED,
    State::UNCONFIGURED,
    State::STOPPED,
    State::PAUSED,
    State::RUNNING
  };

  boost::function<void(const robot_process_msgs::State::ConstPtr&)> check_transitions_cb =
  [&](const robot_process_msgs::State::ConstPtr &s)
  {
    EXPECT_EQ(s->state, static_cast<uint8_t>(expected_transitions[counter++]));
  };

  auto sub = nh.subscribe("/heartbeat", 0, check_transitions_cb);
  auto reconfigure = nh.serviceClient<std_srvs::Empty>("/remapped_name/reconfigure");
  std_srvs::Empty reconfigure_empty;
  EXPECT_EQ(reconfigure.call(reconfigure_empty), true);
  EXPECT_EQ(test.getState(), State::RUNNING);
}

TEST(RobotProcessTests, NoAutostartAfterReconfigureServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/autostart_after_reconfigure", false);
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  AnyRobotProcess test(argc, (char**)argv);
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::RUNNING);

  auto reconfigure = nh.serviceClient<std_srvs::Empty>("/remapped_name/reconfigure");
  std_srvs::Empty reconfigure_empty;
  EXPECT_EQ(reconfigure.call(reconfigure_empty), true);
  EXPECT_EQ(test.getState(), State::STOPPED);
}

TEST(RobotProcessTests, TerminateServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", false);

  AnyRobotProcess test(argc, (char**)argv);
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::STOPPED);

  auto terminate = nh.serviceClient<std_srvs::Empty>("/remapped_name/terminate");
  std_srvs::Empty terminate_empty;
  EXPECT_EQ(terminate.call(terminate_empty), true);
  EXPECT_EQ(test.getState(), State::TERMINATED);
}

TEST(RobotProcessTests, IsolatedAsyncTimer)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  AnyRobotProcessWithTimer test(argc, (char**)argv);
  test.init().runAsync();
  ros::Duration(2.1).sleep();
  EXPECT_EQ(test.context, 2);
}


TEST(RobotProcessTests, StoppableIsolatedAsyncTimer)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  auto start = nh.serviceClient<std_srvs::Empty>("/remapped_name/start");
  auto pause = nh.serviceClient<std_srvs::Empty>("/remapped_name/pause");
  std_srvs::Empty pause_empty, start_empty;

  AnyRobotProcessWithTimer test(argc, (char**)argv);
  test.init().runAsync();

  ros::Duration(1.1).sleep();
  EXPECT_EQ(test.context, 1);
  EXPECT_EQ(pause.call(pause_empty), true);
  ros::Duration(1.0).sleep();
  EXPECT_EQ(test.context, 1);
  EXPECT_EQ(start.call(start_empty), true);
  ros::Duration(1.1).sleep();
  EXPECT_EQ(test.context, 2);

}

TEST(RobotProcessTests, NonStoppableIsolatedAsyncTimer)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  auto start = nh.serviceClient<std_srvs::Empty>("/remapped_name/start");
  auto stop = nh.serviceClient<std_srvs::Empty>("/remapped_name/stop");
  std_srvs::Empty stop_empty, start_empty;

  AnyRobotProcessWithTimer test(argc, (char**)argv);
  test.stoppable = false;
  test.init().runAsync();

  ros::Duration(1.1).sleep();
  EXPECT_EQ(test.context, 1);
  EXPECT_EQ(stop.call(stop_empty), true);
  ros::Duration(1.1).sleep();
  EXPECT_EQ(test.context, 2);
  EXPECT_EQ(start.call(start_empty), true);
  ros::Duration(1.1).sleep();
  EXPECT_EQ(test.context, 3);

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
