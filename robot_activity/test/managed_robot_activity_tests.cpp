/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, University of Luxembourg
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Luxembourg nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Maciej Zurad
 *********************************************************************/
#include <gtest/gtest.h>

#include <ros/ros.h>
#include <robot_activity/managed_robot_activity.h>
#include <robot_activity_msgs/State.h>

#include <std_srvs/Empty.h>
#include <std_msgs/Int32.h>

#include <vector>
#include <string>

using robot_activity::ManagedRobotActivity;
using robot_activity::State;

class AnyManagedRobotActivity : public ManagedRobotActivity
{
public:
  using ManagedRobotActivity::ManagedRobotActivity;
  ~AnyManagedRobotActivity() {};
private:
  void onManagedCreate() override {};
  void onManagedTerminate() override {};

  void onManagedConfigure() override {};
  void onManagedUnconfigure() override {};

  void onManagedStart() override {};
  void onManagedStop() override {};

  void onManagedResume() override {};
  void onManagedPause() override {};
};

class AnyManagedRobotActivityWithSubscriber : public AnyManagedRobotActivity
{
public:
  using AnyManagedRobotActivity::AnyManagedRobotActivity;
  int32_t data = 0;
private:
  void onManagedCreate() override
  {
    subscriber_manager.subscribe("test", 1,
      &AnyManagedRobotActivityWithSubscriber::callback, this);
  }
  void callback(boost::shared_ptr<std_msgs::Int32 const> msg)
  {
    data = msg->data;
  };
};


TEST(ManagedRobotActivityTests, AutostartNonWaitingStateAndNamespace)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  AnyManagedRobotActivity test(argc, const_cast<char**>(argv));

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  EXPECT_EQ(test.getState(), State::LAUNCHING);
  EXPECT_EQ(test.getNamespace(), std::string(""));
  test.init();
  EXPECT_EQ(test.getState(), State::RUNNING);
  EXPECT_EQ(test.getNamespace(), std::string("/remapped_name"));
}

TEST(ManagedRobotActivityTests, StartStopServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  AnyManagedRobotActivity test(argc, const_cast<char**>(argv));

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", false);

  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::STOPPED);

  auto start = nh.serviceClient<std_srvs::Empty>("/remapped_name/robot_activity/start");
  std_srvs::Empty start_empty;
  EXPECT_EQ(start.call(start_empty), true);
  EXPECT_EQ(test.getState(), State::RUNNING);

  auto stop = nh.serviceClient<std_srvs::Empty>("/remapped_name/robot_activity/stop");
  std_srvs::Empty stop_empty;
  EXPECT_EQ(stop.call(stop_empty), true);
  EXPECT_EQ(test.getState(), State::STOPPED);
}

TEST(ManagedRobotActivityTests, PauseResumeServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  AnyManagedRobotActivity test(argc, const_cast<char**>(argv));

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::RUNNING);

  auto pause = nh.serviceClient<std_srvs::Empty>("/remapped_name/robot_activity/pause");
  std_srvs::Empty pause_empty;
  EXPECT_EQ(pause.call(pause_empty), true);
  EXPECT_EQ(test.getState(), State::PAUSED);

  auto start = nh.serviceClient<std_srvs::Empty>("/remapped_name/robot_activity/start");
  std_srvs::Empty start_empty;
  EXPECT_EQ(start.call(start_empty), true);
  EXPECT_EQ(test.getState(), State::RUNNING);
}

TEST(ManagedRobotActivityTests, RestartServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  AnyManagedRobotActivity test(argc, const_cast<char**>(argv));

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::RUNNING);

  int counter = 0;
  std::vector<State> expected_transitions
  {
    State::RUNNING,
    State::PAUSED,
    State::STOPPED,
    State::PAUSED,
    State::RUNNING
  };

  boost::function<void(const robot_activity_msgs::State::ConstPtr&)> check_transitions_cb =
  [&](const robot_activity_msgs::State::ConstPtr &s)
  {
    EXPECT_EQ(s->state, static_cast<uint8_t>(expected_transitions[counter++]));
  };

  auto sub = nh.subscribe("/heartbeat", 0, check_transitions_cb);
  auto restart = nh.serviceClient<std_srvs::Empty>("/remapped_name/robot_activity/restart");
  std_srvs::Empty restart_empty;
  EXPECT_EQ(restart.call(restart_empty), true);
  EXPECT_EQ(test.getState(), State::RUNNING);
}

TEST(ManagedRobotActivityTests, ReconfigureServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  AnyManagedRobotActivity test(argc, const_cast<char**>(argv));

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/autostart_after_reconfigure", true);
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::RUNNING);

  int counter = 0;
  std::vector<State> expected_transitions =
  {
    State::RUNNING,
    State::PAUSED,
    State::STOPPED,
    State::UNCONFIGURED,
    State::STOPPED,
    State::PAUSED,
    State::RUNNING
  };

  boost::function<void(const robot_activity_msgs::State::ConstPtr&)> check_transitions_cb =
  [&](const robot_activity_msgs::State::ConstPtr &s)
  {
    EXPECT_EQ(s->state, static_cast<uint8_t>(expected_transitions[counter++]));
  };

  auto sub = nh.subscribe("/heartbeat", 0, check_transitions_cb);
  auto reconfigure = nh.serviceClient<std_srvs::Empty>("/remapped_name/robot_activity/reconfigure");
  std_srvs::Empty reconfigure_empty;
  EXPECT_EQ(reconfigure.call(reconfigure_empty), true);
  EXPECT_EQ(test.getState(), State::RUNNING);
}

TEST(ManagedRobotActivityTests, NoAutostartAfterReconfigureServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  AnyManagedRobotActivity test(argc, const_cast<char**>(argv));

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/autostart_after_reconfigure", false);
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::RUNNING);

  auto reconfigure = nh.serviceClient<std_srvs::Empty>("/remapped_name/robot_activity/reconfigure");
  std_srvs::Empty reconfigure_empty;
  EXPECT_EQ(reconfigure.call(reconfigure_empty), true);
  EXPECT_EQ(test.getState(), State::STOPPED);
}

TEST(ManagedRobotActivityTests, TerminateServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  AnyManagedRobotActivity test(argc, const_cast<char**>(argv));

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", false);

  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::STOPPED);

  auto terminate = nh.serviceClient<std_srvs::Empty>("/remapped_name/robot_activity/terminate");
  std_srvs::Empty terminate_empty;
  EXPECT_EQ(terminate.call(terminate_empty), true);
  EXPECT_EQ(test.getState(), State::TERMINATED);
}

TEST(ManagedRobotActivityTests, ManagedSubscriberCheck)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  AnyManagedRobotActivityWithSubscriber test(argc, const_cast<char**>(argv));

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", false);

  auto publisher = nh.advertise<std_msgs::Int32>("test", 1);
  EXPECT_EQ(publisher.getNumSubscribers(), 0);

  test.init().runAsync();
  EXPECT_EQ(test.getState(), State::STOPPED);
  EXPECT_EQ(publisher.getNumSubscribers(), 0);

  auto start = nh.serviceClient<std_srvs::Empty>("/remapped_name/robot_activity/start");
  std_srvs::Empty start_empty;
  EXPECT_EQ(start.call(start_empty), true);
  EXPECT_EQ(test.getState(), State::RUNNING);
  EXPECT_EQ(publisher.getNumSubscribers(), 1);

  int32_t data_to_send = 42;
  std_msgs::Int32 msg;
  msg.data = data_to_send;
  publisher.publish(msg);
  ros::spinOnce();
  sleep(4);
  EXPECT_EQ(test.data, data_to_send);

  auto pause = nh.serviceClient<std_srvs::Empty>("/remapped_name/robot_activity/pause");
  std_srvs::Empty pause_empty;
  EXPECT_EQ(pause.call(pause_empty), true);
  EXPECT_EQ(test.getState(), State::PAUSED);
  EXPECT_EQ(publisher.getNumSubscribers(), 1);
}



int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
