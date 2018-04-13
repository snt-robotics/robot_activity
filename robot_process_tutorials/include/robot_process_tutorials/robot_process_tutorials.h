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
#ifndef ROBOT_PROCESS_TUTORIALS_ROBOT_PROCESS_TUTORIALS_H
#define ROBOT_PROCESS_TUTORIALS_ROBOT_PROCESS_TUTORIALS_H

#include <ros/ros.h>
#include <robot_process/managed_robot_process.h>

#include <robot_process_msgs/State.h>
#include <std_srvs/Empty.h>

namespace robot_process_tutorials
{

class RobotProcessTutorials : public robot_process::ManagedRobotProcess
{
public:
  using ManagedRobotProcess::ManagedRobotProcess;
  ~RobotProcessTutorials() { }

  void onManagedCreate() override;
  void onManagedTerminate() override;

  void onManagedConfigure() override;
  void onManagedUnconfigure() override;

  void onManagedStart() override;
  void onManagedStop() override;

  void onManagedResume() override;
  void onManagedPause() override;

private:
  void timerCallback();
  void heartbeatCallback(boost::shared_ptr<robot_process_msgs::State const> msg);

  bool serviceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  int counter = 0;
};

}  // namespace robot_process_tutorials

#endif  // ROBOT_PROCESS_TUTORIALS_ROBOT_PROCESS_TUTORIALS_H
