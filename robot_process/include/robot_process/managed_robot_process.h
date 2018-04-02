/*!
   \file managed_robot_process.h
   \brief ManagedRobotProcess class implements ROS node lifecycle with
    managed subscriptions and services
   \author Maciej Marcin ZURAD
   \date 01/03/2018
*/
#ifndef MANAGED_ROBOT_PROCESS_H
#define MANAGED_ROBOT_PROCESS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <std_srvs/Empty.h>
#include <robot_process_msgs/State.h>
#include <robot_process_msgs/Error.h>

#include <robot_process/robot_process.h>
#include <robot_process/resource/resource_manager.h>

namespace robot_process {

/**
 * @brief Managed RobotProcess class, which adds further functionality to the
 *        RobotProcess class
 * @details ManagedRobotProcess manages ROS Subscribers and ServiceServers.
 *          It automatically pauses all Subscribers and ServiceServers 
 *          during PAUSED state and resumes them when transitioning to the 
 *          RUNNING state. It also shutdowns them in STOPPED state and 
 *          re-acquires them (by re-subscribing or re-advertising) when
 *          transitioning from STOPPED to PAUSED.
 */
class ManagedRobotProcess : public RobotProcess
{
public:
  /**
   * @brief Default constructor inherited from RobotProcess
   */
  using RobotProcess::RobotProcess;

  /**
   * @brief Default virtual destructor
   */
  virtual ~ManagedRobotProcess();

protected:
  /**
   * @brief Manager for subscribing to ROS topics.
   */
  resource::SubscriberManager subscriber_manager;

  /**
   * @brief Manager for advertising ROS services
   */
  resource::ServiceServerManager service_manager;

private:

  /**
   * @brief Overriden onCreate, which calls onManagedCreate. Cannot be 
   *        overriden further by the child class of ManagedRobotProcess.
   */
  void onCreate() override final;

  /**
   * @brief Overriden onTerminate, which calls onManagedTerminate. Cannot be 
   *        overriden further by the child class of ManagedRobotProcess.
   */
  void onTerminate() override final;

  /**
   * @brief Overriden onConfigure, which calls onManagedConfigure. Cannot be 
   *        overriden further by the child class of ManagedRobotProcess.
   */
  void onConfigure() override final;

  /**
   * @brief Overriden onUnconfigure, which calls onManagedUnconfigure. Cannot be 
   *        overriden further by the child class of ManagedRobotProcess.
   */
  void onUnconfigure() override final;

  /**
   * @brief Overriden onStart, which calls onManagedStart. Cannot be 
   *        overriden further by the child class of ManagedRobotProcess.
   *        It subscribes and adverties all ROS topics and services that were
   *        subscribed and advertised with subscription_manager 
   *        and service_manager before calling onManagedStart.
   */ 
  void onStart() override final;

  /**
   * @brief Overriden onStop, which calls onManagedStop. Cannot be 
   *        overriden further by the child class of ManagedRobotProcess.
   *        It shutdowns all ROS topics and services that were
   *        subscribed and advertised with subscription_manager 
   *        and service_manager before calling onManagedStart.
   */ 
  void onStop() override final;

  /**
   * @brief Overriden onPause, which calls onManagedPause. Cannot be 
   *        overriden further by the child class of ManagedRobotProcess.
   *        It pauses all ROS topics and services that were
   *        subscribed and advertised with subscription_manager 
   *        and service_manager before calling onManagedStart.
   */ 
  void onPause() override final;

  /**
   * @brief Overriden onResume, which calls onManagedResume. Cannot be 
   *        overriden further by the child class of ManagedRobotProcess.
   *        It resumes all ROS topics and services that were
   *        subscribed and advertised with subscription_manager 
   *        and service_manager before calling onManagedStart.
   */ 
  void onResume() override final;

  /**
   * @brief User-defined function that's called at the end of transition from
   *        LAUNCHING to UNCONFIGURED state
   */
  virtual void onManagedCreate() = 0;

  /**
   * @brief User-defined function that's called at the end of transition from
   *        UNCONFIGURED to TERMINATED state
   */
  virtual void onManagedTerminate() = 0;

  /**
   * @brief User-defined function that's called at the end of transition from
   *        UNCONFIGURED to STOPPED state
   */
  virtual void onManagedConfigure() = 0;

    /**
   * @brief User-defined function that's called at the end of transition from
   *        STOPPED to UNCONFIGURED state
   */
  virtual void onManagedUnconfigure() = 0;

  /**
   * @brief User-defined function that's called at the end of transition from
   *        STOPPED to PAUSED state
   */
  virtual void onManagedStart() = 0;

  /**
   * @brief User-defined function that's called at the end of transition from
   *        PAUSED to STOPPED state
   */
  virtual void onManagedStop() = 0;

  /**
   * @brief User-defined function that's called at the end of transition from
   *        RUNNING to PAUSED state
   */
  virtual void onManagedPause() = 0;

  /**
   * @brief User-defined function that's called at the end of transition from
   *        PAUSED to RUNNING state
   */
  virtual void onManagedResume() = 0;

};

} // namespace robot_process

#endif
