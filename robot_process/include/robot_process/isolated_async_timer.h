/*!
   \file isolated_async_timer.h
   \brief IsolatedAsyncTimer class implements ROS Timer served by
   a single-threaded async spinner on a separate callback queue
   \author Maciej Marcin ZURAD
   \date 01/03/2018
*/
#ifndef ISOLATED_ASYNC_TIMER_H
#define ISOLATED_ASYNC_TIMER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>

namespace robot_process {

typedef boost::function<void(void)> LambdaCallback;

ros::TimerCallback convert_to_timer_callback(const LambdaCallback& callback);

class IsolatedAsyncTimer
{
public:

  IsolatedAsyncTimer() = delete;

  IsolatedAsyncTimer(const ros::NodeHandle& node_handle,
                     const LambdaCallback& callback,
                     const float& frequency,
                     bool oneshot = false,
                     bool autostart = true)
    : IsolatedAsyncTimer(node_handle,
                         convert_to_timer_callback(callback),
                         frequency,
                         oneshot,
                         autostart) { }

  IsolatedAsyncTimer(const ros::NodeHandle& node_handle,
                     const ros::TimerCallback& callback,
                     const float& frequency,
                     bool oneshot = false,
                     bool autostart = true)
    : node_handle_(node_handle),
      timer_ops_(),
      callback_(callback),
      callback_queue_()
  {
    timer_ops_.period = ros::Duration(1.0 / frequency);
    timer_ops_.callback = callback_;
    timer_ops_.callback_queue = &callback_queue_;
    timer_ops_.oneshot = oneshot;
    timer_ops_.autostart = autostart;

    timer_ = std::make_shared<ros::Timer>();
    *timer_ = node_handle_.createTimer(timer_ops_);

    spinner_ = std::make_shared<ros::AsyncSpinner>(1, &callback_queue_);
    spinner_->start();
  }

  ~IsolatedAsyncTimer() { }

  void start() { timer_->start(); }
  void stop() { timer_->stop(); }
  bool isValid() { return timer_->isValid(); }
  void setPeriod(const ros::Duration& period, bool reset = true)
  {
    timer_->setPeriod(period, reset);
  }

private:

  ros::NodeHandle node_handle_;

  ros::TimerOptions timer_ops_;
  ros::TimerCallback callback_;
  ros::CallbackQueue callback_queue_;

  std::shared_ptr<ros::Timer> timer_;
  std::shared_ptr<ros::AsyncSpinner> spinner_;

};

ros::TimerCallback convert_to_timer_callback(const LambdaCallback& callback)
{
  return [callback](const ros::TimerEvent& e) { callback(); };
}

} // namespace robot_process

#endif
