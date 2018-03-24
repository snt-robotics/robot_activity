/*!
   \file isolated_async_timer.h
   \brief IsolatedAsyncTimer class implements ROS Timer served by
   a single-threaded async spinner on a separate callback queue
   \author Maciej Marcin ZURAD
   \date 01/03/2018
*/
#ifndef ISOLATED_ASYNC_TIMER_H
#define ISOLATED_ASYNC_TIMER_H

#include <atomic>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

namespace robot_process {

class IsolatedAsyncTimer
{
public:

  typedef boost::function<void(void)> LambdaCallback;

  IsolatedAsyncTimer() = delete;

  IsolatedAsyncTimer(const ros::NodeHandle& node_handle,
                     const IsolatedAsyncTimer::LambdaCallback& callback,
                     const float& frequency,
                     bool stoppable = true,
                     bool autostart = true,
                     bool oneshot = false)
    : IsolatedAsyncTimer(node_handle,
                         to_timer_callback(callback),
                         frequency,
                         stoppable,
                         autostart,
                         oneshot) { }

  IsolatedAsyncTimer(const ros::NodeHandle& node_handle,
                     const ros::TimerCallback& callback,
                     const float& frequency,
                     bool stoppable = true,
                     bool autostart = true,
                     bool oneshot = false)
    : node_handle_(node_handle),
      timer_ops_(),
      callback_queue_(),
      stoppable_(stoppable),
      paused_(true)
  {
    callback_ = wrap_callback(callback);

    frequency_ = frequency;
    period_ = ros::Duration(1.0 / frequency);

    timer_ops_.period = period_;
    timer_ops_.callback = callback_;
    timer_ops_.callback_queue = &callback_queue_;
    timer_ops_.oneshot = oneshot;
    timer_ops_.autostart = autostart;

    timer_ = std::make_shared<ros::Timer>();
    *timer_ = node_handle_.createTimer(timer_ops_);

    spinner_ = std::make_shared<ros::AsyncSpinner>(1, &callback_queue_);
    spinner_->start();
  }

  ~IsolatedAsyncTimer() { ROS_DEBUG("IsolatedAsyncTimer destructor"); }

  void start() { timer_->start(); }
  void stop()
  {
    if (stoppable_)
      timer_->stop();
  }
  void pause() { paused_ = true; }
  void resume() { paused_ = false; }

  bool isValid() { return timer_->isValid(); }
  void setRate(const float& frequency, bool reset = true)
  {
    frequency_ = frequency;
    period_ = ros::Duration(1.0 / frequency);
    timer_->setPeriod(period_, reset);
  }

  static ros::TimerCallback to_timer_callback(
    const IsolatedAsyncTimer::LambdaCallback& callback)
  {
    return [callback](const ros::TimerEvent& event) { callback(); };
  }

  ros::TimerCallback wrap_callback(const ros::TimerCallback& callback)
  {
    return [=](const ros::TimerEvent& ev)
    {
      if (stoppable_ == false || paused_ == false)
      {
        auto last_loop_duration = ev.profile.last_duration.toSec();
        if ( ev.last_real.toSec() != 0 && last_loop_duration > period_.toSec())
        {
          auto lag = last_loop_duration - period_.toSec();
          ROS_WARN_STREAM(
            "Missed it's desired rate of " << frequency_ <<
            " [Hz], the loop actually took " << last_loop_duration
            << " [s], which is " << lag << " [s] longer");
        }
        callback(ev);
      }
    };
  }

private:

  ros::NodeHandle node_handle_;

  float frequency_;
  ros::Duration period_;
  ros::TimerOptions timer_ops_;
  ros::TimerCallback callback_;
  ros::CallbackQueue callback_queue_;

  std::shared_ptr<ros::Timer> timer_;
  std::shared_ptr<ros::AsyncSpinner> spinner_;

  std::atomic<bool> stoppable_;
  std::atomic<bool> paused_;

};

} // namespace robot_process

#endif
