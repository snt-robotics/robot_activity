#ifndef MANAGED_RESOURCE_H
#define MANAGED_RESOURCE_H

#include <atomic>

#include <ros/ros.h>
#include <ros/console.h>

namespace robot_process { 
namespace resource {

class ManagedSubscriber;
class ManagedServiceServer;

template <class Specialization, class Resource>
class Managed
{
public:

  Managed() = delete;
  ~Managed();

  template<typename... Args>
  Managed(Args&& ...args)
  : acquired_(false), paused_(true), resource_(), lazy_acquirer_()
  {
    ROS_DEBUG("Managed::ctor");
    lazy_acquirer_ = makeLazyAcquirer(std::forward<Args>(args)...);
  }

  void acquire(const ros::NodeHandlePtr& node_handle);
  void release();

  void pause();
  void resume();

  typedef std::shared_ptr<Managed<Specialization, Resource>> SharedPtr;

protected:

  template <class Message>
  using Callback = boost::function<void(Message)>;

  template<class Message>
  Callback<Message> wrapCallback(const Callback<Message>& callback) const
  {
    return [this, &callback](Message message) {
      ROS_DEBUG("wrapped callback executed!");
      if (!paused_)
        callback(message);
      else
        ROS_DEBUG("callback is paused!");
    };
  }

  typedef std::function<Resource(const ros::NodeHandlePtr&)> LazyAcquirer;

  template<typename... Args>
  LazyAcquirer makeLazyAcquirer(Args&& ...args)
  {
    return static_cast<Specialization*>(this)
      ->makeLazyAcquirer(std::forward<Args>(args)...);
  }

private:

  std::atomic<bool> acquired_;
  std::atomic<bool> paused_;

  Resource resource_;
  LazyAcquirer lazy_acquirer_;

};

} // namespace resource
} // namespace robot_process

#endif
