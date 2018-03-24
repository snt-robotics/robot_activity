#ifndef MANAGED_RESOURCE_H
#define MANAGED_RESOURCE_H

#include <atomic>

#include <ros/ros.h>
#include <ros/console.h>

namespace robot_process {

template <class Specialization, class Resource>
class Managed
{
public:

  Managed() = delete;
  ~Managed();

  template<typename... Args>
  Managed(Args&& ...args)
  : acquired_(false), paused_(true), resource_(), lazy_acquire()
  {
    ROS_DEBUG("Managed::ctor");
    lazy_acquire = makeLazyAcquire(std::forward<Args>(args)...);
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

  typedef std::function<Resource(const ros::NodeHandlePtr&)> LazyAcquire;

  template<typename... Args>
  LazyAcquire makeLazyAcquire(Args&& ...args)
  {
    return static_cast<Specialization*>(this)
      ->makeLazyAcquire(std::forward<Args>(args)...);
  }

private:

  std::atomic<bool> acquired_;
  std::atomic<bool> paused_;

  Resource resource_;
  LazyAcquire lazy_acquire;

};

class ManagedSubscriber : public Managed<ManagedSubscriber, ros::Subscriber>
{
public:
  using Managed<ManagedSubscriber, ros::Subscriber>::Managed;

  template<class Message>
  LazyAcquire makeLazyAcquire(
    const std::string& topic, uint32_t queue_size,
    const Callback<Message>& callback,
    const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
    const ros::TransportHints& transport_hints = ros::TransportHints());

  template<class M, class T>
  LazyAcquire makeLazyAcquire(
    const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj,
    const ros::TransportHints& transport_hints = ros::TransportHints());


  void subscribe(const ros::NodeHandlePtr& node_handle)
  {
    return acquire(node_handle);
  }

  void shutdown()
  {
    return release();
  }
};

class ManagedServiceServer : public Managed<ManagedServiceServer, ros::ServiceServer>
{
public:
  template<class Message>
  LazyAcquire makeLazyAcquire(
    const std::string& topic, uint32_t queue_size,
    const Callback<Message>& callback,
    const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
    const ros::TransportHints& transport_hints = ros::TransportHints());

  template<class M, class T>
  LazyAcquire makeLazyAcquire(
    const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj,
    const ros::TransportHints& transport_hints = ros::TransportHints());


  void advertiseService(const ros::NodeHandlePtr& node_handle)
  {
    return acquire(node_handle);
  }

  void shutdown()
  {
    return release();
  }
};


// template <class T>
// class ManagedWrapper : public Managed<T> {};
//
// template<>
// class ManagedWrapper<ros::Subscriber> : public Managed<ros::Subscriber>
// {
// public:
//   using Managed<ros::Subscriber>::acquire;
//   using Managed<ros::Subscriber>::release;
//
//   void subscribe(const ros::NodeHandlePtr& node_handle)
//   {
//     return acquire(node_handle);
//   }
//
//   void shutdown()
//   {
//     return release();
//   }
//
//   template<class Message>
//   LazyAcquire makeLazyAcquire(
//     const std::string& topic, uint32_t queue_size,
//     const Callback<Message>& callback,
//     const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
//     const ros::TransportHints& transport_hints = ros::TransportHints());
//
//   template<class M, class T>
//   LazyAcquire makeLazyAcquire(
//     const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj,
//     const ros::TransportHints& transport_hints = ros::TransportHints());
//
// };

// template<>
// class ManagedWrapper<ros::ServiceServer> : public Managed<ros::ServiceServer>
// {
// public:
//   using Managed<ros::ServiceServer>::acquire;
//   using Managed<ros::ServiceServer>::release;
//
//   void advertiseService(const ros::NodeHandlePtr& node_handle)
//   {
//     return acquire(node_handle);
//   }
//
//   void shutdown()
//   {
//     return release();
//   }
// };

//typedef ManagedWrapper<ros::Subscriber> ManagedSubscriber;
//typedef ManagedWrapper<ros::ServiceServer> ManagedServiceServer;


// typedef Managed<ros::Subscriber> ManagedSubscriber;
// typedef Managed<ros::ServiceServer> ManagedServiceServer;

#include <robot_process/manager/managed_resource.tpp>

} // namespace robot_process

#endif
