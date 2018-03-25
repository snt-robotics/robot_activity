#ifndef MANAGED_SUBSCRIBER_H
#define MANAGED_SUBSCRIBER_H

#include <ros/ros.h>
#include <ros/console.h>

#include <robot_process/manager/managed_resource.h>

namespace robot_process {
namespace resource {

class ManagedSubscriber : public Managed<ManagedSubscriber, ros::Subscriber>
{
public:
  using Managed<ManagedSubscriber, ros::Subscriber>::Managed;

  void subscribe(const ros::NodeHandlePtr& node_handle)
  {
    return acquire(node_handle);
  }

  void shutdown()
  {
    return release();
  }

  template<class Message>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    const Callback<Message>& callback,
    const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
    const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ROS_DEBUG("makeLazyAcquirer Callback<Message>& callback form exec");
    return [=](const ros::NodeHandlePtr& nh) -> ros::Subscriber {
      ROS_DEBUG("Subscribing...");
      return nh->subscribe<Message>(
        topic,
        queue_size,
        static_cast<Callback<Message>>(wrapCallback(callback)),
        tracked_object,
        transport_hints);
    };
  }


  template<class M, class T>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj,
    const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ROS_DEBUG("makeLazyAcquirer void(T::*fp)(M), T* obj, form exec");
    Callback<M> callback = boost::bind(fp, obj, _1);
    return makeLazyAcquirer(topic, queue_size, callback, ros::VoidConstPtr(), transport_hints);
  }

  template<class M, class T>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size, void(T::*fp)(M), const T* obj,
    const ros::TransportHints& transport_hints)
  {
    ROS_DEBUG("makeLazyAcquirer void(T::*fp)(M), const T* obj, form exec");
    Callback<M> callback = boost::bind(fp, obj, _1);
    return makeLazyAcquirer(topic, queue_size, callback, ros::VoidConstPtr(), transport_hints);
  }

  template<class M, class T>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(const boost::shared_ptr<M const>&), T* obj,
    const ros::TransportHints& transport_hints)
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
      << "void(T::*fp)(const boost::shared_ptr<M const>&), "
      << "T* obj, form exec");
    Callback<M> callback = boost::bind(fp, obj, _1);
    return makeLazyAcquirer(topic, queue_size, callback, ros::VoidConstPtr(), transport_hints);
  }

  template<class M, class T>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(const boost::shared_ptr<M const>&) const, T* obj,
    const ros::TransportHints& transport_hints)
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
      << "void(T::*fp)(const boost::shared_ptr<M const>&) const, "
      << "T* obj, form exec");
    Callback<M> callback = boost::bind(fp, obj, _1);
    return makeLazyAcquirer(topic, queue_size, callback, ros::VoidConstPtr(), transport_hints);
  }

  template<class M, class T>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(M), const boost::shared_ptr<T>& obj,
    const ros::TransportHints& transport_hints)
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
      << "void(T::*fp)(M), "
      << "const boost::shared_ptr<T>& obj, form exec");
    Callback<M> callback = boost::bind(fp, obj.get(), _1);
    return makeLazyAcquirer(topic, queue_size, callback, obj, transport_hints);
  }

  template<class M, class T>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(M) const, const boost::shared_ptr<T>& obj,
    const ros::TransportHints& transport_hints)
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
      << "void(T::*fp)(M) const, "
      << "const boost::shared_ptr<T>& obj, form exec");
    Callback<M> callback = boost::bind(fp, obj.get(), _1);
    return makeLazyAcquirer(topic, queue_size, callback, obj, transport_hints);
  }

  template<class M, class T>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(const boost::shared_ptr<M const>&),
    const boost::shared_ptr<T>& obj,
    const ros::TransportHints& transport_hints)
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
      << "void(T::*fp)(const boost::shared_ptr<M const>&), "
      << "const boost::shared_ptr<T>& obj, form exec");
    Callback<M> callback = boost::bind(fp, obj.get(), _1);
    return makeLazyAcquirer(topic, queue_size, callback, obj, transport_hints);
  }

  template<class M, class T>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(const boost::shared_ptr<M const>&) const,
    const boost::shared_ptr<T>& obj,
    const ros::TransportHints& transport_hints)
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
      << "void(T::*fp)(const boost::shared_ptr<M const>&) const, "
      << "const boost::shared_ptr<T>& obj, form exec");
    Callback<M> callback = boost::bind(fp, obj.get(), _1);
    return makeLazyAcquirer(topic, queue_size, callback, obj, transport_hints);
  }

  template<class M>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size, void(*fp)(M),
    const ros::TransportHints& transport_hints)
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
      << "void(*fp)(M) form exec");
    Callback<M> callback = boost::bind(fp);
    return makeLazyAcquirer(topic, queue_size, callback, ros::VoidConstPtr(), transport_hints);
  }

  template<class M>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr<M const>&),
    const ros::TransportHints& transport_hints)
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
      << "void(*fp)(const boost::shared_ptr<M const>&) form exec");
    Callback<M> callback = boost::bind(fp);
    return makeLazyAcquirer(topic, queue_size, callback, ros::VoidConstPtr(), transport_hints);
  }

  // template<class M>
  // LazyAcquirer makeLazyAcquirer(
  //   const std::string& topic, uint32_t queue_size,
  //   const boost::function<void (const boost::shared_ptr<M const>&)>& callback,
  //   const ros::VoidConstPtr& tracked_object,
  //   const ros::TransportHints& transport_hints)
  // {
  //   ROS_DEBUG_STREAM("makeLazyAcquirer "
  //     << "const boost::function<void (const boost::shared_ptr<M const>&)>& callback, "
  //     << "form exec");
  //   Callback<const boost::shared_ptr<M const>&> callback = callback;
  //   return makeLazyAcquirer(topic, queue_size, callback, tracked_object, transport_hints);
  // }

  template<class M, class C>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    const boost::function<void (C)>& callback,
    const ros::VoidConstPtr& tracked_object,
    const ros::TransportHints& transport_hints)
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
      << "const boost::function<void (C)>& callback form exec");
    return makeLazyAcquirer(topic, queue_size, callback, tracked_object, transport_hints);
  }

};


} // namespace resource
} // namespace robot_process

#endif