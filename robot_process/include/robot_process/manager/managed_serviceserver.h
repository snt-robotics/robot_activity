#ifndef MANAGED_SERVICESERVER_H
#define MANAGED_SERVICESERVER_H

#include <ros/ros.h>
#include <ros/console.h>

#include <robot_process/manager/managed_resource.h>

namespace robot_process {
namespace resource {

class ManagedServiceServer : public Managed<ManagedServiceServer, ros::ServiceServer>
{
public:
  using Managed<ManagedServiceServer, ros::ServiceServer>::Managed;

  void advertiseService(const ros::NodeHandlePtr& node_handle)
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

};

} // namespace resource
} // namespace robot_process

#endif
