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

  template <typename ...Args>
  using ServiceCallback = boost::function<bool(Args...)>;

  template <typename ...Args>
  ServiceCallback<Args...> wrapServiceCallback(
    const ServiceCallback<Args...>& callback) const
  {
    return [this, &callback](Args ... args) -> bool {
      ROS_DEBUG("wrapped service callback executed!");
      if (paused_)
      {
        ROS_DEBUG("service is paused!");
        return false;
      }
      return callback(std::forward<Args>(args)...);
    };
  }

  template<class MReq, class MRes>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service, 
    const ServiceCallback<MReq&, MRes&>& callback,
    const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr()) const
  {
    ROS_DEBUG("makeLazyAcquirer ServiceCallback<MReq&, MRes&>&");
    return [=](const ros::NodeHandlePtr& nh) -> ros::ServiceServer {
      ROS_DEBUG("Advertising...");
      return nh->advertiseService(
        service,
        static_cast<ServiceCallback<MReq&, MRes&>>(wrapServiceCallback(callback)),
        tracked_object);
    };
  }
 
  template<class ServiceEvent>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service, 
    const ServiceCallback<ServiceEvent&>& callback, 
    const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr()) const
  {
    ROS_DEBUG("makeLazyAcquirer ServiceEventCallback<ServiceEvent&>&");
    return [=](const ros::NodeHandlePtr& nh) -> ros::ServiceServer {
      ROS_DEBUG("Advertising...");
      return nh->advertiseService(
        service,
        static_cast<ServiceCallback<ServiceEvent&>>(wrapServiceCallback(callback)),
        tracked_object);
    };
  }

  template<class T, class MReq, class MRes>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service, 
    bool(T::*srv_func)(MReq&, MRes&), 
    T *obj) const
  {
    ServiceCallback<MReq&, MRes&> callback = boost::bind(srv_func, obj, _1, _2);
    return makeLazyAcquirer(service, callback);
  }

  template<class T, class MReq, class MRes>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service, 
    bool(T::*srv_func)(ros::ServiceEvent<MReq, MRes>&), 
    T *obj) const
  {
    ServiceCallback<ros::ServiceEvent<MReq, MRes>&> callback 
      = boost::bind(srv_func, obj, _1);
    return makeLazyAcquirer(service, callback);
  }

  template<class T, class MReq, class MRes>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service, 
    bool(T::*srv_func)(MReq &, MRes &), 
    const boost::shared_ptr<T>& obj) const
  {
    ServiceCallback<MReq&, MRes&> callback = boost::bind(srv_func, obj.get(), _1, _2);
    return makeLazyAcquirer(service, callback, obj);
  }

  template<class T, class MReq, class MRes>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service, 
    bool(T::*srv_func)(ros::ServiceEvent<MReq, MRes>&), 
    const boost::shared_ptr<T>& obj) const
  {
    ServiceCallback<ros::ServiceEvent<MReq, MRes>&> callback 
      = boost::bind(srv_func, obj.get(), _1);
    return makeLazyAcquirer(service, callback, obj);
  }

  template<class MReq, class MRes>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service, 
    bool(*srv_func)(MReq&, MRes&)) const
  {
    ServiceCallback<MReq&, MRes&> callback = boost::bind(srv_func);
    return makeLazyAcquirer(service, callback);
  }

  template<class MReq, class MRes>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service, 
    bool(*srv_func)(ros::ServiceEvent<MReq, MRes>&)) const
  {
    ServiceCallback<ros::ServiceEvent<MReq, MRes>&> callback 
      = boost::bind(srv_func);
    return makeLazyAcquirer(service, callback);
  }


  /* TODO - remove if unneccessary
  template<class MReq, class MRes>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service, 
    const boost::function<bool(MReq&, MRes&)>& callback, 
    const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr())
  {
    AdvertiseServiceOptions ops;
    ops.template init<MReq, MRes>(service, callback);
    ops.tracked_object = tracked_object;
    return makeLazyAcquirer(ops);
  }

  template<class S>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service, 
    const boost::function<bool(S&)>& callback, 
    const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr())
  {
    AdvertiseServiceOptions ops;
    ops.template initBySpecType<S>(service, callback);
    ops.tracked_object = tracked_object;
    return makeLazyAcquirer(ops);
  }
  */
 
};

} // namespace resource
} // namespace robot_process

#endif
