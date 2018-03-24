#ifndef SUBSCRIPTION_MANAGER_H
#define SUBSCRIPTION_MANAGER_H

#include <ros/ros.h>
#include <ros/console.h>

#include <robot_process/manager/managed_resource.h>

namespace robot_process {

template <class Resource>
class ResourceManager
{
public:
  ResourceManager() : resources_() {}
  ~ResourceManager() {}

  template<typename... Args>
  typename Managed<Resource>::SharedPtr acquire(Args&& ...args)
  {
    auto managed_subscription =
      std::make_shared<Managed<Resource>>(std::forward<Args>(args)...);
    resources_.push_back(managed_subscription);
    return managed_subscription;
  }

  void acquireAll(const ros::NodeHandlePtr& node_handle);
  void releaseAll();

  void pauseAll();
  void resumeAll();

private:
  std::vector<typename Managed<Resource>::SharedPtr> resources_;
};

template <class T>
class RMWrapper : public ResourceManager<T> {};

template<>
class RMWrapper<ros::Subscriber> : public ResourceManager<ros::Subscriber>
{
public:
  using ResourceManager<ros::Subscriber>::acquire;

  template<typename... Args>
  Managed<ros::Subscriber>::SharedPtr subscribe(Args&& ...args)
  {
    return acquire(std::forward<Args>(args)...);
  }
};

template<>
class RMWrapper<ros::ServiceServer> : public ResourceManager<ros::ServiceServer>
{
public:
  using ResourceManager<ros::ServiceServer>::acquire;

  template<typename... Args>
  Managed<ros::ServiceServer>::SharedPtr advertiseService(Args&& ...args)
  {
    return acquire(std::forward<Args>(args)...);
  }
};

typedef RMWrapper<ros::Subscriber> SubscriberManager;
typedef RMWrapper<ros::ServiceServer> ServiceServerManager;

}

#endif
