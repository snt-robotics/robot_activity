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
  typename Resource::SharedPtr acquire(Args&& ...args)
  {
    auto managed_subscription =
      std::make_shared<Resource>(std::forward<Args>(args)...);
    resources_.push_back(managed_subscription);
    return managed_subscription;
  }

  void acquireAll(const ros::NodeHandlePtr& node_handle);
  void releaseAll();

  void pauseAll();
  void resumeAll();

private:
  std::vector<typename Resource::SharedPtr> resources_;
};

template <class T>
class RMWrapper : public ResourceManager<T> {};

template<>
class RMWrapper<ManagedSubscriber> : public ResourceManager<ManagedSubscriber>
{
public:
  using ResourceManager<ManagedSubscriber>::acquire;

  template<typename... Args>
  ManagedSubscriber::SharedPtr subscribe(Args&& ...args)
  {
    return acquire(std::forward<Args>(args)...);
  }
};

template<>
class RMWrapper<ManagedServiceServer> : public ResourceManager<ManagedServiceServer>
{
public:
  using ResourceManager<ManagedServiceServer>::acquire;

  template<typename... Args>
  ManagedServiceServer::SharedPtr advertiseService(Args&& ...args)
  {
    return acquire(std::forward<Args>(args)...);
  }
};

typedef RMWrapper<ManagedSubscriber> SubscriberManager;
typedef RMWrapper<ManagedServiceServer> ServiceServerManager;

}

#endif
