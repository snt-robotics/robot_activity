/*!
   \file resource_manager.h
   \brief ResourceManager<R> class implements a resource manager for 
    subscriptions and services 
   \author Maciej Marcin ZURAD
   \date 01/03/2018
*/
#ifndef SUBSCRIPTION_MANAGER_H
#define SUBSCRIPTION_MANAGER_H

#include <ros/ros.h>
#include <ros/console.h>

#include <robot_process/resource/managed_subscriber.h>
#include <robot_process/resource/managed_serviceserver.h>

namespace robot_process {
namespace resource {

template <class Resource>
class ResourceManager
{
public:
  ResourceManager() : resources_() {}
  ~ResourceManager() {}

  template<typename... Args>
  typename Resource::SharedPtr acquire(Args&& ...args)
  {
    auto resource = std::make_shared<Resource>(std::forward<Args>(args)...);
    resources_.push_back(resource);
    return resource;
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

} // namespace resource
} // namespace robot_process

#endif
