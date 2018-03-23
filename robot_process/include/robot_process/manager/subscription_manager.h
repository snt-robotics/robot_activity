#ifndef SUBSCRIPTION_MANAGER_H
#define SUBSCRIPTION_MANAGER_H

#include <ros/ros.h>

#include <robot_process/manager/managed_subscription.h>

namespace robot_process {

class SubscriptionManager
{
public:
  SubscriptionManager() : subscriptions_() {}
  ~SubscriptionManager() {}

  template<typename... Args>
  void subscribe(Args&& ...args)
  {
    ManagedSubscription a(std::forward<Args>(args)...);
    //subscriptions_.push_back(a);
    //return a;
  }

  void subscribeAll(const ros::NodeHandlePtr& node_handle);
  void unsubscribeAll();

  void pauseAll();
  void resumeAll();

private:
  std::vector<ManagedSubscription> subscriptions_;
};

}

#endif
