#ifndef SUBSCRIPTION_MANAGER_H
#define SUBSCRIPTION_MANAGER_H

#include <ros/ros.h>
#include <ros/console.h>

#include <robot_process/manager/managed_subscription.h>

namespace robot_process {

class SubscriptionManager
{
public:
  SubscriptionManager() : subscriptions_() {}
  ~SubscriptionManager() {}

  template<typename... Args>
  ManagedSubscriptionPtr subscribe(Args&& ...args)
  {
    auto managed_subscription =
      std::make_shared<ManagedSubscription>(std::forward<Args>(args)...);
    subscriptions_.push_back(managed_subscription);
    return managed_subscription;
  }

  void subscribeAll(const ros::NodeHandlePtr& node_handle);
  void unsubscribeAll();

  void pauseAll();
  void resumeAll();

private:
  std::vector<ManagedSubscriptionPtr> subscriptions_;
};

}

#endif
