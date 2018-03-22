#include <robot_process/subscription_manager.h>

namespace robot_process {

template<typename... Args>
ManagedSubscription SubscriptionManager::listen(Args&& ...args)
{
  // auto managed_subscription = std::make_shared<ManagedSubscription>(
  //   std::forward<Args>(args)...);

  ManagedSubscription a(std::forward<Args>(args)...);
  subscriptions_.push_back(a);
  return a;
}

void SubscriptionManager::subscribeAll(const ros::NodeHandlePtr& node_handle)
{
  for (auto&& sub: subscriptions_)
    sub.subscribe(node_handle);
}

void SubscriptionManager::unsubscribeAll()
{
  for (auto&& sub: subscriptions_)
    sub.unsubscribe();
}

void SubscriptionManager::pauseAll()
{
  for (auto&& sub: subscriptions_)
    sub.pause();
}

void SubscriptionManager::resumeAll()
{
  for (auto&& sub: subscriptions_)
    sub.resume();
}

} // namespace robot_process