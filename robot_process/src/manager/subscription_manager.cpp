#include <robot_process/manager/subscription_manager.h>

namespace robot_process {

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
