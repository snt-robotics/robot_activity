#include <robot_process/manager/managed_subscription_impl.h>

namespace robot_process {

ManagedSubscription::Impl::Impl()
: subscribed_(false), paused_(true), subscriber_(), lazy_subscribe_()
{
  ROS_DEBUG("ManagedSubscription::Imp::ctor !")
}

ManagedSubscription::Impl::~Impl()
{
  ROS_DEBUG("ManagedSubscription::Imp::dtor !");
  unsubscribe();
}


void ManagedSubscription::Impl::subscribe(const ros::NodeHandlePtr& node_handle)
{
  ROS_DEBUG("ManagedSubscription::subscribe executed!");
  if (subscribed_)
  {
    ROS_WARN("Already subscribed!");
    return;
  }

  ROS_DEBUG("Subscribing...");
  subscriber_ = lazy_subscribe_(node_handle);
  subscribed_ = true;
}

void ManagedSubscription::Impl::unsubscribe()
{
  ROS_DEBUG("ManagedSubscription::unsubscribe executed!");
  if (subscribed_)
  {
    ROS_DEBUG("Unsubscribing...");
    subscriber_.shutdown();
    subscribed_ = false;
  }
  else
  {
    ROS_WARN("Cannot unsubscribe ");
  }
}

void ManagedSubscription::Impl::pause()
{
  ROS_DEBUG("ManagedSubscription::pause executed!");
  paused_ = true;
}

void ManagedSubscription::Impl::resume()
{
  ROS_DEBUG("ManagedSubscription::resume executed!");
  paused_ = false;
}

} // namespace robot_process
