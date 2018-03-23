#include <robot_process/manager/managed_subscription.h>

namespace robot_process {

ManagedSubscription::Impl::Impl()
: subscribed_(false), paused_(true), subscriber_(), lazy_subscribe_()
{
  ROS_DEBUG("ManagedSubscription::Imp::ctor !");
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

ManagedSubscription::ManagedSubscription(const ManagedSubscription& rhs)
{
  ROS_DEBUG("ManagedSubscription::ManagedSubscription::copy_ctor triggered!");
  impl_ = rhs.impl_;
}

ManagedSubscription::ManagedSubscription()
{
  ROS_DEBUG("ManagedSubscription::ManagedSubscription::ctor triggered!");
}

void ManagedSubscription::subscribe(const ros::NodeHandlePtr& node_handle)
{
  impl_->subscribe(node_handle);
}

void ManagedSubscription::unsubscribe()
{
  impl_->unsubscribe();
}

void ManagedSubscription::pause()
{
  impl_->pause();
}

void ManagedSubscription::resume()
{
  impl_->resume();
}

/*
template<class M, class T>
ros::Subscriber make_subscriber_creator(
  const std::string& topic, uint32_t queue_size, void(T::*fp)(M), const T* obj,
  const ros::TransportHints& transport_hints)
{
  return ros::Subscriber();
}

template<class M, class T>
ros::Subscriber make_subscriber_creator(
  const std::string& topic, uint32_t queue_size,
  void(T::*fp)(const boost::shared_ptr<M const>&), T* obj,
  const ros::TransportHints& transport_hints)
{
  return ros::Subscriber();
}

template<class M, class T>
ros::Subscriber make_subscriber_creator(
  const std::string& topic, uint32_t queue_size,
  void(T::*fp)(const boost::shared_ptr<M const>&) const, T* obj,
  const ros::TransportHints& transport_hints)
{
  return ros::Subscriber();
}

template<class M, class T>
ros::Subscriber make_subscriber_creator(
  const std::string& topic, uint32_t queue_size,
  void(T::*fp)(M), const boost::shared_ptr<T>& obj,
  const ros::TransportHints& transport_hints)
{
  return ros::Subscriber();
}

template<class M, class T>
ros::Subscriber make_subscriber_creator(
  const std::string& topic, uint32_t queue_size,
  void(T::*fp)(M) const, const boost::shared_ptr<T>& obj,
  const ros::TransportHints& transport_hints)
{
  return ros::Subscriber();
}

template<class M, class T>
ros::Subscriber make_subscriber_creator(
  const std::string& topic, uint32_t queue_size,
  void(T::*fp)(const boost::shared_ptr<M const>&),
  const boost::shared_ptr<T>& obj,
  const ros::TransportHints& transport_hints)
{
  return ros::Subscriber();
}

template<class M, class T>
ros::Subscriber make_subscriber_creator(
  const std::string& topic, uint32_t queue_size,
  void(T::*fp)(const boost::shared_ptr<M const>&) const,
  const boost::shared_ptr<T>& obj,
  const ros::TransportHints& transport_hints)
{
  return ros::Subscriber();
}

template<class M>
ros::Subscriber make_subscriber_creator(
  const std::string& topic, uint32_t queue_size, void(*fp)(M),
  const ros::TransportHints& transport_hints)
{
  return ros::Subscriber();
}

template<class M>
ros::Subscriber make_subscriber_creator(
  const std::string& topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr<M const>&),
  const ros::TransportHints& transport_hints)
{
  return ros::Subscriber();
}

template<class M>
ros::Subscriber make_subscriber_creator(
  const std::string& topic, uint32_t queue_size,
  const boost::function<void (const boost::shared_ptr<M const>&)>& callback,
  const ros::VoidConstPtr& tracked_object,
  const ros::TransportHints& transport_hints)
{
  return ros::Subscriber();
}

template<class M, class C>
ros::Subscriber make_subscriber_creator(
  const std::string& topic, uint32_t queue_size,
  const boost::function<void (C)>& callback,
  const ros::VoidConstPtr& tracked_object,
  const ros::TransportHints& transport_hints)
{
  return ros::Subscriber();
}
*/

} // namespace robot_process
