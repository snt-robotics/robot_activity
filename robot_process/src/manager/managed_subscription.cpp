#include <robot_process/manager/managed_subscription.h>

namespace robot_process {

ManagedSubscription::~ManagedSubscription()
{
  ROS_DEBUG("ManagedSubscription::dtor");
}

void ManagedSubscription::subscribe(const ros::NodeHandlePtr& node_handle)
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

void ManagedSubscription::unsubscribe()
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

void ManagedSubscription::pause()
{
  ROS_DEBUG("ManagedSubscription::pause executed!");
  paused_ = true;
}

void ManagedSubscription::resume()
{
  ROS_DEBUG("ManagedSubscription::resume executed!");
  paused_ = false;
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
