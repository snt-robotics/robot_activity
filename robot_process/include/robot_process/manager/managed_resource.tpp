template<class Message>
ManagedSubscriber::LazyAcquire ManagedSubscriber::makeLazyAcquire(
  const std::string& topic, uint32_t queue_size,
  const Callback<Message>& callback,
  const ros::VoidConstPtr& tracked_object,
  const ros::TransportHints& transport_hints)
{
  ROS_DEBUG("makeLazyAcquire Callback<Message>& callback form exec");
  return [=](const ros::NodeHandlePtr& nh) -> ros::Subscriber {
    ROS_DEBUG("Subscribing...");
    return nh->subscribe<Message>(
      topic,
      queue_size,
      static_cast<Callback<Message>>(wrapCallback(callback)),
      tracked_object,
      transport_hints);
  };
}

template<class M, class T>
ManagedSubscriber::LazyAcquire ManagedSubscriber::makeLazyAcquire(
  const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj,
  const ros::TransportHints& transport_hints)
{
  ROS_DEBUG("makeLazyAcquire void(T::*fp)(M), T* obj, form exec");
  Callback<M> callback = boost::bind(fp, obj, _1);
  return makeLazyAcquire(topic, queue_size, callback, ros::VoidConstPtr(), transport_hints);
}
