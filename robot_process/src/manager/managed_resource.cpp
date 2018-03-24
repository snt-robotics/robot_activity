#include <robot_process/manager/managed_resource.h>

namespace robot_process {

template<class Specialization, class Resource>
Managed<Specialization,Resource>::~Managed()
{
  ROS_DEBUG("Managed::dtor");
}

template<class Specialization, class Resource>
void Managed<Specialization,Resource>::acquire(const ros::NodeHandlePtr& node_handle)
{
  ROS_DEBUG("Managed::acquire executed!");
  if (acquired_)
  {
    ROS_DEBUG("Already acquired!");
    return;
  }

  ROS_DEBUG("Subscribing...");
  resource_ = lazy_acquire(node_handle);
  acquired_ = true;
}

template<class Specialization, class Resource>
void Managed<Specialization,Resource>::release()
{
  ROS_DEBUG("Managed::release executed!");
  if (acquired_)
  {
    ROS_DEBUG("Releasing...");
    resource_.shutdown();
    acquired_ = false;
  }
  else
  {
    ROS_DEBUG("Cannot release ");
  }
}

template<class Specialization, class Resource>
void Managed<Specialization,Resource>::pause()
{
  ROS_DEBUG("Managed::pause executed!");
  paused_ = true;
}

template<class Specialization, class Resource>
void Managed<Specialization,Resource>::resume()
{
  ROS_DEBUG("Managed::resume executed!");
  paused_ = false;
}

template class Managed<ManagedSubscriber, ros::Subscriber>;
template class Managed<ManagedServiceServer, ros::ServiceServer>;

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
