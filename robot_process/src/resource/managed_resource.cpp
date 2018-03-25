#include <robot_process/resource/managed_resource.h>

namespace robot_process {
namespace resource {

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
  resource_ = lazy_acquirer_(node_handle);
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

} // namespace resource
} // namespace robot_process
