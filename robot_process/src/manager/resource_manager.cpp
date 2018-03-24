#include <robot_process/manager/resource_manager.h>

namespace robot_process {

template<class Resource>
void ResourceManager<Resource>::acquireAll(const ros::NodeHandlePtr& node_handle)
{
  for (auto&& r: resources_)
    r->acquire(node_handle);
}

template<class Resource>
void ResourceManager<Resource>::releaseAll()
{
  for (auto&& r: resources_)
    r->release();
}

template<class Resource>
void ResourceManager<Resource>::pauseAll()
{
  for (auto&& r: resources_)
    r->pause();
}

template<class Resource>
void ResourceManager<Resource>::resumeAll()
{
  for (auto&& r: resources_)
    r->resume();
}

template class ResourceManager<ros::Subscriber>;
template class ResourceManager<ros::ServiceServer>;

} // namespace robot_process
