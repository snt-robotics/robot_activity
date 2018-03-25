#include <robot_process/resource/resource_manager.h>

namespace robot_process {
namespace resource {

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

template class ResourceManager<ManagedSubscriber>;
template class ResourceManager<ManagedServiceServer>;

} // namespace resource
} // namespace robot_process
