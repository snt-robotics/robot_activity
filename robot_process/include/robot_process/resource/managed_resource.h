/*!
   \file managed_subscriber.h
   \brief Managed<Derived,R> class implements a base class which manages 
     ROS resources, such as ros::Subscriber and ros::ServiceServer.
     It uses CRTP idiom for static polymorphism and adds functionality to
     pause and resume, as well as acquire and release.
   \author Maciej Marcin ZURAD
   \date 01/03/2018
*/
#ifndef MANAGED_RESOURCE_H
#define MANAGED_RESOURCE_H

#include <atomic>

#include <ros/ros.h>
#include <ros/console.h>

namespace robot_process { 
namespace resource {

class ManagedSubscriber;
class ManagedServiceServer;

/**
 * @brief Wrapper for ROS resources, such as Subscriber and ServiceServer
 * @details This class adds additional functionality to ROS resources such
 *          as pausing, resuming and re-acquiring it. It also implements 
 *          lazy acquisition of a resource due to the fact that the resource is 
 *          not acquired upon instantiation, but when acquire is called.
 *          The class is the base wrapper class and has to be sub-classed by a 
 *          specific resource. We use CRTP idiom, hence the templated class,
 *          in order to achieve static-time (compile-time) polymorphism.
 *        
 * 
 * @tparam Specialization CRTP derived class
 * @tparam Resource ROS resource, such as ros::Subscriber or ros::ServiceServer
 */
template <class Specialization, class Resource>
class Managed
{
public:

  /**
   * @brief Default empty constructor is deleted
   * @details Resource has to be fully described upon instantiation
   */
  Managed() = delete;

  /**
   * @brief Destructor
   */
  ~Managed();

  /**
   * @brief Variadic constructor
   * @details Main constructor, where args should fully specify the resource
   *          and correspond to one of the original function signatures from 
   *          ROS resources (eg. ros::Subscriber). Args are forwarded 
   *          to the makeLazyAcquirer function, which is bound at compile-time 
   *          to the deriving class, which HAS to implement ALL function signatures
   *          available in ROS for creating that particular resource via 
   *          a node handle. 
   * 
   * @tparam Args Types have to match ROS resource creation function signature 
   *         (e.g node_handle->subscribe(...))
   * @param args Specify the resource, e.g. topic, queue_size and a callback
   */
  template<typename... Args>
  Managed(Args&& ...args)
  : acquired_(false), paused_(true), resource_(), lazy_acquirer_()
  {
    ROS_DEBUG("Managed::ctor");
    lazy_acquirer_ = makeLazyAcquirer(std::forward<Args>(args)...);
  }

  /**
   * @brief Acquires the resource if it's not already acquired
   * 
   * @param node_handle NodeHandle is needed for acquisition like in ROS
   */
  void acquire(const ros::NodeHandlePtr& node_handle);

  /**
   * @brief Release the resource if it's already acquired
   */
  void release();

  /**
   * @brief Pauses the resource
   * @details Depending on the deriving concrete resource, it usually means that
   *          the callback is returned before actually executing
   */
  void pause();

  /**
   * @brief Resumes the resource
   * @details Undoes the pause operation
   */
  void resume();

  typedef std::shared_ptr<Managed<Specialization, Resource>> SharedPtr;

protected:

  typedef std::function<Resource(const ros::NodeHandlePtr&)> LazyAcquirer;

  /**
   * @brief Lazily acquires a resource
   * @details Creates a function that when called with a ROS node handle will 
   *          acquire the specifiec resource
   * 
   * @param args Specifies the resource 
   * @return Function that will acquire resource when called with a node handle
   */
  template<typename... Args>
  LazyAcquirer makeLazyAcquirer(Args&& ...args) const
  {
    return static_cast<const Specialization*>(this)
      ->makeLazyAcquirer(std::forward<Args>(args)...);
  }

  /**
   * \brief Atomic flag specifing whether the resource is acquired or not
   */
  std::atomic<bool> acquired_;

  /**
   * \brief Atomic flag specifing whether the resource is paused or not
   */
  std::atomic<bool> paused_;


  /**
   * \brief The actual resource controlled by this class
   */
  Resource resource_;

  /**
   * \brief Function that will acquire the resource upon calling with a node handle
   */
  LazyAcquirer lazy_acquirer_;

};

} // namespace resource
} // namespace robot_process

#endif
