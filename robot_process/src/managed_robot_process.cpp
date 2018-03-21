#include <robot_process/managed_robot_process.h>

namespace robot_process {

  void ManagedRobotProcess::onCreate()
  {
    ROS_INFO("onCreate");
    onManagedCreate();
  }

  void ManagedRobotProcess::onTerminate()
  {
    ROS_INFO("onTerminate");
    onManagedTerminate();
  }

  void ManagedRobotProcess::onConfigure()
  {
    ROS_INFO("onConfigure");
    onManagedConfigure();
  }

  void ManagedRobotProcess::onUnconfigure()
  {
    ROS_INFO("onUnconfigure");
    onManagedUnconfigure();
  }

  void ManagedRobotProcess::onStart()
  {
    ROS_INFO("onStart");
    onManagedStart();
  }

  void ManagedRobotProcess::onStop()
  {
    ROS_INFO("onStop");
    onManagedStop();
  }

  void ManagedRobotProcess::onPause()
  {
    ROS_INFO("onPause");
    onManagedPause();
  }

  void ManagedRobotProcess::onResume()
  {
    ROS_INFO("onResume");
    onManagedResume();
  }

  template<class M, class T>
  ros::Subscriber ManagedRobotProcess::subscribe(
    const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj,
    const ros::TransportHints& transport_hints)
  {
    auto sub = node_handle_->subscribe(topic, queue_size, fp, obj, transport_hints);
    managed_subscribers_.push_back(sub);
    return sub;
  }


  template<class M, class T>
  ros::Subscriber ManagedRobotProcess::subscribe(
    const std::string& topic, uint32_t queue_size, void(T::*fp)(M), const T* obj,
    const ros::TransportHints& transport_hints)
  {
    return ros::Subscriber();
  }

  template<class M, class T>
  ros::Subscriber ManagedRobotProcess::subscribe(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(const boost::shared_ptr<M const>&), T* obj,
    const ros::TransportHints& transport_hints)
  {
    return ros::Subscriber();
  }

  template<class M, class T>
  ros::Subscriber ManagedRobotProcess::subscribe(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(const boost::shared_ptr<M const>&) const, T* obj,
    const ros::TransportHints& transport_hints)
  {
    return ros::Subscriber();
  }

  template<class M, class T>
  ros::Subscriber ManagedRobotProcess::subscribe(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(M), const boost::shared_ptr<T>& obj,
    const ros::TransportHints& transport_hints)
  {
    return ros::Subscriber();
  }

  template<class M, class T>
  ros::Subscriber ManagedRobotProcess::subscribe(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(M) const, const boost::shared_ptr<T>& obj,
    const ros::TransportHints& transport_hints)
  {
    return ros::Subscriber();
  }

  template<class M, class T>
  ros::Subscriber ManagedRobotProcess::subscribe(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(const boost::shared_ptr<M const>&),
    const boost::shared_ptr<T>& obj,
    const ros::TransportHints& transport_hints)
  {
    return ros::Subscriber();
  }

  template<class M, class T>
  ros::Subscriber ManagedRobotProcess::subscribe(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(const boost::shared_ptr<M const>&) const,
    const boost::shared_ptr<T>& obj,
    const ros::TransportHints& transport_hints)
  {
    return ros::Subscriber();
  }

  template<class M>
  ros::Subscriber ManagedRobotProcess::subscribe(
    const std::string& topic, uint32_t queue_size, void(*fp)(M),
    const ros::TransportHints& transport_hints)
  {
    return ros::Subscriber();
  }

  template<class M>
  ros::Subscriber ManagedRobotProcess::subscribe(
    const std::string& topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr<M const>&),
    const ros::TransportHints& transport_hints)
  {
    return ros::Subscriber();
  }

  template<class M>
  ros::Subscriber ManagedRobotProcess::subscribe(
    const std::string& topic, uint32_t queue_size,
    const boost::function<void (const boost::shared_ptr<M const>&)>& callback,
    const ros::VoidConstPtr& tracked_object,
    const ros::TransportHints& transport_hints)
  {
    return ros::Subscriber();
  }

  template<class M, class C>
  ros::Subscriber ManagedRobotProcess::subscribe(
    const std::string& topic, uint32_t queue_size,
    const boost::function<void (C)>& callback,
    const ros::VoidConstPtr& tracked_object,
    const ros::TransportHints& transport_hints)
  {
    return ros::Subscriber();
  }


} // namespace robot_process
