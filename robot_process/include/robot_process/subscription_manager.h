#ifndef SUBSCRIPTION_MANAGER_H
#define SUBSCRIPTION_MANAGER_H

#include <atomic>

#include <ros/ros.h>

namespace robot_process {

class ManagedSubscription;
typedef std::shared_ptr<ManagedSubscription> ManagedSubscriptionPtr;

class SubscriptionManager
{
public:
  SubscriptionManager() : subscriptions_() {}
  ~SubscriptionManager() {}

  template<typename... Args>
  ManagedSubscription listen(Args&& ...args);

  void subscribeAll(const ros::NodeHandlePtr& node_handle);
  void unsubscribeAll();

  void pauseAll();
  void resumeAll();

private:
  std::vector<ManagedSubscription> subscriptions_;
};


class ManagedSubscription
{
public:

  template<typename... Args>
  ManagedSubscription(Args&& ...args)
  : subscribed_(false), paused_(true), subscriber_()
  {
    make_subscriber_function_ = constructMakeSubscriberFunc(std::forward<Args>(args)...);
  }

  void subscribe(const ros::NodeHandlePtr& node_handle)
  {
    ROS_DEBUG("ManagedSubscription::subscribe executed!");
    if (subscribed_)
    {
      ROS_WARN("Already subscribed!");
      return;
    }

    ROS_DEBUG("Subscribing...");
    subscriber_ = make_subscriber_function_(node_handle);
    subscribed_ = true;
  }

  void unsubscribe()
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

  void pause()
  {
    ROS_DEBUG("ManagedSubscription::pause executed!");
    paused_ = true;
  }

  void resume()
  {
    ROS_DEBUG("ManagedSubscription::resume executed!");
    paused_ = false;
  }

private:

  std::atomic<bool> subscribed_;
  std::atomic<bool> paused_;

  ros::Subscriber subscriber_;

  template <class Message>
  using Callback = boost::function<void(Message)>;

  template<class Message>
  Callback<Message> wrapCallback(const Callback<Message>& callback) const
  {
    return [this, &callback](Message message) {
      ROS_DEBUG("WrappedCallback executed!");
      if (!paused_)
        callback(message);
      else
        ROS_DEBUG("Callback is PAUSED");
    };
  }

  typedef std::function<ros::Subscriber(const ros::NodeHandlePtr&)> MakeSubscriberFunc;
  MakeSubscriberFunc make_subscriber_function_;

  template<class M, class T>
  MakeSubscriberFunc constructMakeSubscriberFunc(
    const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj,
    const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ROS_DEBUG("constructMakeSubscriberFunc executed!");
    Callback<M> callback = boost::bind(fp, obj, _1);

    return [=](const ros::NodeHandlePtr& nh) -> ros::Subscriber {
      ROS_DEBUG("Subscription is being created!");
      return nh->subscribe<M>(
        topic, 
        queue_size, 
        static_cast<Callback<M>>(wrapCallback(callback)), 
        ros::VoidConstPtr(), 
        transport_hints);
    };
  }


  //template<class M, class T>
  // ros::Subscriber make_subscriber_creator(
  //   const std::string& topic, uint32_t queue_size, void(T::*fp)(M), const T* obj,
  //   const ros::TransportHints& transport_hints)
  // {
  //   return ros::Subscriber();
  // }
  //
  // template<class M, class T>
  // ros::Subscriber make_subscriber_creator(
  //   const std::string& topic, uint32_t queue_size,
  //   void(T::*fp)(const boost::shared_ptr<M const>&), T* obj,
  //   const ros::TransportHints& transport_hints)
  // {
  //   return ros::Subscriber();
  // }
  //
  // template<class M, class T>
  // ros::Subscriber make_subscriber_creator(
  //   const std::string& topic, uint32_t queue_size,
  //   void(T::*fp)(const boost::shared_ptr<M const>&) const, T* obj,
  //   const ros::TransportHints& transport_hints)
  // {
  //   return ros::Subscriber();
  // }
  //
  // template<class M, class T>
  // ros::Subscriber make_subscriber_creator(
  //   const std::string& topic, uint32_t queue_size,
  //   void(T::*fp)(M), const boost::shared_ptr<T>& obj,
  //   const ros::TransportHints& transport_hints)
  // {
  //   return ros::Subscriber();
  // }
  //
  // template<class M, class T>
  // ros::Subscriber make_subscriber_creator(
  //   const std::string& topic, uint32_t queue_size,
  //   void(T::*fp)(M) const, const boost::shared_ptr<T>& obj,
  //   const ros::TransportHints& transport_hints)
  // {
  //   return ros::Subscriber();
  // }
  //
  // template<class M, class T>
  // ros::Subscriber make_subscriber_creator(
  //   const std::string& topic, uint32_t queue_size,
  //   void(T::*fp)(const boost::shared_ptr<M const>&),
  //   const boost::shared_ptr<T>& obj,
  //   const ros::TransportHints& transport_hints)
  // {
  //   return ros::Subscriber();
  // }
  //
  // template<class M, class T>
  // ros::Subscriber make_subscriber_creator(
  //   const std::string& topic, uint32_t queue_size,
  //   void(T::*fp)(const boost::shared_ptr<M const>&) const,
  //   const boost::shared_ptr<T>& obj,
  //   const ros::TransportHints& transport_hints)
  // {
  //   return ros::Subscriber();
  // }
  //
  // template<class M>
  // ros::Subscriber make_subscriber_creator(
  //   const std::string& topic, uint32_t queue_size, void(*fp)(M),
  //   const ros::TransportHints& transport_hints)
  // {
  //   return ros::Subscriber();
  // }
  //
  // template<class M>
  // ros::Subscriber make_subscriber_creator(
  //   const std::string& topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr<M const>&),
  //   const ros::TransportHints& transport_hints)
  // {
  //   return ros::Subscriber();
  // }
  //
  // template<class M>
  // ros::Subscriber make_subscriber_creator(
  //   const std::string& topic, uint32_t queue_size,
  //   const boost::function<void (const boost::shared_ptr<M const>&)>& callback,
  //   const ros::VoidConstPtr& tracked_object,
  //   const ros::TransportHints& transport_hints)
  // {
  //   return ros::Subscriber();
  // }
  //
  // template<class M, class C>
  // ros::Subscriber make_subscriber_creator(
  //   const std::string& topic, uint32_t queue_size,
  //   const boost::function<void (C)>& callback,
  //   const ros::VoidConstPtr& tracked_object,
  //   const ros::TransportHints& transport_hints)
  // {
  //   return ros::Subscriber();
  // }

};

}

#endif
