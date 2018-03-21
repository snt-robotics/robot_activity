#ifndef SUBSCRIPTION_MANAGER_H
#define SUBSCRIPTION_MANAGER_H

#include <atomic>

namespace robot_process {

//class SubscriptionManager;
class ManagedSubscription;

// class SubscriptionManager
// {
//
// public:
//   SubscriptionManager();
//   ~SubscriptionManager();
//
//   template <typename ...Args>
//   ManagedSubscription register(Args&& ...args)
//   {
//
//   }
//
//   void subscribeAll()
//   {
//     for (const auto& sub: subscriptions_)
//       sub.subscribe();
//   }
//
//   void unsubscribeAll()
//   {
//     for (const auto& sub: subscriptions_)
//       sub.unsubscribe();
//   }
//
//   void pauseAll();
//   {
//     for (const auto& sub: subscriptions_)
//       sub.pause();
//   }
//
//   void resumeAll();
//   {
//     for (const auto& sub: subscriptions_)
//       sub.resume();
//   }
//
// private:
//
//   std::vector<shared_ptr<ManagedSubscription>> subscriptions_;
// }

class ManagedSubscription
{
public:

  template<typename... Args>
  ManagedSubscription(Args&& ...args)
  : subscribed_(false), paused_(true), subscriber_()
  {
    subscribe_function_ = make_subscriber_creator(std::forward<Args>(args)...);
  }

  void subscribe()
  {
    if (subscribed_)
    {
      ROS_WARN("Already subscribed!");
      return;
    }

    ROS_INFO("Subscribing...")
    subscriber_ = subscribe_function_();
  }

  void unsubscribe()
  {
    if (subscribed_)
    {
      ROS_INFO("Unsubscribing...");
      subscriber_.shutdown();
    }
    else
    {
      ROS_WARN("Not subscribed!");
    }
  }

  void pause()
  {
    paused_ = true;
  }

  void resume()
  {
    paused_ = false;
  }

private:

  std::atomic<bool> subscribed_;
  std::atomic<bool> paused_;
  ros::Subscriber subscriber_;

  std::function<void(void)> subscribe_function_;

  template<class M, class T>
  std::function<void(void)> make_subscriber_creator(
    const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj,
    const ros::TransportHints& transport_hints)
  {
    auto callback = boost::bind(fp, obj, _1);

    boost::function<void(M)> wrapped_callback = [=]()

    ros::SubscribeOptions ops;
    ops.transport_hints = transport_hints;

    return [](ros::NodeHandle nh) {
      nh->subscribe(ops)
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

}

}

#endif
