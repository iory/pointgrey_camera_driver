#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <boost/thread.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <geometry_msgs/Point.h>
#include <opencv_apps/MomentArrayStamped.h>

namespace pointgrey_camera_driver
{
class PointGreySyncStereoImagesNodelet : public nodelet::Nodelet
{
public:
  // Constructor
  PointGreySyncStereoImagesNodelet() = default;

  ~PointGreySyncStereoImagesNodelet()
  {
    std::cerr << "destructor" << std::endl;
    boost::mutex::scoped_lock scoped_lock(connect_mutex_);

    if (pub_thread_)
    {
      pub_thread_->interrupt();
      pub_thread_->join();
    }
  }

private:
  void connectCb()
  {
    std::cerr << "connectCb" << std::endl;
    boost::mutex::scoped_lock scoped_lock(connect_mutex_);
    if (pub_.getNumSubscribers() == 0)
    {
      if (pub_thread_)
      {
        pub_thread_->interrupt();
        scoped_lock.unlock();
        pub_thread_->join();
        scoped_lock.lock();
        pub_thread_.reset();
        std::cerr << "delete everything" << std::endl;
      }
    }
    else if (not pub_thread_)
    {
      std::cerr << "not pub thread" << std::endl;
      pub_thread_.reset(new boost::thread(boost::bind(&PointGreySyncStereoImagesNodelet::loop, this)));
    }
  }

  virtual void onInit()
  {
    std::cerr << "onInit" << std::endl;
    ros::NodeHandle& pnh = getMTPrivateNodeHandle();
    pnh.param("approximate_sync", approximate_sync_, true);
    pnh.param("queue_size", queue_size_, 10);

    boost::mutex::scoped_lock scoped_lock(connect_mutex_);

    ros::SubscriberStatusCallback cb = boost::bind(&PointGreySyncStereoImagesNodelet::connectCb, this);
    pub_ = getMTNodeHandle().advertise<geometry_msgs::Point>("/pointgrey/ball_point", 1, cb, cb);
  }

  void loop()
  {
    std::unique_ptr<message_filters::Subscriber<opencv_apps::MomentArrayStamped>> sub;
    sub.reset(new message_filters::Subscriber<opencv_apps::MomentArrayStamped>(getMTNodeHandle(),
                                                                               "/pointgrey/left/centroid/moments", 1));
    std::unique_ptr<message_filters::Subscriber<opencv_apps::MomentArrayStamped>> rsub;
    rsub.reset(new message_filters::Subscriber<opencv_apps::MomentArrayStamped>(
        getMTNodeHandle(), "/pointgrey/right/centroid/moments", 1));
    typedef message_filters::sync_policies::ApproximateTime<opencv_apps::MomentArrayStamped,
                                                            opencv_apps::MomentArrayStamped>
        MyApproxSyncPolicy;
    std::unique_ptr<message_filters::Synchronizer<MyApproxSyncPolicy>> approximate_synchronizer;
    std::cerr << "loop start" << std::endl;
    if (approximate_sync_)
    {
      approximate_synchronizer.reset(
          new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queue_size_), *sub, *rsub));
      approximate_synchronizer->registerCallback(
          boost::bind(&PointGreySyncStereoImagesNodelet::callback, this, _1, _2));
    }
    else
    {
      exact_synchronizer_.reset(
          new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queue_size_), *sub, *rsub));
      exact_synchronizer_->registerCallback(boost::bind(&PointGreySyncStereoImagesNodelet::callback, this, _1, _2));
    }
    boost::this_thread::disable_interruption no_interruption{};
    pub_thread_->yield();
    while (not boost::this_thread::interruption_requested())
    {
    }
    std::cerr << "loop ended" << std::endl;
  }

  void callback(const boost::shared_ptr<opencv_apps::MomentArrayStamped const>& moments,
                const boost::shared_ptr<opencv_apps::MomentArrayStamped const>& rmoments)
  {
    boost::this_thread::disable_interruption no_interruption{};
    std::cerr << "callback" << std::endl;
    boost::mutex::scoped_lock scoped_lock(connect_mutex_);
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    p_msg_.x = x;
    p_msg_.y = y;
    p_msg_.z = z;
    pub_.publish(p_msg_);
    // ros::Time now = ros::Time::now();
    // std::cerr << "now" << now.sec << "[s] + " << now.nsec << "[ns] + " << std::endl;
    // std::cerr << image_ptr->header.stamp.sec << "[s] + " << image_ptr->header.stamp.nsec << "[ns] + " <<
    // std::endl;
  }

  geometry_msgs::Point p_msg_;
  ros::Publisher pub_;

  boost::shared_ptr<boost::thread> pub_thread_;
  boost::mutex connect_mutex_;

  bool approximate_sync_ = true;
  int queue_size_ = 10;

  ros::NodeHandle pnh_;

  typedef message_filters::sync_policies::ExactTime<opencv_apps::MomentArrayStamped, opencv_apps::MomentArrayStamped>
      MyExactSyncPolicy;
  std::unique_ptr<message_filters::Synchronizer<MyExactSyncPolicy>> exact_synchronizer_;
};

PLUGINLIB_DECLARE_CLASS(pointgrey_camera_driver, PointGreySyncStereoImagesNodelet,
                        pointgrey_camera_driver::PointGreySyncStereoImagesNodelet, nodelet::Nodelet);
}
