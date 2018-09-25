#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/CameraInfo.h>

namespace pointgrey_camera_driver
{
class PointGreySyncStereoImagesNodelet : public nodelet::Nodelet
{
public:
  // Constructor
  PointGreySyncStereoImagesNodelet() = default;

  ~PointGreySyncStereoImagesNodelet() = default;

private:
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    ros::NodeHandle lnh(nh, "/pointgrey/left");
    ros::NodeHandle rnh(nh, "/pointgrey/right");
    ros::NodeHandle lpnh(pnh, "/pointgrey/left");
    ros::NodeHandle rpnh(pnh, "/pointgrey/right");
    image_transport::ImageTransport it(lnh);
    image_transport::ImageTransport rit(rnh);

    int queue_size;
    bool approximate_sync;
    pnh.param("approximate_sync", approximate_sync, true);
    pnh.param("queue_size", queue_size, 100);

    if (approximate_sync)
    {
      approximate_sync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queue_size),
                                                                                image_filter_, rimage_filter_);
      approximate_sync_->registerCallback(boost::bind(&PointGreySyncStereoImagesNodelet::callback, this, _1, _2));
    }
    else
    {
      exact_sync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queue_size), image_filter_,
                                                                         rimage_filter_);
      exact_sync_->registerCallback(boost::bind(&PointGreySyncStereoImagesNodelet::callback, this, _1, _2));
    }

    image_filter_.subscribe(it, lnh.resolveName("dilate/output"), 10);
    rimage_filter_.subscribe(rit, rnh.resolveName("dilate/output"), 10);
  };

  void callback(const sensor_msgs::ImageConstPtr& image_filter, const sensor_msgs::ImageConstPtr& rimage_filter)
  {
    std::cerr << "callback" << std::endl;
    ros::Time now = ros::Time::now();
    std::cerr << "now" << now.sec << "[s] + " << now.nsec << "[ns] + " << std::endl;
    std::cerr << image_filter->header.stamp.sec << "[s] + " << image_filter->header.stamp.nsec << "[ns] + "
              << std::endl;
  }

  image_transport::SubscriberFilter image_filter_;
  image_transport::SubscriberFilter rimage_filter_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MyApproxSyncPolicy;
  message_filters::Synchronizer<MyApproxSyncPolicy>* approximate_sync_;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MyExactSyncPolicy;
  message_filters::Synchronizer<MyExactSyncPolicy>* exact_sync_;
};

PLUGINLIB_DECLARE_CLASS(pointgrey_camera_driver, PointGreySyncStereoImagesNodelet,
                        pointgrey_camera_driver::PointGreySyncStereoImagesNodelet, nodelet::Nodelet);
}
