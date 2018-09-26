#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <limits>
#include <boost/thread.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

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
      }
    }
    else if (not pub_thread_)
    {
      pub_thread_.reset(new boost::thread(boost::bind(&PointGreySyncStereoImagesNodelet::loop, this)));
    }
  }

  virtual void onInit()
  {
    ros::NodeHandle& pnh = getMTPrivateNodeHandle();
    pnh.param("approximate_sync", approximate_sync_, true);
    pnh.param("queue_size", queue_size_, 10);

    boost::mutex::scoped_lock scoped_lock(connect_mutex_);

    ros::SubscriberStatusCallback cb = boost::bind(&PointGreySyncStereoImagesNodelet::connectCb, this);
    pub_ = getMTNodeHandle().advertise<geometry_msgs::Point>("/pointgrey/ball_point", 1, cb, cb);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr&)
  {
    // this is dummy; we need this to acquire camera_info
  }

  void infoCallback(const sensor_msgs::CameraInfoConstPtr& msg, const bool& is_left)
  {
    sensor_msgs::CameraInfo ci = *msg;
    if (is_left and not ci_)
    {
      ci_.reset(new sensor_msgs::CameraInfo(ci));
    }
    else if (not is_left and not rci_)
    {
      rci_.reset(new sensor_msgs::CameraInfo(ci));
    }
  }

  template <typename T = Eigen::MatrixXd>
  T pseudoInverse(const T& m, double eps = std::numeric_limits<double>::epsilon())
  {
    using namespace Eigen;
    typedef JacobiSVD<T> TSVD;
    unsigned int svd_opt(ComputeThinU | ComputeThinV);
    if (m.RowsAtCompileTime != Dynamic || m.ColsAtCompileTime != Dynamic)
      svd_opt = ComputeFullU | ComputeFullV;
    TSVD svd(m, svd_opt);
    const typename TSVD::SingularValuesType& sigma(svd.singularValues());
    typename TSVD::SingularValuesType sigma_inv(sigma.size());
    for (long i = 0; i < sigma.size(); ++i)
    {
      if (sigma(i) > eps)
        sigma_inv(i) = 1.0 / sigma(i);
      else
        sigma_inv(i) = 0.0;
    }
    return svd.matrixV() * sigma_inv.asDiagonal() * svd.matrixU().transpose();
  }

  void momentsCallback(const boost::shared_ptr<opencv_apps::MomentArrayStamped const>& moments,
                       const boost::shared_ptr<opencv_apps::MomentArrayStamped const>& rmoments)
  {
    boost::this_thread::disable_interruption no_interruption{};
    boost::mutex::scoped_lock scoped_lock(connect_mutex_);

    if (not moments or not rmoments)
    {
      std::cerr << "moments or rmoments is nullptr " << std::endl;
      return;
    }
    if (moments->moments.empty() or rmoments->moments.empty())
    {
      std::cerr << "moments are empty" << std::endl;
      return;
    }
    // getting each ball position in rectified camera pixel
    auto center_it = std::max_element(
        moments->moments.begin(), moments->moments.end(),
        [](const opencv_apps::Moment& m1, const opencv_apps::Moment& m2) { return m1.area < m2.area; });
    double center_x = center_it->center.x;
    double center_y = center_it->center.y;
    auto rcenter_it = std::max_element(
        rmoments->moments.begin(), rmoments->moments.end(),
        [](const opencv_apps::Moment& m1, const opencv_apps::Moment& m2) { return m1.area < m2.area; });
    double rcenter_x = rcenter_it->center.x;
    double rcenter_y = rcenter_it->center.y;
    std::cerr << "(" << center_x << ", " << center_y << ") "
              << "(" << rcenter_x << ", " << rcenter_y << ") " << std::endl;
    uint32_t height = ci_->height;
    uint32_t width = ci_->width;
    uint32_t rheight = rci_->height;
    uint32_t rwidth = rci_->width;
    Eigen::MatrixXd p(3, 4);
    Eigen::MatrixXd rp(3, 4);
    p << ci_->P[0], ci_->P[1], ci_->P[2], ci_->P[3], ci_->P[4], ci_->P[5], ci_->P[6], ci_->P[7], ci_->P[8], ci_->P[9],
        ci_->P[10], ci_->P[11];
    rp << rci_->P[0], rci_->P[1], rci_->P[2], rci_->P[3], rci_->P[4], rci_->P[5], rci_->P[6], rci_->P[7], rci_->P[8],
        rci_->P[9], rci_->P[10], rci_->P[11];

    // (pixel_x, pixel_y, 1)
    Eigen::Vector3d pixel;
    Eigen::Vector3d rpixel;
    pixel << center_x, center_y, 1;
    rpixel << rcenter_x, rcenter_y, 1;
    Eigen::VectorXd line = pseudoInverse(p) * pixel;
    Eigen::VectorXd rline = pseudoInverse(rp) * rpixel;
    std::cerr << "line and rline" << std::endl;
    std::cerr << line[0] << " " << line[1] << " " << line[2] << " " << line[3] << std::endl;
    std::cerr << rline[0] << " " << rline[1] << " " << rline[2] << " " << rline[3] << std::endl;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    p_msg_.x = x;
    p_msg_.y = y;
    p_msg_.z = z;
    pub_.publish(p_msg_);
  }

  void loop()
  {
    // Image subscriber is dummy
    ros::Subscriber sub_cam_img = getMTNodeHandle().subscribe<sensor_msgs::Image>(
        "/pointgrey/left/image_raw", 10, boost::bind(&PointGreySyncStereoImagesNodelet::imageCallback, this, _1));
    ros::Subscriber rsub_cam_img = getMTNodeHandle().subscribe<sensor_msgs::Image>(
        "/pointgrey/right/image_raw", 10, boost::bind(&PointGreySyncStereoImagesNodelet::imageCallback, this, _1));
    ros::Subscriber sub_cam_info = getMTNodeHandle().subscribe<sensor_msgs::CameraInfo>(
        "/pointgrey/left/camera_info", 10,
        boost::bind(&PointGreySyncStereoImagesNodelet::infoCallback, this, _1, true));
    ros::Subscriber rsub_cam_info = getMTNodeHandle().subscribe<sensor_msgs::CameraInfo>(
        "/pointgrey/right/camera_info", 10,
        boost::bind(&PointGreySyncStereoImagesNodelet::infoCallback, this, _1, false));

    // getting camera info
    ros::Rate loop(10);
    while (ros::ok())
    {
      if (ci_ and rci_)
      {
        sub_cam_info.shutdown();
        rsub_cam_info.shutdown();
        break;
      }
      loop.sleep();
    }

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
    if (approximate_sync_)
    {
      approximate_synchronizer.reset(
          new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queue_size_), *sub, *rsub));
      approximate_synchronizer->registerCallback(
          boost::bind(&PointGreySyncStereoImagesNodelet::momentsCallback, this, _1, _2));
    }
    else
    {
      exact_synchronizer_.reset(
          new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queue_size_), *sub, *rsub));
      exact_synchronizer_->registerCallback(
          boost::bind(&PointGreySyncStereoImagesNodelet::momentsCallback, this, _1, _2));
    }
    pub_thread_->yield();
    boost::this_thread::disable_interruption no_interruption{};
    while (not boost::this_thread::interruption_requested())
    {
    }
  }

  geometry_msgs::Point p_msg_;
  ros::Publisher pub_;
  std::unique_ptr<sensor_msgs::CameraInfo> ci_ = nullptr;
  std::unique_ptr<sensor_msgs::CameraInfo> rci_ = nullptr;

  boost::shared_ptr<boost::thread> pub_thread_;
  boost::mutex connect_mutex_;

  bool approximate_sync_ = true;
  int queue_size_ = 5;

  ros::NodeHandle pnh_;

  typedef message_filters::sync_policies::ExactTime<opencv_apps::MomentArrayStamped, opencv_apps::MomentArrayStamped>
      MyExactSyncPolicy;
  std::unique_ptr<message_filters::Synchronizer<MyExactSyncPolicy>> exact_synchronizer_;
};

PLUGINLIB_DECLARE_CLASS(pointgrey_camera_driver, PointGreySyncStereoImagesNodelet,
                        pointgrey_camera_driver::PointGreySyncStereoImagesNodelet, nodelet::Nodelet);
}
