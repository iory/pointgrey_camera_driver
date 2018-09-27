#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

int main() {
  float pd[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  cv::Mat p(cv::Size(4, 3), CV_32F, pd);
  std::vector<cv::Point2f> xy;
  xy.push_back(cv::Point2f(2, 3));

  float rpd[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  cv::Mat rp(cv::Size(4, 3), CV_32F, rpd);
  std::vector<cv::Point2f> rxy;
  rxy.push_back(cv::Point2f(1, 2));
  float resultd[4] = {0.0, 0.0, 0.0, 0.0};
  cv::Mat result(cv::Size(1, 4), CV_32F, resultd);

  std::cerr << "p: " << std::endl;
  std::cerr << p << std::endl;
  std::cerr << "rp: " << std::endl;
  std::cerr << rp << std::endl;
  std::cerr << "xy: " << std::endl;
  std::cerr << xy << std::endl;
  std::cerr << "rxy: " << std::endl;
  std::cerr << rxy << std::endl;
  std::cerr << "result: " << std::endl;
  std::cerr << result << std::endl;

  cv::triangulatePoints(p, rp, xy, rxy, result);

  std::cerr << result << std::endl;
  return 0;
}
