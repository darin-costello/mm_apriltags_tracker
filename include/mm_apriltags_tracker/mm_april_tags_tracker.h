#ifndef APRIL_TAGS_TRACKER_H_
#define APRIL_TAGS_TRACKER_H_

#include <utility>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
#include <Eigen/Dense>

struct Pos2D{
  int x;
  int y;
};

class MMAprilTagsTracker {
public:
  MMAprilTagsTracker( AprilTags::TagCodes codes = AprilTags::tagCodes36h11 );
  virtual ~MMAprilTagsTracker();

  void imageCallback( const sensor_msgs::ImageConstPtr& msg);
  std::vector<AprilTags::TagDetection> extractTags( cv::Mat& image);

  ros::NodeHandle                 m_nh;
  image_transport::ImageTransport m_it;
  image_transport::Subscriber     m_sub;

  ros::Publisher                  m_pos_pub;
  ros::Publisher                  m_t_pos_pub;



  AprilTags::TagDetector* mp_tag_detector;
  AprilTags::TagCodes     m_tag_codes;

  std::map<int, std::pair<double,double> > groundLocs;

  bool calibrated = false;
  void calibrate(vector<AprilTags::TagDetection>);
  std::pair<double,double> transform(double x, double y);

  Eigen::Vector4d transformVect;


};

#endif // APRIL_TAGS_TRACKER_H_
