#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include <geometry_msgs/Pose2D.h>
#include "mm_apriltags_tracker/mm_april_tags_tracker.h"
#include "mm_apriltags_tracker/april_tag_pos.h"

using namespace std;
using namespace cv;


#define MM_APRIL_TAGS_TRACKER_VIEW "mm April Tags Tracker"
#define APRIL_TAG_POS_MSG_NAME  "april_tag_pos"
//#define TARGET_POS_MSG_NAME     "target_pos"

#define PI 3.1415926
#define REACH_THRESHOLD 30

float convRadius(float radius) {
  if( radius < 0 ) {
    radius = 2*PI + radius;
  }
  return radius;
}

void MMAprilTagsTracker::calibrate(vector<AprilTags::TagDetection> tags){
  vector<AprilTags::TagDetection> grounding;
  vector<cv::Point2f> srcPoints;
  vector<cv::Point2f> dstPoints;
  int foundNumber = 0;
  for(auto t:tags){
    if(groundLocs.end() != groundLocs.find(t.id)){
      srcPoints.push_back(Point2f(t.cxy.first, t.cxy.second));
      dstPoints.push_back(Point2f(groundLocs[t.id].first, groundLocs[t.id].second));
      foundNumber++;
    }

  }

  if(foundNumber < 4) return;
  transformer = findHomography(srcPoints, dstPoints);
  calibrated = true;
}

std::pair<double,double> MMAprilTagsTracker::transform(double x, double y){

  return std::pair<double, double>(0, 0);
  }

void MMAprilTagsTracker::imageCallback( const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy( msg, "bgr8" );
  }
  catch( cv_bridge::Exception& e ) {
    ROS_ERROR( "cv_bridge exce[topm: %s", e.what() );
    return;
  }
  vector<AprilTags::TagDetection> tags = extractTags( cv_ptr->image );
  if(!calibrated){
    calibrate(tags);
  } else if( tags.size() > 0) {
    mm_apriltags_tracker::april_tag_pos msg;
    std::vector<cv::Point2f> src;
    for( unsigned int i=0; i<tags.size(); i++ ){
      AprilTags::TagDetection tag = tags[i];
      if(groundLocs.end() != groundLocs.find(tag.id)) continue;
      tag.draw(cv_ptr->image);
      msg.id.push_back( tag.id );
      src.push_back(Point2f(tag.cxy.first, tag.cxy.second));
    }
    if(src.size() == 0) return;
    vector<Point2f> dst(src.size());
    perspectiveTransform(src, dst, transformer);

    for(int i = 0; i < dst.size() ; i++){
      geometry_msgs::Pose2D pose;
      pose.x = dst[i].x;
      pose.y = dst[i].y;
      msg.pose.push_back(pose);
    }
    m_pos_pub.publish(msg);
  }

  //std::cout << "paint " << std::endl;
  cv::imshow(MM_APRIL_TAGS_TRACKER_VIEW, cv_ptr->image );
  /*
  int key_value = cv::waitKey(30);
  if( key_value == (int)('q') ) {
    ros::shutdown();
  }*/
  if( false == ros::ok() ) {
    ros::shutdown();
  }
}

MMAprilTagsTracker::MMAprilTagsTracker( AprilTags::TagCodes codes  ) : m_it( m_nh ) , m_tag_codes( codes )  {
  cv::namedWindow(MM_APRIL_TAGS_TRACKER_VIEW);
  cv::startWindowThread();

  m_sub = m_it.subscribe("image",
                          1,
                          &MMAprilTagsTracker::imageCallback,
                          this);
  mp_tag_detector = new AprilTags::TagDetector( m_tag_codes );

  m_pos_pub = m_nh.advertise<mm_apriltags_tracker::april_tag_pos>(
        APRIL_TAG_POS_MSG_NAME, 1 );

  std::map<std::string, int> param;
  m_nh.getParam("/abs_april_tag_loc", param);
  std::map<int, std::pair<double,double> > absLocs;

  for (auto x: param){
    int id = stoi(x.first.substr(0, x.first.size() - 1));
    std::cout << "id: " << id << "\t";
    std::pair<double, double> coord (0,0);
    if(absLocs.end() != absLocs.find(id)){
      coord = absLocs[id];
    }
    if(x.first[x.first.size() - 1] == 'x'){
      coord.first = (double)x.second;
    } else {
      coord.second = (double)x.second;
    }
    std::cout << "(" << coord.first << ", " << coord.second <<")\n";
    absLocs[id] = coord;
  }
  groundLocs = absLocs;
}

MMAprilTagsTracker::~MMAprilTagsTracker() {
  if( mp_tag_detector ) {
    delete mp_tag_detector;
    mp_tag_detector = NULL;
  }
  cv::destroyWindow( MM_APRIL_TAGS_TRACKER_VIEW );
}

std::vector<AprilTags::TagDetection> MMAprilTagsTracker::extractTags( cv::Mat& image) {
   cv::Mat gray_img;
   cvtColor( image, gray_img, CV_BGR2GRAY );
   return mp_tag_detector->extractTags( gray_img );
}
