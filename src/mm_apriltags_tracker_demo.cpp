#include <ros/ros.h>
#include "mm_apriltags_tracker/mm_april_tags_tracker.h"

int main( int argc, char** argv ) {
  ros::init( argc, argv, "mm_apriltags_tracker" );
  MMAprilTagsTracker tracker;
  ros::spin();
  return 0;
}
