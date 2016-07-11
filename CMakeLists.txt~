# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8)

# Declare the name of the CMake Project
project(multi_apriltags_tracker)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
             roscpp
             sensor_msgs
             cv_bridge
             image_transport
             std_msgs
             message_generation
            )

#find_package(OpenCV 2.4 REQUIRED COMPONENTS core highgui imgproc calib3d
#                                            ml features2d objdetect )
find_package(cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

# Find package apriltags
find_package(PkgConfig)
pkg_check_modules(APRILTAGS apriltags)
if( NOT APRILTAGS_FOUND )
  message( "Apriltags not found\n" )
  return()
endif()
find_path(APRILTAGS_INCLUDE_DIR AprilTags/TagDetector.h
    HINTS ${APRILTAGS_INCLUDEDIR} ${APRILTAGS_INCLUDE_DIRS})

find_library(APRILTAGS_LIBRARY apriltags
    HINTS ${APRILTAGS_LIBDIR} ${APRILTAGS_LIBRARY_DIRS})

add_message_files (
  FILES
  april_tag_pos.msg
)

generate_messages (
  DEPENDENCIES
  std_msgs
)

# Declare this project as a catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hello_world
   CATKIN_DEPENDS message_runtime
)


# Add the headers from roscpp
include_directories(include ${catkin_INCLUDE_DIRS}
                            ${OpenCV_INCLUDE_DIRS}
                            ${Eigen_INCLUDE_DIRS}
                            ${APRILTAGS_INCLUDE_DIRS}
                            )

# Define a library target called hello_world
#add_library(hello_world src/hello_world.cpp)
#target_link_libraries(hello_world ${roscpp_LIBRARIES})

add_executable(multi_april_tags_tracker src/multi_april_tags_tracker.cpp
                                        src/multi_apriltags_tracker_demo.cpp
              )   
target_link_libraries(multi_april_tags_tracker ${catkin_LIBRARIES} 
                                               ${OpenCV_LIBRARIES}
                                               ${APRILTAGS_LIBRARY}
                     )

