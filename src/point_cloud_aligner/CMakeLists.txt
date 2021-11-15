cmake_minimum_required(VERSION 3.10.0)
project(point_cloud_aligner)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O3")


find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

find_package(catkin REQUIRED COMPONENTS
      cv_bridge
      image_transport
      roscpp
      sensor_msgs
      std_msgs
      pcl_conversions
      pcl_ros
      tf
      tf_conversions
      tf2_eigen
      tf2
      rospy
      message_generation
      geometry_msgs
      rosbag
      nav_msgs
      dynamic_reconfigure
      )
      


generate_dynamic_reconfigure_options(
        cfg/Trans6.cfg
)


catkin_package() # dont forget this for launch file or rosrun

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
        ${CERES_INCLUDE_DIRS}
)

add_executable(point_cloud_aligner_exe
        src/main.cpp
        src/point_cloud_aligner.cpp
        src/point_to_plane_solver.cpp
        src/point_cloud_handler.cpp
        src/transform_handler.cpp
)

add_executable(keyboard_publisher_exe
  src/keyboard_publisher.cpp)


target_link_libraries(point_cloud_aligner_exe
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  Ceres::ceres
)

target_link_libraries(keyboard_publisher_exe
  ${catkin_LIBRARIES}
  )

add_dependencies(point_cloud_aligner_exe
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencfg
        )

add_dependencies(keyboard_publisher_exe
  ${catkin_EXPORTED_TARGETS}
  )


add_definitions(${PCL_DEFINITIONS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )





