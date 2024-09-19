cmake_minimum_required(VERSION 2.8.3)
project(localization)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE Debug)
# add_compile_options(-g -O0)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
  cv_bridge
  pcl_conversions
  sensor_msgs
  nav_msgs
  message_generation
  visualization_msgs
)

set(PCL_DIR "/home/yuanyuan/Documents/pcl_version/share/pcl-1.14")
find_package(PCL 1.14.0 REQUIRED)
find_package(yaml-cpp REQUIRED)
# LIO-SAM
find_package(OpenMP REQUIRED)
# find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)
add_service_files(
  DIRECTORY srv
  FILES
  saveMap.srv
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)
catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)



#add project
set(ALL_TARGET_LIBRARIES "")
# include(cmake/global_definition.cmake)
# include(cmake/glog.cmake)
# include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/geographic.cmake)
include_directories(
	include/
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)
include_directories(/home/yuanyuan/Documents/homework/devel/include/localization)
# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

file(GLOB_RECURSE ALL_SRCS "src/*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})
# add_executable(localization_slam src/test_frame_node.cpp ${ALL_SRCS})
# target_link_libraries(localization_slam ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} glog::glog ${PCL_LIBRARIES} yaml-cpp )

# add_executable(front_end_node src/front_end_node.cpp ${ALL_SRCS})
# ## 添加依赖关系
# add_dependencies(front_end_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS} saveMap_gencpp)
# target_link_libraries(front_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} glog::glog ${PCL_LIBRARIES} yaml-cpp )


add_executable(test_frame_node src/apps/test_frame_node.cpp ${ALL_SRCS})
target_link_libraries(test_frame_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} glog::glog ${PCL_LIBRARIES} yaml-cpp)

add_executable(data_pretreat_node src/apps/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(data_pretreat_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} glog::glog ${PCL_LIBRARIES} yaml-cpp)

add_executable(front_end_node src/apps/front_end_node.cpp ${ALL_SRCS})
add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(front_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} glog::glog ${PCL_LIBRARIES} yaml-cpp)

add_executable(back_end_node src/apps/back_end_node.cpp ${ALL_SRCS})
add_dependencies(back_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(back_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} glog::glog ${PCL_LIBRARIES} yaml-cpp)

add_executable(viewer_node src/apps/viewer_node.cpp ${ALL_SRCS})
add_dependencies(viewer_node ${catkin_EXPORTED_TARGETS} saveMap_gencpp)
target_link_libraries(viewer_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} glog::glog ${PCL_LIBRARIES} yaml-cpp)











# # link directories
# link_directories(
# 	include
# 	${PCL_LIBRARY_DIRS}
#   ${OpenCV_LIBRARY_DIRS}
#   ${GTSAM_LIBRARY_DIRS}
# )





###########
## Build ##
###########

# Range Image Projection
# add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
# add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Feature Association
# add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
# add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
# add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
# add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
# target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
# target_link_libraries(${PROJECT_NAME}_mapOptmization Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam)

# # IMU Preintegration
# add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
# target_link_libraries(${PROJECT_NAME}_imuPreintegration Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)
