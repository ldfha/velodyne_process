cmake_minimum_required(VERSION 3.0.2)
project(velodyne_process)

## C++14 사용
add_compile_options(-std=c++14)

## catkin 매크로와 라이브러리 찾기
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  velodyne_pcl
  rubber_cone_mission
  message_generation
  geometry_msgs
  cv_bridge
  image_transport
  nav_msgs
  visualization_msgs
  erp_driver
)

## 메시지 파일 등록
add_message_files(
  FILES
  CentroidWithLabelArray.msg
  CentroidWithLabel.msg
)

## 메시지 생성
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## PCL 찾기
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros pcl_conversions velodyne_pcl message_runtime
  DEPENDS PCL mlpack
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}  # PCL 헤더 추가
  ${PROJECT_SOURCE_DIR}/velodyne_process/include
)


## PCL 라이브러리 링크 디렉토리 추가
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## 실행 파일 선언
add_executable(lidar_ransac_node src/lidar_ransac.cpp)
add_executable(lidar_voxel_node src/lidar_voxel.cpp)
add_executable(lidar_roi_node src/lidar_roi.cpp)
add_executable(lidar_outlier_node src/lidar_outlier.cpp)
add_executable(lidar_dbscan_node src/lidar_dbscan.cpp)
add_executable(lidar_dbscan_cone_node src/lidar_dbscan_cone.cpp)
add_executable(lane_filtering_node src/lane_filtering.cpp)
add_executable(centroid_pub_color_node src/centroid_pub_color.cpp)
add_executable(waypoint_publisher_node src/waypoint_publisher.cpp)

## 라이브러리 링크
target_link_libraries(lidar_ransac_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}  # PCL 라이브러리 추가
)

target_link_libraries(lidar_voxel_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}  # PCL 라이브러리 추가
)

target_link_libraries(lidar_roi_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}  # PCL 라이브러리 추가
)

target_link_libraries(lidar_outlier_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}  # PCL 라이브러리 추가
)

target_link_libraries(lane_filtering_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}  # PCL 라이브러리 추가
)

target_link_libraries(lidar_dbscan_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}  # PCL 라이브러리 추가
)

target_link_libraries(lidar_dbscan_cone_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}  # PCL 라이브러리 추가
)

target_link_libraries(centroid_pub_color_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}  # OpenCV 라이브러리 추가
)

target_link_libraries(waypoint_publisher_node
  ${catkin_LIBRARIES}
)


## 빌드 순서를 위한 의존성 추가
add_dependencies(lidar_ransac_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lidar_voxel_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lidar_roi_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lidar_outlier_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})  
add_dependencies(lidar_dbscan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lidar_dbscan_cone_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})  
add_dependencies(lane_filtering_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(centroid_pub_color_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(waypoint_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
