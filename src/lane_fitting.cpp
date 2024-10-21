#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <velodyne_process/CentroidWithLabelArray.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pub_boxes;
ros::Publisher pub_cent;
ros::Publisher pub_centroid;

// 바운딩 박스 생성 함수
struct Box {
  float x_min, y_min, z_min;
  float x_max, y_max, z_max;
};

Box createBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster) {
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);
  
  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;
  
  return box;
}

// 클러스터의 centroid를 계산하는 함수
geometry_msgs::Point calculateCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster) {
  geometry_msgs::Point centroid;
  double x_sum = 0.0, y_sum = 0.0, z_sum = 0.0;

  for (const auto& point : cluster->points) {
    x_sum += point.x;
    y_sum += point.y;
    z_sum += point.z;
  }

  centroid.x = x_sum / cluster->points.size();
  centroid.y = y_sum / cluster->points.size();
  centroid.z = z_sum / cluster->points.size();

  return centroid;
}

void process_clusters(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  if (cloud->points.empty()) {
    ROS_WARN("Input cloud is empty, skipping this callback.");
    return;
  }

  // 클러스터에서 바운딩 박스와 센트로이드를 생성
  visualization_msgs::MarkerArray boxArray;
  visualization_msgs::MarkerArray centArray;
  velodyne_process::CentroidWithLabelArray centroidArray;
  int cluster_id = 0;

  // 예시로 클러스터가 있다고 가정합니다.
  // 실제로는 클러스터 정보가 들어오는 다른 노드와 연동할 필요가 있습니다.
  pcl::PointCloud<pcl::PointXYZ>::Ptr example_cluster(new pcl::PointCloud<pcl::PointXYZ>);
  // 여기에 클러스터링된 점들(example_cluster)이 들어간다고 가정.

  // 바운딩 박스 생성
  Box box = createBoundingBox(example_cluster);

  // 바운딩 박스 시각화를 위한 Marker 설정
  visualization_msgs::Marker marker;
  marker.header.frame_id = "velodyne";
  marker.header.stamp = ros::Time::now();
  marker.ns = "cluster_boxes";
  marker.id = cluster_id++;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = (box.x_min + box.x_max) / 2.0;
  marker.pose.position.y = (box.y_min + box.y_max) / 2.0;
  marker.pose.position.z = (box.z_min + box.z_max) / 2.0;
  marker.scale.x = box.x_max - box.x_min;
  marker.scale.y = box.y_max - box.y_min;
  marker.scale.z = box.z_max - box.z_min;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5f;
  marker.lifetime = ros::Duration(0.1);
  boxArray.markers.push_back(marker);

  // 클러스터의 centroid 계산 및 추가
  geometry_msgs::Point centroid = calculateCentroid(example_cluster);

  // CentroidWithLabel 메시지에 추가
  velodyne_process::CentroidWithLabel centroidMsg;
  centroidMsg.centroid = centroid;
  centroidMsg.label = cluster_id;  // 각 클러스터에 ID 부여
  centroidArray.centroids.push_back(centroidMsg);

  // 시각화를 위한 Marker 추가 (centroid)
  visualization_msgs::Marker centroid_marker;
  centroid_marker.header.frame_id = "velodyne";
  centroid_marker.header.stamp = ros::Time::now();
  centroid_marker.ns = "cluster_centroids";
  centroid_marker.id = cluster_id++;
  centroid_marker.type = visualization_msgs::Marker::SPHERE;
  centroid_marker.action = visualization_msgs::Marker::ADD;
  centroid_marker.pose.position = centroid;
  centroid_marker.scale.x = 0.2;
  centroid_marker.scale.y = 0.2;
  centroid_marker.scale.z = 0.2;
  centroid_marker.color.r = 1.0f;
  centroid_marker.color.g = 0.0f;
  centroid_marker.color.b = 0.0f;
  centroid_marker.color.a = 1.0f;
  centroid_marker.lifetime = ros::Duration(0.1);
  centArray.markers.push_back(centroid_marker);

  // Publish 결과
  pub_boxes.publish(boxArray);
  pub_cent.publish(centArray);
  pub_centroid.publish(centroidArray);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bounding_box_and_centroid_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("dbscan_result", 1, process_clusters);
  pub_boxes = nh.advertise<visualization_msgs::MarkerArray>("bounding_boxes", 1);
  pub_cent = nh.advertise<visualization_msgs::MarkerArray>("centroid", 1);
  pub_centroid = nh.advertise<velodyne_process::CentroidWithLabelArray>("/centroid_info", 1);

  ros::spin();

  return 0;
}
