#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <rubber_cone_mission/CentroidWithLabelArray.h>  // 정의한 메시지 헤더 포함

ros::Publisher pub_cloud;
ros::Publisher pub_boxes;
ros::Publisher pub_cent;
ros::Publisher pub_centroid;  // Centroid 퍼블리셔 추가

struct Box {
  float x_min, y_min, z_min;
  float x_max, y_max, z_max;
};

// 바운딩 박스 생성 함수
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

// DBSCAN 클러스터링 구현
struct DBSCAN
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  float epsilon;
  int minPts;
  
  DBSCAN(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, float eps, int min_points)
    : cloud(input_cloud), epsilon(eps), minPts(min_points) {}

  std::vector<int> regionQuery(int pointIdx)
  {
    std::vector<int> neighbors;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<float> pointRadiusSquaredDistance;
    kdtree.radiusSearch(cloud->points[pointIdx], epsilon, neighbors, pointRadiusSquaredDistance);
    return neighbors;
  }

  void expandCluster(int pointIdx, std::vector<int>& neighbors, std::vector<int>& cluster, std::vector<bool>& visited, std::vector<int>& labels, int clusterID)
  {
    cluster.push_back(pointIdx);
    labels[pointIdx] = clusterID;

    for (size_t i = 0; i < neighbors.size(); ++i)
    {
      int neighborIdx = neighbors[i];

      if (!visited[neighborIdx])
      {
        visited[neighborIdx] = true;
        std::vector<int> neighborNeighbors = regionQuery(neighborIdx);

        if (neighborNeighbors.size() >= minPts)
        {
          neighbors.insert(neighbors.end(), neighborNeighbors.begin(), neighborNeighbors.end());
        }
      }

      if (labels[neighborIdx] == -1)
      {
        cluster.push_back(neighborIdx);
        labels[neighborIdx] = clusterID;
      }
    }
  }

  std::vector<std::vector<int>> run()
  {
    std::vector<std::vector<int>> clusters;
    std::vector<bool> visited(cloud->points.size(), false);
    std::vector<int> labels(cloud->points.size(), -1);  // -1 means unclassified

    int clusterID = 0;

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      if (!visited[i])
      {
        visited[i] = true;
        std::vector<int> neighbors = regionQuery(i);

        if (neighbors.size() >= minPts)
        {
          std::vector<int> cluster;
          expandCluster(i, neighbors, cluster, visited, labels, clusterID);
          clusters.push_back(cluster);
          clusterID++;
        }
      }
    }
    return clusters;
  }
};

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // 포인트 클라우드를 변환
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // 입력 클라우드가 비어있는지 확인
  if (cloud->points.empty()) {
    ROS_WARN("Input cloud is empty, skipping this callback.");
    return;
  }

  // DBSCAN 클러스터링 수행
  float epsilon = 0.7; // DBSCAN 거리 임계값
  int minPts = 10;      // 최소 포인트 개수
  DBSCAN dbscan(cloud, epsilon, minPts);
  std::vector<std::vector<int>> clusters = dbscan.run();

  // 클러스터링 결과를 Colored PointCloud로 변환
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  visualization_msgs::MarkerArray boxArray;
  visualization_msgs::MarkerArray centArray;
  rubber_cone_mission::CentroidWithLabelArray centroidArray;  // CentroidWithLabelArray 메시지 초기화
  int cluster_id = 0;

  for (const auto& cluster_indices : clusters)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    // 각 클러스터마다 색상을 다르게 설정
    uint8_t r = static_cast<uint8_t>(rand() % 256);
    uint8_t g = static_cast<uint8_t>(rand() % 256);
    uint8_t b = static_cast<uint8_t>(rand() % 256);

    for (const auto& index : cluster_indices)
    {
      pcl::PointXYZRGB point;
      point.x = cloud->points[index].x;
      point.y = cloud->points[index].y;
      point.z = cloud->points[index].z;
      point.r = r;
      point.g = g;
      point.b = b;
      result_cloud->points.push_back(point);
      cluster->points.push_back(cloud->points[index]); // 클러스터에 해당 포인트 추가
    }

    // 바운딩 박스 생성
    Box box = createBoundingBox(cluster);

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
    geometry_msgs::Point centroid = calculateCentroid(cluster);

    // CentroidWithLabel 메시지에 추가
    rubber_cone_mission::CentroidWithLabel centroidMsg;
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
  }

  result_cloud->width = result_cloud->points.size();
  result_cloud->height = 1;
  result_cloud->is_dense = true;

  // pcl::PointCloud를 sensor_msgs/PointCloud2로 변환
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*result_cloud, output);
  output.header.frame_id = "velodyne";

  // 변환된 메시지 publish
  pub_cloud.publish(output);
  pub_boxes.publish(boxArray);
  pub_cent.publish(centArray);
  pub_centroid.publish(centroidArray);  // Centroid 정보 퍼블리시
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_dbscan");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("lidar_ransac", 1, cloud_cb);
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("dbscan_result", 1);
  pub_boxes = nh.advertise<visualization_msgs::MarkerArray>("bounding_boxes", 1);
  pub_cent = nh.advertise<visualization_msgs::MarkerArray>("centroid", 1);
  pub_centroid = nh.advertise<rubber_cone_mission::CentroidWithLabelArray>("/centroid_info", 1);  // Centroid 퍼블리셔 설정

  ros::spin();

  return 0;
}
