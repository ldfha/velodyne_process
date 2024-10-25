#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_process/CentroidWithLabelArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Centroid 정보를 퍼블리시할 퍼블리셔 선언
ros::Publisher pub_centroid;

// 클러스터를 시각화할 퍼블리셔 선언
ros::Publisher pub_cluster_cloud;

// 센트로이드를 시각화할 MarkerArray 퍼블리셔 선언
ros::Publisher pub_centroid_marker;

// Centroid와 레이블을 저장할 구조체 (사용자 정의 메시지)
struct CentroidWithLabel {
    geometry_msgs::Point centroid;
    int label;
};

// DBSCAN 클러스터링 구현 구조체
struct DBSCAN
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    float epsilon;
    int minPts;

    DBSCAN(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, float eps, int min_points)
        : cloud(input_cloud), epsilon(eps), minPts(min_points) {}

    // epsilon 반경 내의 이웃 포인트를 찾는 함수
    std::vector<int> regionQuery(int pointIdx)
    {
        std::vector<int> neighbors;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);
        std::vector<float> pointRadiusSquaredDistance;
        kdtree.radiusSearch(cloud->points[pointIdx], epsilon, neighbors, pointRadiusSquaredDistance);
        return neighbors;
    }

    // 클러스터를 확장하는 함수
    void expandCluster(int pointIdx, std::vector<int>& neighbors, std::vector<int>& cluster, 
                      std::vector<bool>& visited, std::vector<int>& labels, int clusterID)
    {
        cluster.push_back(pointIdx);
        labels[pointIdx] = clusterID;

        for (size_t i = 0; i < neighbors.size(); ++i) {
            int neighborIdx = neighbors[i];

            if (!visited[neighborIdx]) {
                visited[neighborIdx] = true;
                std::vector<int> neighborNeighbors = regionQuery(neighborIdx);

                if (neighborNeighbors.size() >= minPts) {
                    neighbors.insert(neighbors.end(), neighborNeighbors.begin(), neighborNeighbors.end());
                }
            }

            if (labels[neighborIdx] == -1) {
                cluster.push_back(neighborIdx);
                labels[neighborIdx] = clusterID;
            }
        }
    }

    // DBSCAN 클러스터링 실행 함수
    std::vector<std::vector<int>> run()
    {
        std::vector<std::vector<int>> clusters;
        std::vector<bool> visited(cloud->points.size(), false);
        std::vector<int> labels(cloud->points.size(), -1);  // -1은 미분류 상태

        int clusterID = 0;

        for (size_t i = 0; i < cloud->points.size(); ++i) {
            if (!visited[i]) {
                visited[i] = true;
                std::vector<int> neighbors = regionQuery(i);

                if (neighbors.size() >= minPts) {
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

// 클러스터의 센트로이드를 계산하는 함수
geometry_msgs::Point calculateCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster) {
    geometry_msgs::Point centroid;
    double x_sum = 0.0, y_sum = 0.0, z_sum = 0.0;

    for (const auto& point : cluster->points) {
        x_sum += point.x;
        y_sum += point.y;
        z_sum += point.z;
    }

    if (cluster->points.empty()) {
        centroid.x = centroid.y = centroid.z = 0.0;
    } else {
        centroid.x = x_sum / cluster->points.size();
        centroid.y = y_sum / cluster->points.size();
        centroid.z = z_sum / cluster->points.size();
    }

    return centroid;
}

// 포인트 클라우드 데이터를 수신하는 콜백 함수
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // ROS 메시지를 PCL 포인트 클라우드로 변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 입력 클라우드가 비어있는지 확인
    if (cloud->points.empty()) {
        ROS_WARN("수신한 포인트 클라우드가 비어있습니다. 처리를 건너뜁니다.");
        return;
    }

    // DBSCAN 클러스터링 수행
    float epsilon = 0.7; // DBSCAN 거리 임계값 (미터 단위)
    int minPts = 10;      // 클러스터를 형성하기 위한 최소 포인트 개수
    DBSCAN dbscan(cloud, epsilon, minPts);
    std::vector<std::vector<int>> clusters = dbscan.run();

    // CentroidWithLabelArray 메시지 초기화
    velodyne_process::CentroidWithLabelArray centroidArray;
    centroidArray.centroids.reserve(clusters.size());

    // 클러스터를 색상별로 표시할 컬러 포인트 클라우드 생성
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clusters(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Centroid를 시각화할 MarkerArray 초기화
    visualization_msgs::MarkerArray centroidMarkers;
    centroidMarkers.markers.reserve(clusters.size());

    int cluster_id = 0;
    for (const auto& cluster_indices : clusters) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

        // 현재 클러스터에 속한 포인트들을 추출 및 컬러 할당
        uint8_t r = static_cast<uint8_t>(rand() % 256);
        uint8_t g = static_cast<uint8_t>(rand() % 256);
        uint8_t b = static_cast<uint8_t>(rand() % 256);

        for (const auto& index : cluster_indices) {
            pcl::PointXYZRGB colored_point;
            colored_point.x = cloud->points[index].x;
            colored_point.y = cloud->points[index].y;
            colored_point.z = cloud->points[index].z;
            colored_point.r = r;
            colored_point.g = g;
            colored_point.b = b;
            colored_clusters->points.push_back(colored_point);

            cluster->points.push_back(cloud->points[index]);
        }

        // 클러스터의 센트로이드 계산
        geometry_msgs::Point centroid = calculateCentroid(cluster);

        // CentroidWithLabel 메시지에 추가
        velodyne_process::CentroidWithLabel centroidMsg;
        centroidMsg.centroid = centroid;
        centroidMsg.label = cluster_id;  // 클러스터에 고유 ID 부여
        centroidArray.centroids.push_back(centroidMsg);

        // 센트로이드를 시각화할 Marker 설정
        visualization_msgs::Marker centroid_marker;
        centroid_marker.header.frame_id = cloud_msg->header.frame_id;
        centroid_marker.header.stamp = ros::Time::now();
        centroid_marker.ns = "centroids";
        centroid_marker.id = cluster_id;
        centroid_marker.type = visualization_msgs::Marker::SPHERE;
        centroid_marker.action = visualization_msgs::Marker::ADD;
        centroid_marker.pose.position = centroid;
        centroid_marker.pose.orientation.w = 1.0;
        centroid_marker.scale.x = 0.3;
        centroid_marker.scale.y = 0.3;
        centroid_marker.scale.z = 0.3;
        centroid_marker.color.r = 1.0f;
        centroid_marker.color.g = 0.0f;
        centroid_marker.color.b = 0.0f;
        centroid_marker.color.a = 1.0f;
        centroid_marker.lifetime = ros::Duration(0.1);
        centroidMarkers.markers.push_back(centroid_marker);

        cluster_id++;
    }

    // 클러스터 컬러 포인트 클라우드 설정
    colored_clusters->width = colored_clusters->points.size();
    colored_clusters->height = 1;
    colored_clusters->is_dense = true;

    // pcl::PointCloud를 sensor_msgs/PointCloud2로 변환
    sensor_msgs::PointCloud2 cluster_cloud_msg;
    pcl::toROSMsg(*colored_clusters, cluster_cloud_msg);
    cluster_cloud_msg.header = cloud_msg->header;

    // CentroidWithLabelArray 메시지 퍼블리시
    pub_centroid.publish(centroidArray);

    // 클러스터 컬러 포인트 클라우드 퍼블리시
    pub_cluster_cloud.publish(cluster_cloud_msg);

    // 센트로이드 MarkerArray 퍼블리시
    pub_centroid_marker.publish(centroidMarkers);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_dbscan");
    ros::NodeHandle nh;

    // `lidar_obstacle_roi` 토픽을 구독
    ros::Subscriber sub = nh.subscribe("lidar_obstacle_roi", 1, cloudCallback);

    // `centroid_info` 토픽을 퍼블리시
    pub_centroid = nh.advertise<velodyne_process::CentroidWithLabelArray>("/centroid_info", 1);

    // 클러스터 컬러 포인트 클라우드를 퍼블리시할 토픽
    pub_cluster_cloud = nh.advertise<sensor_msgs::PointCloud2>("cluster_cloud", 1);

    // 센트로이드 MarkerArray를 퍼블리시할 토픽
    pub_centroid_marker = nh.advertise<visualization_msgs::MarkerArray>("centroid_markers", 1);

    ROS_INFO("DBSCAN 노드가 초기화되고 실행 중입니다...");
    ros::spin();

    return 0;
}
