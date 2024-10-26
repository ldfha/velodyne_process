#include "velodyne_process/lidar_header.h"
#include <velodyne_process/CentroidWithLabelArray.h>

ros::Publisher pub_cloud;
ros::Publisher pub_boxes;
ros::Publisher pub_cent;
ros::Publisher pub_centroid;
ros::Subscriber sub_lane_center;

std::vector<geometry_msgs::Point> lane_centers;  // 중앙선 포인트들을 저장할 벡터

// 중앙 차선 좌표를 구독하는 콜백 함수
void laneCenterCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
    lane_centers.clear();
    for (const auto& point : msg->points) {
        lane_centers.push_back(point);  // 중앙선 포인트를 저장
    }
}

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

    std::vector<std::vector<int>> run()
    {
        std::vector<std::vector<int>> clusters;
        std::vector<bool> visited(cloud->points.size(), false);
        std::vector<int> labels(cloud->points.size(), -1);  // -1 means unclassified

        int clusterID = 0;

        for (size_t i = 0; i < cloud->points.size(); ++i) {
            if (!visited[i]) {
                visited[i] = true;
                std::vector<int> neighbors = regionQuery(i);

                if (neighbors.size() >= minPts) {
                    std::vector<int> cluster;
                    expandCluster(i, neighbors, cluster, visited, labels, clusterID);
                    if (cluster.size() < 400) { // max voxel cluster size
                      clusters.push_back(cluster);
                      clusterID++;
                      std::cout << "size : " << cluster.size() << std::endl;
                    }
                }
            }
        }
        return clusters;
    }
};

// 주어진 센트로이드가 중앙선에서 1.5미터 이내에 있는지 판단하는 함수
bool isWithinLaneCenter(const geometry_msgs::Point& centroid) {
    for (const auto& center : lane_centers) {
        double distance_to_center = fabs(centroid.y - center.y);  // y축을 기준으로 중앙선과의 거리를 계산
        if (distance_to_center <= 1.5) {
            return true;  // 하나라도 1.5미터 이내에 있으면 True 반환
        }
    }
    return false;  // 중앙선과 1.5미터 내에 있는 포인트가 없으면 False 반환
}

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
    velodyne_process::CentroidWithLabelArray centroidArray;  // CentroidWithLabelArray 메시지 초기화
    int cluster_id = 0;

    for (const auto& cluster_indices : clusters) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        // 각 클러스터마다 색상을 다르게 설정
        uint8_t r = static_cast<uint8_t>(rand() % 256);
        uint8_t g = static_cast<uint8_t>(rand() % 256);
        uint8_t b = static_cast<uint8_t>(rand() % 256);

        for (const auto& index : cluster_indices) {
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

        // 클러스터의 centroid 계산
        geometry_msgs::Point centroid = calculateCentroid(cluster);

        // 중앙 차선에서 양옆으로 1.5미터 이내인지 확인
        if (isWithinLaneCenter(centroid)) {
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
        }
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

    ros::Subscriber sub = nh.subscribe("lidar_outlier", 1, cloud_cb);
    sub_lane_center = nh.subscribe("/lane_center", 1, laneCenterCallback);  // 중앙 차선 구독
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("dbscan_result", 1);
    pub_boxes = nh.advertise<visualization_msgs::MarkerArray>("bounding_boxes", 1);
    pub_cent = nh.advertise<visualization_msgs::MarkerArray>("centroid", 1);
    pub_centroid = nh.advertise<velodyne_process::CentroidWithLabelArray>("/centroid_info", 1);  // Centroid 퍼블리셔 설정

    ros::spin();

    return 0;
}
