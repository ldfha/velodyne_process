#include "velodyne_process/lidar_header.h"
#include <velodyne_process/CentroidWithLabelArray.h>
#include <velodyne_process/ClusterBounds.h>
#include <velodyne_process/ClusterInfo.h>
#include <std_msgs/Header.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cstdlib>
#include <ctime>

ros::Publisher pub_cloud;
ros::Publisher pub_boxes;
ros::Publisher pub_cent;
ros::Publisher pub_centroid;
ros::Publisher pub_all_centroids;
ros::Publisher pub_all_centroid_markers;
ros::Publisher pub_cluster_info;
ros::Subscriber sub_lane_center;

std::vector<geometry_msgs::Point> lane_centers;

void laneCenterCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
    lane_centers.clear();
    for (const auto& point : msg->points) {
        lane_centers.push_back(point);
    }
}

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

bool isWithinLaneCenter(const geometry_msgs::Point& centroid) {
    for (const auto& center : lane_centers) {
        double distance_to_center = fabs(centroid.y - center.y);
        if (distance_to_center <= 1.5) {
            return true;
        }
    }
    return false;
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

    std::vector<std::vector<int>> run()
    {
        std::vector<std::vector<int>> clusters;
        std::vector<bool> visited(cloud->points.size(), false);
        std::vector<int> labels(cloud->points.size(), -1);  // -1은 미분류를 의미

        int clusterID = 0;

        for (size_t i = 0; i < cloud->points.size(); ++i) {
            if (!visited[i]) {
                visited[i] = true;
                std::vector<int> neighbors = regionQuery(i);

                if (neighbors.size() >= minPts) {
                    std::vector<int> cluster;
                    expandCluster(i, neighbors, cluster, visited, labels, clusterID);
                    if (cluster.size() < 400) {  // 최대 클러스터 크기 제한
                        clusters.push_back(cluster);
                        clusterID++;
                        std::cout << "Cluster size: " << cluster.size() << std::endl;
                    }
                }
            }
        }
        return clusters;
    }
};

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if (cloud->points.empty()) {
        ROS_WARN("Input cloud is empty, skipping this callback.");
        return;
    }

    float epsilon = 0.7;
    int minPts = 10;
    DBSCAN dbscan(cloud, epsilon, minPts);
    std::vector<std::vector<int>> clusters = dbscan.run();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    visualization_msgs::MarkerArray boxArray;
    visualization_msgs::MarkerArray centArray;
    visualization_msgs::MarkerArray allCentroidMarkers;
    velodyne_process::CentroidWithLabelArray centroidArray;
    velodyne_process::CentroidWithLabelArray allCentroidArray;
    velodyne_process::ClusterInfo clusterInfoMsg;
    clusterInfoMsg.header.stamp = ros::Time::now();
    clusterInfoMsg.header.frame_id = "velodyne";
    int cluster_id = 0;

    for (const auto& cluster_indices : clusters) {
        int current_cluster_id = cluster_id;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
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
            cluster->points.push_back(cloud->points[index]);
        }

        geometry_msgs::Point centroid = calculateCentroid(cluster);

        // 모든 센트로이드를 allCentroidArray에 추가
        velodyne_process::CentroidWithLabel allCentroidMsg;
        allCentroidMsg.centroid = centroid;
        allCentroidMsg.label = current_cluster_id;
        allCentroidArray.centroids.push_back(allCentroidMsg);

        // 모든 센트로이드에 대한 마커 생성
        visualization_msgs::Marker all_centroid_marker;
        all_centroid_marker.header.frame_id = "velodyne";
        all_centroid_marker.header.stamp = ros::Time::now();
        all_centroid_marker.ns = "all_cluster_centroids";
        all_centroid_marker.id = current_cluster_id;
        all_centroid_marker.type = visualization_msgs::Marker::SPHERE;
        all_centroid_marker.action = visualization_msgs::Marker::ADD;
        all_centroid_marker.pose.position = centroid;
        all_centroid_marker.scale.x = 0.2;
        all_centroid_marker.scale.y = 0.2;
        all_centroid_marker.scale.z = 0.2;
        all_centroid_marker.color.r = 0.0f;
        all_centroid_marker.color.g = 0.0f;
        all_centroid_marker.color.b = 1.0f;
        all_centroid_marker.color.a = 1.0f;
        all_centroid_marker.lifetime = ros::Duration(1.0);
        allCentroidMarkers.markers.push_back(all_centroid_marker);

        if (isWithinLaneCenter(centroid)) {
            Box box = createBoundingBox(cluster);

            // 바운딩 박스 마커 생성
            visualization_msgs::Marker marker;
            marker.header.frame_id = "velodyne";
            marker.header.stamp = ros::Time::now();
            marker.ns = "cluster_boxes";
            marker.id = current_cluster_id;
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
            marker.lifetime = ros::Duration(1.0);
            boxArray.markers.push_back(marker);

            // 센트로이드와 레이블 추가
            velodyne_process::CentroidWithLabel centroidMsg;
            centroidMsg.centroid = centroid;
            centroidMsg.label = current_cluster_id;
            centroidArray.centroids.push_back(centroidMsg);

            // 센트로이드 마커 생성
            visualization_msgs::Marker centroid_marker;
            centroid_marker.header.frame_id = "velodyne";
            centroid_marker.header.stamp = ros::Time::now();
            centroid_marker.ns = "cluster_centroids";
            centroid_marker.id = current_cluster_id;
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
            centroid_marker.lifetime = ros::Duration(1.0);
            centArray.markers.push_back(centroid_marker);

            // y_max와 y_min 계산하여 clusterInfoMsg에 추가
            velodyne_process::ClusterBounds bounds;
            bounds.y_max = box.y_max;
            bounds.y_min = box.y_min;
            clusterInfoMsg.clusters.push_back(bounds);
        }

        cluster_id++;
    }

    result_cloud->width = result_cloud->points.size();
    result_cloud->height = 1;
    result_cloud->is_dense = true;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*result_cloud, output);
    output.header.frame_id = "velodyne";

    pub_cloud.publish(output);
    pub_boxes.publish(boxArray);
    pub_cent.publish(centArray);
    pub_centroid.publish(centroidArray);
    pub_all_centroids.publish(allCentroidArray);
    pub_all_centroid_markers.publish(allCentroidMarkers);
    pub_cluster_info.publish(clusterInfoMsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_dbscan");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("lidar_outlier", 1, cloud_cb);
    sub_lane_center = nh.subscribe("/lane_center", 1, laneCenterCallback);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("dbscan_result", 1);
    pub_boxes = nh.advertise<visualization_msgs::MarkerArray>("bounding_boxes", 1);
    pub_cent = nh.advertise<visualization_msgs::MarkerArray>("centroid", 1);
    pub_centroid = nh.advertise<velodyne_process::CentroidWithLabelArray>("/centroid_info", 1);
    pub_all_centroids = nh.advertise<velodyne_process::CentroidWithLabelArray>("/all_centroid_info", 1);
    pub_all_centroid_markers = nh.advertise<visualization_msgs::MarkerArray>("/all_centroid_markers", 1);
    pub_cluster_info = nh.advertise<velodyne_process::ClusterInfo>("/cluster_y", 1);

    ros::spin();

    return 0;
}
