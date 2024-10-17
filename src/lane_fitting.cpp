#include "velodyne_process/lidar_header.h"

ros::Publisher pub_lane_marker;  // 필터링된 포인트를 발행할 Publisher

void fitting_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // 포인트 클라우드 포인터 초기화
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);  // 필터링된 포인트 클라우드

    // ROS 메시지를 PCL 형식으로 변환
    pcl::PCLPointCloud2* cloud_intermediate = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*input, *cloud_intermediate);
    pcl::fromPCLPointCloud2(*cloud_intermediate, *cloud);

    // 인텐시티 값에 기반하여 필터링
    float intensity_threshold = 40.0;  // 인텐시티 필터링 기준값 설정 (필요에 따라 변경 가능)
    
    for (const auto& point : cloud->points) {
        if (point.intensity > intensity_threshold) {
            filtered_cloud->points.push_back(point);  // 인텐시티가 threshold보다 큰 포인트만 저장
        }
    }

    // 필터링된 포인트 클라우드를 ROS 메시지로 변환
    sensor_msgs::PointCloud2 filtered_lane_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_lane_msg);
    filtered_lane_msg.header = input->header;  // 원래의 헤더를 유지

    // 필터링된 차선 포인트 발행
    pub_lane_marker.publish(filtered_lane_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lane_fitting");
    ros::NodeHandle nh;

    // Subscriber 설정
    ros::Subscriber sub = nh.subscribe("filtered_lane", 1, fitting_callback);

    // Publisher 설정
    pub_lane_marker = nh.advertise<sensor_msgs::PointCloud2>("lane_fitting", 1);

    std::cout << "lane fitting node is running..." << std::endl;

    ros::spin();
    return 0;
}
