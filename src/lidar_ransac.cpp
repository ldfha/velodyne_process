#include "velodyne_process/lidar_header.h"

// Plane model segmentation
// http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation

ros::Publisher pub_non_ground; // 비바닥 포인트를 발행할 Publisher
ros::Publisher pub_ground;     // 바닥 포인트를 발행할 Publisher

void ransac_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // 포인트 클라우드 포인터 초기화
    pcl::PointCloud<pcl::PointXYZI>::Ptr inlierPoints (new pcl::PointCloud<pcl::PointXYZI>),
                                        inlierPoints_neg (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PCLPointCloud2* cloud_intermediate = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    // ROS 메시지를 PCL 형식으로 변환
    pcl_conversions::toPCL(*input, *cloud_intermediate);
    pcl::fromPCLPointCloud2(*cloud_intermediate, cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_p = cloud.makeShared();

    // 평면 모델 계수를 저장할 객체
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // 세그멘테이션 객체 생성 및 설정
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients (true);       // 모델 계수 최적화 활성화
    seg.setModelType (pcl::SACMODEL_PLANE);    // 평면 모델 적용
    seg.setMethodType (pcl::SAC_RANSAC);       // RANSAC 방법 사용
    seg.setMaxIterations (3000);               // 최대 반복 횟수 설정
    seg.setDistanceThreshold (0.08);           // 거리 임계값 설정
    seg.setInputCloud (cloud_p);               // 입력 클라우드 설정
    seg.segment (*inliers, *coefficients);     // 세그멘테이션 실행

    // 인라이어(평면) 포인트 복사
    pcl::copyPointCloud<pcl::PointXYZI>(*cloud_p, *inliers, *inlierPoints);

    // 비인라이어(비평면) 포인트 추출
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud (cloud_p);
    extract.setIndices (inliers);
    extract.setNegative (true); // true로 설정하여 비인라이어 추출
    extract.filter (*inlierPoints_neg);

    // PCL 포인트 클라우드를 ROS 메시지로 변환
    sensor_msgs::PointCloud2 non_ground_msg;
    pcl::toROSMsg(*inlierPoints_neg, non_ground_msg);
    non_ground_msg.header = input->header;

    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*inlierPoints, ground_msg);
    ground_msg.header = input->header;

    // 비바닥 포인트 발행
    pub_non_ground.publish(non_ground_msg);

    // 바닥 포인트 발행
    pub_ground.publish(ground_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_ransac");
    ros::NodeHandle nh;

    // Subscriber 설정
    ros::Subscriber sub = nh.subscribe("lidar_voxel", 1, ransac_callback);

    // Publisher 설정
    pub_non_ground = nh.advertise<sensor_msgs::PointCloud2>("lidar_ransac", 1);
    pub_ground = nh.advertise<sensor_msgs::PointCloud2>("ground_point", 1);

    std::cout << "RANSAC node is running..." << std::endl;

    ros::spin();
}
