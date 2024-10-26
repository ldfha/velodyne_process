#include "velodyne_process/lidar_header.h"

// double ROI_theta(double x, double y);
// using namespace std;

ros::Publisher floor_pub;
ros::Publisher obstacle_pub;
pcl::PassThrough<pcl::PointXYZI> pass;

void roi_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PCLPointCloud2* cloud_intermediate = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    // Convert to PCL data type
    pcl_conversions::toPCL(*input, *cloud_intermediate);

    // Convert PCL::PointCloud2 to PCL::PointCloud<PointXYZI>
    pcl::fromPCLPointCloud2(*cloud_intermediate, cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_p = cloud.makeShared();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_p2 = cloud.makeShared();

    // Apply Passthrough Filter
    pass.setInputCloud (cloud_p);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-2, 1);   // 상하거리
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_p);

    // // Apply Passthrough Filter
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0, 10);  // 앞뒤거리
    pass.setInputCloud (cloud_p);
    pass.filter (*cloud_p);

    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-2.5, 2.5); //좌우거리
    // pass.setFilterLimitsNegative (true);
    pass.setInputCloud (cloud_p);
    pass.filter (*cloud_p);

//라바콘 roi
    
    // Apply Passthrough Filter
    pass.setInputCloud (cloud_p2);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.5, 1);   // 상하거리
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_p2);

    // // Apply Passthrough Filter
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0, 15);  // 앞뒤거리
    pass.setInputCloud (cloud_p2);
    pass.filter (*cloud_p2);

    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-3, 3); //좌우거리
    // pass.setFilterLimitsNegative (true);
    pass.setInputCloud (cloud_p2);
    pass.filter (*cloud_p2);

    floor_pub.publish(*cloud_p);
    obstacle_pub.publish(*cloud_p2);
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "lidar_roi");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("lidar_voxel", 1, roi_callback);
    floor_pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar_floor_roi",1);
    obstacle_pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar_obstacle_roi",1);

    std::cout << "roi complete" << std::endl;

    ros::spin();
}