// waypoint_extractor.cpp

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <rubber_cone_mission/CentroidWithLabelArray.h>
#include <erp_driver/erpCmdMsg.h>
#include <erp_driver/erpStatusMsg.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <math.h>
#include <vector>
#include <algorithm>

class WaypointExtractor
{
public:
    WaypointExtractor()
    {
        // Initialize subscribers
        centroid_sub_ = nh_.subscribe("/classified_centroids", 10, &WaypointExtractor::centroidCallback, this);
        status_sub_ = nh_.subscribe("/erp42_status", 10, &WaypointExtractor::statusCallback, this);

        // Initialize publishers
        path_pub_ = nh_.advertise<nav_msgs::Path>("/waypoint_path", 10);
        waypoint_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/waypoint_markers", 10);

        // Initialize parameters
        centroid_threshold_ = 5.0;    // Threshold distance
        default_offset_ = 1.0;         // Offset when only one side centroid is detected
        steering_angle_deg_ = 0.0;     // Default steering angle
        x_match_threshold_ = 0.3;      // X-coordinate matching threshold

        // Initialize current steering angle
        current_steering_angle_ = 0.0;
    }

    void centroidCallback(const rubber_cone_mission::CentroidWithLabelArray::ConstPtr& msg)
    {
        std::vector<geometry_msgs::Point> left_centroids;
        std::vector<geometry_msgs::Point> right_centroids;

        for (const auto& centroid_with_label : msg->centroids)
        {
            geometry_msgs::Point point = centroid_with_label.centroid;
            std::string label = centroid_with_label.label;

            // Calculate distance from the vehicle
            double distance = sqrt(pow(point.x, 2) + pow(point.y, 2));

            // Apply threshold filtering
            if (distance > centroid_threshold_)
                continue;

            if (label == "left")
                left_centroids.push_back(point);
            else if (label == "right")
                right_centroids.push_back(point);
        }

        // Sort centroids based on x-coordinate (forward direction)
        std::sort(left_centroids.begin(), left_centroids.end(),
                  [](const geometry_msgs::Point& a, const geometry_msgs::Point& b) { return a.x < b.x; });
        std::sort(right_centroids.begin(), right_centroids.end(),
                  [](const geometry_msgs::Point& a, const geometry_msgs::Point& b) { return a.x < b.x; });

        // Calculate waypoints
        std::vector<geometry_msgs::Point> waypoints = calculateWaypoints(left_centroids, right_centroids);

        // Publish waypoints
        publishWaypointMarkers(waypoints);
        publishWaypoints(waypoints);
    }

    void statusCallback(const erp_driver::erpStatusMsg::ConstPtr& msg)
    {
        // Update current steering angle
        // Assuming msg->steer ranges from -2000 to 2000 corresponding to -30 to 30 degrees
        current_steering_angle_ = (static_cast<double>(msg->steer) / 2000.0) * 30.0;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber centroid_sub_;
    ros::Subscriber status_sub_;
    ros::Publisher path_pub_;
    ros::Publisher waypoint_marker_pub_;

    double centroid_threshold_;
    double default_offset_;
    double steering_angle_deg_;
    double x_match_threshold_;

    double current_steering_angle_;

    std::vector<geometry_msgs::Point> calculateWaypoints(const std::vector<geometry_msgs::Point>& left_centroids,
                                                        const std::vector<geometry_msgs::Point>& right_centroids)
    {
        std::vector<geometry_msgs::Point> waypoints;

        if (!left_centroids.empty() && !right_centroids.empty())
        {
            // Both left and right centroids exist
            waypoints = calculateMidpoints(left_centroids, right_centroids);
        }
        else if (!left_centroids.empty())
        {
            ROS_WARN("Only left centroids detected.");
            waypoints = calculateOffsetWaypoints(left_centroids, "left");
        }
        else if (!right_centroids.empty())
        {
            ROS_WARN("Only right centroids detected.");
            waypoints = calculateOffsetWaypoints(right_centroids, "right");
        }
        else
        {
            ROS_WARN("No centroids detected.");
        }

        return waypoints;
    }

    std::vector<geometry_msgs::Point> calculateMidpoints(std::vector<geometry_msgs::Point> left_centroids,
                                                        std::vector<geometry_msgs::Point> right_centroids)
    {
        std::vector<geometry_msgs::Point> waypoints;
        std::vector<int> right_indices;
        for (size_t i = 0; i < right_centroids.size(); ++i)
            right_indices.push_back(i);

        for (size_t left_idx = 0; left_idx < left_centroids.size(); ++left_idx)
        {
            geometry_msgs::Point left_centroid = left_centroids[left_idx];

            // Find the closest right centroid in x
            double min_x_diff = x_match_threshold_;
            int matching_idx = -1;

            for (size_t i = 0; i < right_indices.size(); ++i)
            {
                int idx = right_indices[i];
                double x_diff = fabs(right_centroids[idx].x - left_centroid.x);
                if (x_diff < min_x_diff)
                {
                    min_x_diff = x_diff;
                    matching_idx = idx;
                }
            }

            if (matching_idx != -1)
            {
                geometry_msgs::Point right_centroid = right_centroids[matching_idx];

                // Calculate midpoint
                geometry_msgs::Point midpoint;
                midpoint.x = (left_centroid.x + right_centroid.x) / 2.0;
                midpoint.y = (left_centroid.y + right_centroid.y) / 2.0;
                midpoint.z = 0.0;

                waypoints.push_back(midpoint);

                // Remove the matched right centroid
                right_indices.erase(std::remove(right_indices.begin(), right_indices.end(), matching_idx), right_indices.end());
            }
            else
            {
                // No matching right centroid found, apply offset
                waypoints.push_back(applyOffset(left_centroid, "left"));
            }
        }

        // Handle remaining right centroids
        for (size_t i = 0; i < right_indices.size(); ++i)
        {
            int idx = right_indices[i];
            waypoints.push_back(applyOffset(right_centroids[idx], "right"));
        }

        return waypoints;
    }

    std::vector<geometry_msgs::Point> calculateOffsetWaypoints(const std::vector<geometry_msgs::Point>& centroids, const std::string& side)
    {
        std::vector<geometry_msgs::Point> waypoints;
        for (const auto& centroid : centroids)
        {
            waypoints.push_back(applyOffset(centroid, side));
        }
        return waypoints;
    }

    geometry_msgs::Point applyOffset(const geometry_msgs::Point& centroid, const std::string& side)
    {
        geometry_msgs::Point waypoint;
        waypoint.x = centroid.x;
        waypoint.y = centroid.y;

        if (side == "left")
        {
            waypoint.y -= default_offset_;
        }
        else if (side == "right")
        {
            waypoint.y += default_offset_;
        }

        waypoint.z = 0.0;
        return waypoint;
    }

    void publishWaypoints(const std::vector<geometry_msgs::Point>& waypoints)
    {
        if (waypoints.empty())
            return;

        // Sort waypoints based on x-coordinate
        std::vector<geometry_msgs::Point> sorted_waypoints = waypoints;
        std::sort(sorted_waypoints.begin(), sorted_waypoints.end(),
                  [](const geometry_msgs::Point& a, const geometry_msgs::Point& b) { return a.x < b.x; });

        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "velodyne"; // Change if necessary

        for (const auto& waypoint : sorted_waypoints)
        {
            geometry_msgs::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position = waypoint;
            pose.pose.orientation.w = 1.0; // Default orientation
            path_msg.poses.push_back(pose);
        }

        path_pub_.publish(path_msg);
        ROS_INFO("Published %lu waypoints.", waypoints.size());
    }

    void publishWaypointMarkers(const std::vector<geometry_msgs::Point>& waypoints)
    {
        visualization_msgs::MarkerArray marker_array;

        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "velodyne";
            marker.header.stamp = ros::Time::now();
            marker.ns = "waypoints";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = waypoints[i];
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;
            marker.lifetime = ros::Duration(0.3);

            marker_array.markers.push_back(marker);
        }

        waypoint_marker_pub_.publish(marker_array);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_extractor");
    WaypointExtractor extractor;
    ros::spin();
    return 0;
}
