#include "velodyne_process/lidar_header.h"
#include <velodyne_process/CentroidWithLabel.h>
#include <velodyne_process/CentroidWithLabelArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CentroidColorClassifier {
public:
    CentroidColorClassifier() {
        ros::NodeHandle nh;

        // Parameters
        kernel_size_ = 51;  // Pixel area size (should be odd)
        threshold_ = 0.4;   // Color classification threshold

        // Camera parameters
        camera_params_["left"]["camera_matrix"] = (cv::Mat_<double>(3,3) << 
            5.201364791416012e+02, 0, 3.216949161493492e+02,
            0, 5.318044062657257e+02, 1.977774069204301e+02,
            0, 0, 1);
        camera_params_["left"]["R"] = (cv::Mat_<double>(3,3) << 
            0.394230287631315, -0.918950222405988, -0.010628690141133,
            -0.455883885053441, -0.185506282332136, -0.870492563187010,
            0.797967645749320, 0.348019982120704, -0.492066792602379);
        camera_params_["left"]["T"] = (cv::Mat_<double>(3,1) << 
            0.385472551707401, 0.574845930983101, 1.266383264744131);

        camera_params_["right"]["camera_matrix"] = (cv::Mat_<double>(3,3) << 
            4.327558922371229e+02, 0, 3.248154270949172e+02,
            0, 4.393889014615702e+02, 2.392671823824636e+02,
            0, 0, 1);
        camera_params_["right"]["R"] = (cv::Mat_<double>(3,3) << 
            -0.354126260156503, -0.935152839833323, 0.009151940736245,
            -0.519345917018068, 0.188510112079435, -0.833512900992217,
            0.777736723026649, -0.299921829474731, -0.552424190147661);
        camera_params_["right"]["T"] = (cv::Mat_<double>(3,1) << 
            -0.396962199403921, 0.604084261640270, 0.751677189083849);

        // Subscribers
        centroid_sub_ = nh.subscribe("/centroid_info", 1, &CentroidColorClassifier::centroidCallback, this);
        image_sub_left_ = nh.subscribe("/camera2/usb_cam_2/image_raw", 1, &CentroidColorClassifier::imageCallbackLeft, this);
        image_sub_right_ = nh.subscribe("/camera1/usb_cam_1/image_raw", 1, &CentroidColorClassifier::imageCallbackRight, this);

        // Publishers
        classified_centroid_pub_ = nh.advertise<velodyne_process::CentroidWithLabelArray>("/classified_centroids", 1);
        centroid_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/classified_centroid_markers", 1);
        annotated_image_pub_left_ = nh.advertise<sensor_msgs::Image>("/camera2/image_annotated", 1);
        annotated_image_pub_right_ = nh.advertise<sensor_msgs::Image>("/camera1/image_annotated", 1);

        // Initialize images
        current_image_left_ = cv::Mat();
        current_image_right_ = cv::Mat();
    }

    void centroidCallback(const velodyne_process::CentroidWithLabelArray::ConstPtr& centroid_array) {
        if (!centroid_array || centroid_array->centroids.empty()) {
            ROS_WARN("Received empty centroid array.");
            return;
        }
        lidar_centroids_ = *centroid_array;
        processCentroids();
    }

    void imageCallbackLeft(const sensor_msgs::ImageConstPtr& img_msg) {
        
        try {
            current_image_left_ = cv_bridge::toCvShare(img_msg, "bgr8")->image.clone();
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception (left camera): %s", e.what());
            current_image_left_ = cv::Mat();
        }
    }

    void imageCallbackRight(const sensor_msgs::ImageConstPtr& img_msg) {
        try {
            current_image_right_ = cv_bridge::toCvShare(img_msg, "bgr8")->image.clone();
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception (right camera): %s", e.what());
            current_image_right_ = cv::Mat();
        }
    }

    void processCentroids() {
        if (lidar_centroids_.centroids.empty()) {
            ROS_INFO("No centroid data to process.");
            return;
        }

        if (current_image_left_.empty() || current_image_right_.empty()) {
            ROS_INFO("Waiting for images from both cameras.");
            return;
        }

        velodyne_process::CentroidWithLabelArray classified_centroid_array;
        classified_centroid_array.header.stamp = ros::Time::now();
        classified_centroid_array.header.frame_id = "map";

        visualization_msgs::MarkerArray marker_array;
        int marker_id = 0;

        // Copy images for annotation
        cv::Mat annotated_image_left = current_image_left_.clone();
        cv::Mat annotated_image_right = current_image_right_.clone();

        for (const auto& centroid_msg : lidar_centroids_.centroids) {
            cv::Point3d centroid_point(
                centroid_msg.centroid.x,
                centroid_msg.centroid.y,
                centroid_msg.centroid.z
            );

            std::vector<std::string> labels_detected;

            // Left camera projection and color classification
            cv::Point2i uv_left;
            if (projectPointToImage(centroid_point, "left", uv_left)) {
                std::string label_left = classifyConeColor(annotated_image_left, uv_left.x, uv_left.y);
                if (label_left == "left" || label_left == "right") {
                    labels_detected.push_back(label_left);
                    cv::Scalar color = (label_left == "left") ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 0, 0);
                    cv::circle(annotated_image_left, cv::Point(uv_left.x, uv_left.y + 10), 5, color, 2);
                    cv::rectangle(annotated_image_left,
                        cv::Point(uv_left.x - kernel_size_ / 2, uv_left.y - kernel_size_ / 2),
                        cv::Point(uv_left.x + kernel_size_ / 2, uv_left.y + kernel_size_ / 2),
                        color, 2);
                }
            }

            // Right camera projection and color classification
            cv::Point2i uv_right;
            if (projectPointToImage(centroid_point, "right", uv_right)) {
                std::string label_right = classifyConeColor(annotated_image_right, uv_right.x, uv_right.y);
                if (label_right == "left" || label_right == "right") {
                    labels_detected.push_back(label_right);
                    cv::Scalar color = (label_right == "left") ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 0, 0);
                    cv::circle(annotated_image_right, cv::Point(uv_right.x, uv_right.y + 10), 5, color, 2);
                    cv::rectangle(annotated_image_right,
                        cv::Point(uv_right.x - kernel_size_ / 2, uv_right.y - kernel_size_ / 2),
                        cv::Point(uv_right.x + kernel_size_ / 2, uv_right.y + kernel_size_ / 2),
                        color, 2);
                }
            }

            // Decide the label
            std::string label = "unknown";
            if (std::find(labels_detected.begin(), labels_detected.end(), "left") != labels_detected.end()) {
                label = "left";
            } else if (std::find(labels_detected.begin(), labels_detected.end(), "right") != labels_detected.end()) {
                label = "right";
            }

            // For unclassified centroids, classify based on y-coordinate
            if (label == "unknown") {
                if (centroid_msg.centroid.y < 0) {
                    label = "right";
                    ROS_INFO("Centroid classified as 'right' based on y-coordinate.");
                } else if (centroid_msg.centroid.y > 0) {
                    label = "left";
                    ROS_INFO("Centroid classified as 'left' based on y-coordinate.");
                } else {
                    ROS_INFO("Centroid y-coordinate is zero; label remains 'unknown'.");
                }
            }

            // Publish centroid with label
            if (label == "left" || label == "right") {
                velodyne_process::CentroidWithLabel classified_centroid;
                classified_centroid.centroid = centroid_msg.centroid;
                classified_centroid.label = label;
                classified_centroid_array.centroids.push_back(classified_centroid);

                // Create marker
                visualization_msgs::Marker marker;
                marker.header.frame_id = "velodyne";
                marker.id = marker_id++;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position = centroid_msg.centroid;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.2;
                marker.scale.y = 0.2;
                marker.scale.z = 0.2;
                if (label == "left") {
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                } else if (label == "right") {
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                }
                marker.color.a = 1.0;
                marker.lifetime = ros::Duration(0.3);
                marker_array.markers.push_back(marker);
            } else {
                ROS_INFO("Centroid ignored due to color mismatch or projection failure.");
            }
        }

        // Publish classified centroids and markers
        if (!classified_centroid_array.centroids.empty()) {
            classified_centroid_pub_.publish(classified_centroid_array);
            centroid_marker_pub_.publish(marker_array);
            ROS_INFO("Number of published centroids: %zu", classified_centroid_array.centroids.size());
        } else {
            ROS_INFO("No centroids classified.");
        }

        // Publish annotated images
        sensor_msgs::ImagePtr annotated_msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", annotated_image_left).toImageMsg();
        sensor_msgs::ImagePtr annotated_msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", annotated_image_right).toImageMsg();
        annotated_image_pub_left_.publish(annotated_msg_left);
        annotated_image_pub_right_.publish(annotated_msg_right);

        // Clear data
        lidar_centroids_.centroids.clear();
    }


private:
    bool projectPointToImage(const cv::Point3d& point_lidar, const std::string& camera_side, cv::Point2i& uv) {
        cv::Mat camera_matrix = camera_params_[camera_side]["camera_matrix"];
        cv::Mat R = camera_params_[camera_side]["R"];
        cv::Mat T = camera_params_[camera_side]["T"];

        // Convert point to camera coordinates
        cv::Mat point_camera = R * cv::Mat(point_lidar) + T;

        // Only process points in front of the camera
        if (point_camera.at<double>(2,0) > 0) {
            cv::Mat uvw = camera_matrix * point_camera;
            int u = static_cast<int>(uvw.at<double>(0,0) / uvw.at<double>(2,0));
            int v = static_cast<int>(uvw.at<double>(1,0) / uvw.at<double>(2,0));

            cv::Mat image = (camera_side == "left") ? current_image_left_ : current_image_right_;
            int width = image.cols;
            int height = image.rows;

            if (u >= 0 && u < width && v >= 0 && v < height) {
                uv = cv::Point2i(u, v);
                return true;
            }
        }
        return false;
    }

    std::string classifyConeColor(cv::Mat& image, int u, int v) {
        int half_kernel = kernel_size_ / 2;
        int u_start = std::max(u - half_kernel, 0);
        int u_end = std::min(u + half_kernel, image.cols - 1);
        int v_start = std::max(v, 0);
        int v_end = std::min(v + kernel_size_ + 15, image.rows - 1);

        cv::Mat pixel_region = image(cv::Range(v_start, v_end + 1), cv::Range(u_start, u_end + 1));

        if (pixel_region.empty()) {
            return "unknown";
        }

        // Apply gamma correction
        pixel_region = adjustGamma(pixel_region, 1.2);

        // Apply Gaussian Blur
        cv::Mat pixel_region_blur;
        cv::GaussianBlur(pixel_region, pixel_region_blur, cv::Size(5, 5), 0);

        // Convert to HSV
        cv::Mat hsv_region;
        cv::cvtColor(pixel_region_blur, hsv_region, cv::COLOR_BGR2HSV);

        // Define color ranges
        cv::Scalar yellow_lower(20, 20, 50);
        cv::Scalar yellow_upper(50, 255, 255);
        cv::Scalar blue_lower(80, 20, 30);
        cv::Scalar blue_upper(170, 255, 255);

        // Create masks
        cv::Mat yellow_mask, blue_mask;
        cv::inRange(hsv_region, yellow_lower, yellow_upper, yellow_mask);
        cv::inRange(hsv_region, blue_lower, blue_upper, blue_mask);

        // Calculate color ratios
        int total_pixels = hsv_region.rows * hsv_region.cols;
        double yellow_ratio = static_cast<double>(cv::countNonZero(yellow_mask)) / total_pixels;
        double blue_ratio = static_cast<double>(cv::countNonZero(blue_mask)) / total_pixels;

        // Classification based on threshold
        if (yellow_ratio > threshold_) {
            return "left";
        } else if (blue_ratio > threshold_) {
            return "right";
        } else {
            return "unknown";
        }
    }

    cv::Mat adjustGamma(cv::Mat& image, double gamma) {
        cv::Mat lookUpTable(1, 256, CV_8U);
        uchar* p = lookUpTable.ptr();
        for (int i = 0; i < 256; ++i) {
            p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, 1.0 / gamma) * 255.0);
        }
        cv::Mat res;
        cv::LUT(image, lookUpTable, res);
        return res;
    }

    // ROS NodeHandle
    ros::Subscriber centroid_sub_;
    ros::Subscriber image_sub_left_;
    ros::Subscriber image_sub_right_;
    ros::Publisher classified_centroid_pub_;
    ros::Publisher centroid_marker_pub_;
    ros::Publisher annotated_image_pub_left_;
    ros::Publisher annotated_image_pub_right_;

    // Data storage
    velodyne_process::CentroidWithLabelArray lidar_centroids_;
    cv::Mat current_image_left_;
    cv::Mat current_image_right_;

    // Parameters
    int kernel_size_;
    double threshold_;
    std::map<std::string, std::map<std::string, cv::Mat>> camera_params_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "centroid_color_classifier");
    CentroidColorClassifier classifier;
    ros::spin();
    return 0;
}
