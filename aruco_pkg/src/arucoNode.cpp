#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ArucoMarkerProcessor : public rclcpp::Node {
public:
    ArucoMarkerProcessor() : Node("aruco_marker_processor") {
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&ArucoMarkerProcessor::imageCallback, this, std::placeholders::_1));
        marker_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("detected_markers", 10);
        marker_detected_publisher_ = this->create_publisher<std_msgs::msg::Bool>("marker_detected", 10);
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    }
private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::aruco::detectMarkers(cv_ptr->image, dictionary_, markerCorners, markerIds);
        geometry_msgs::msg::PoseArray marker_poses;
        marker_poses.header.stamp = this->get_clock()->now();
        marker_poses.header.frame_id = "camera_link";

        for (size_t i = 0; i < markerIds.size(); i++) {
            geometry_msgs::msg::Pose marker_pose;
            cv::Point2f center(0.f, 0.f);
            for (const auto& corner : markerCorners[i]) {
                center += corner;
            }
            center /= 4.0;
            marker_pose.position.x = center.x;
            marker_pose.position.y = center.y;
            marker_pose.position.z = 0;
            marker_pose.orientation.w = 1.0;

            marker_poses.poses.push_back(marker_pose);
        }

        marker_publisher_->publish(marker_poses);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr marker_detected_publisher_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoMarkerProcessor>());
    rclcpp::shutdown();
    return 0;
}
