#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"

#include "std_msgs/msg/float32.hpp"

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode() : Node("odometry_node") {}

    void init_publishers_and_subscribers() {
        // Inisialisasi Publisher
        speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("vehicle/speed", 10);
        distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("vehicle/distance", 10);

        // Inisialisasi Subscriber
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        bev_canny_sub_ = it_->subscribe("camera/image_processed", 1, 
            std::bind(&OdometryNode::image_callback, this, std::placeholders::_1));

        last_frame_time_ = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Odometry Node started. Subscribing to /camera/image_processed");
    }

private:
    // ROS 2
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber bev_canny_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;

    // Variabel state untuk Optical Flow
    cv::Mat prev_gray_bev_;
    std::vector<cv::Point2f> prev_points_;
    rclcpp::Time last_frame_time_;

    float kecepatan_optical_flow_cms_ = 0.0f;
    float total_distance_meters_ = 0.0f;
    long current_frame_ = 0;

    // --- Konstanta & Parameter ---
    const double CM_PER_PIXEL_Y = 10.0 / 100.0; // 10cm di dunia nyata = 100px di BEV

    // Parameter Optical Flow (LK)
    cv::Size lk_win_size_ = cv::Size(15, 15);
    int lk_max_level_ = 2;
    cv::TermCriteria lk_criteria_ = cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 0.03);

    // Parameter Good Features to Track
    int feature_max_corners_ = 100;
    double feature_quality_level_ = 0.3;
    double feature_min_distance_ = 7;
    int feature_block_size_ = 7;

    // Callback untuk Subscriber gambar BEV Canny
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        cv::Mat current_gray_bev;

        // Konversi pesan ROS ke cv::Mat
        try {
            current_gray_bev = cv_bridge::toCvShare(msg, "mono8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        current_frame_++;

        // Optical Flow
        rclcpp::Time current_time = msg->header.stamp;
        rclcpp::Duration dt_duration = current_time - last_frame_time_;
        double dt = dt_duration.seconds();
        double current_frame_distance = 0.0;

        if (prev_points_.empty() || prev_points_.size() < 20) {
                if (!prev_gray_bev_.empty()) {
                        cv::goodFeaturesToTrack(prev_gray_bev_, prev_points_,
                                            feature_max_corners_,
                                            feature_quality_level_,
                                            feature_min_distance_,
                                            cv::Mat(), // no mask
                                            feature_block_size_);
                }
        }

        if (!prev_gray_bev_.empty() && !prev_points_.empty() && dt > 0 && (current_frame_ % 30 == 0)) {
                std::vector<cv::Point2f> new_points;
                std::vector<uchar> status;
                std::vector<float> error;

                cv::calcOpticalFlowPyrLK(
                        prev_gray_bev_, current_gray_bev, prev_points_,
                        new_points, status, error, lk_win_size_, lk_max_level_, lk_criteria_
                );

                std::vector<cv::Point2f> good_new;
                std::vector<cv::Point2f> good_old;
                double sum_dy = 0.0;
                
                for (std::vector<unsigned char>::size_type i = 0; i < status.size(); ++i) {
                        if (status[i]) {
                        good_new.push_back(new_points[i]);
                        good_old.push_back(prev_points_[i]);
                        sum_dy += (new_points[i].y - prev_points_[i].y);
                        }
                }

                if (!good_new.empty()) {
                        double avg_dy_pixels = sum_dy / good_new.size();
                        current_frame_distance = avg_dy_pixels * CM_PER_PIXEL_Y;
                        kecepatan_optical_flow_cms_ = current_frame_distance / dt;
                        prev_points_ = good_new;
                } else {
                        prev_points_.clear(); // Kehilangan semua titik, cari ulang
                }
        }

        // total_distance_meters_ += current_frame_distance / 100.0;
        total_distance_meters_ += static_cast<float>(current_frame_distance / 100.0);

        prev_gray_bev_ = current_gray_bev.clone();
        last_frame_time_ = current_time;

        // --- PUBLISH DATA KE ROS 2 ---
        auto speed_msg = std_msgs::msg::Float32();
        speed_msg.data = kecepatan_optical_flow_cms_;
        speed_pub_->publish(speed_msg);

        auto dist_msg = std_msgs::msg::Float32();
        dist_msg.data = total_distance_meters_;
        distance_pub_->publish(dist_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OdometryNode>();
    node->init_publishers_and_subscribers();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
