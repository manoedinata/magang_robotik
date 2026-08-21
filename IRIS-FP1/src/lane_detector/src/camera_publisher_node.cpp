#include <chrono>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"

class CameraPublisherNode : public rclcpp::Node
{
public:
    CameraPublisherNode() : Node("camera_publisher_node")
    {
        RCLCPP_INFO(this->get_logger(), "Camera Publisher Node partially started.");
    }

    void init_publishers_and_subscribers() {
        // Gunakan image_transport untuk mem-publish gambar (praktik terbaik)
        // image_transport sebenarnya tidak hanya membuat satu topik. Dia membuat satu set lengkap topik:
        // /camera/image_raw (untuk gambar raw, tipe sensor_msgs/msg/Image)
        // /camera/image_raw/compressed (untuk gambar terkompresi, tipe sensor_msgs/msg/CompressedImage)
        // /camera/image_raw/theora
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        raw_image_pub_ = it_->advertise("camera/image_raw", 1); 

        // Buat timer untuk membaca frame
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(
            33ms, std::bind(&CameraPublisherNode::timer_callback, this));
      
        // Buka sumber video
        // std::string file = "./assets/FIRA_ARENA.mp4";
        // std::string file = "http://10.7.101.143:8080/video";
        this->declare_parameter<std::string>("video_source", "./assets/FIRA_ARENA.mp4");
        std::string file;
        this->get_parameter("video_source", file);
        RCLCPP_INFO(this->get_logger(), "Using video source: %s", file.c_str());

        cap_.open(file);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Error opening video file: %s", file.c_str());
            rclcpp::shutdown();
        }
            RCLCPP_INFO(this->get_logger(), "Camera Publisher Node started. Publishing to /camera/image_raw");
    }

private:
    // ROS 2
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher raw_image_pub_;

    // OpenCV
    cv::VideoCapture cap_;

    void timer_callback()
    {
        cv::Mat frame;
        cap_.read(frame);
        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "End of video. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // Resize ke 640x360
        if(frame.cols != 640 || frame.rows != 360)
            cv::resize(frame, frame, cv::Size(640, 360));

        // Konversi cv::Mat ke pesan ROS
        auto header = std_msgs::msg::Header();
        header.stamp = this->get_clock()->now();
        header.frame_id = "camera_frame";
        
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            header, "bgr8", frame).toImageMsg();

        // Publish pesan
        raw_image_pub_.publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CameraPublisherNode>();
    node->init_publishers_and_subscribers();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
