#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"

#include "std_msgs/msg/float32.hpp" // Float32
#include "std_msgs/msg/string.hpp"  // String

void draw_fit_line(cv::Mat& img, const cv::Vec4f& line_params, int y_top, int y_bottom, 
                 const cv::Scalar& color, int thickness) {
    if (line_params[1] == 0) return; // Hindari pembagian dengan nol (vy == 0)

    double vx = line_params[0];
    double vy = line_params[1];
    double x = line_params[2];
    double y = line_params[3];

    // xtarget = x + (y_target- y) * (vx / vy)
    int x1 = cvRound(x + (y_bottom - y) * vx / vy);
    int x2 = cvRound(x + (y_top - y) * vx / vy);
    int y1 = y_bottom;
    int y2 = y_top;

    cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), color, thickness);
}

double get_line_angle(const cv::Vec4f& line_params) {
    if (line_params[0] == 0 && line_params[1] == 0) return 0.0;

    double vx = line_params[0];
    double vy = line_params[1];

    // Standarisasi: vy selalu menunjuk ke ATAS (negatif y)
    if (vy > 0) {
        vx = -vx;
        vy = -vy;
    }

    // atan2 mengembalikan sudut antara sumbu x positif dan vektor (vx, vy)
    double angle_rad = std::atan2(vy, vx);

    // Konversi radian ke derajat
    return angle_rad * 180.0 / M_PI;
}

class LaneDetectorNode : public rclcpp::Node
{
public:
    LaneDetectorNode() : Node("lane_detector_node") {}

    void init_publishers_and_subscribers()
    {
        // Gunakan image_transport untuk mem-publish gambar (praktik terbaik)
        // image_transport sebenarnya tidak hanya membuat satu topik. Dia membuat satu set lengkap topik:
        // /camera/image_processed (untuk gambar raw, tipe sensor_msgs/msg/Image)
        // /camera/image_processed/compressed (untuk gambar terkompresi, tipe sensor_msgs/msg/CompressedImage)
        // /camera/image_processed/theora
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        bev_image_pub_ = it_->advertise("camera/image_processed_result", 1);
        bev_canny_pub_ = it_->advertise("camera/image_processed", 1);

        // Publisher untuk data telemetri
        turn_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("vehicle/steering", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("vehicle/status", 10);
        obstacle_status_pub_ = this->create_publisher<std_msgs::msg::String>("obstacle/status", 10);

        // Subscribe ke topik yang di-publish oleh camera_publisher_node
        camera_sub_ = it_->subscribe("camera/image_raw", 1, 
            std::bind(&LaneDetectorNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Lane Detector Node started. Subscribing to /camera/image_raw");
    }

private:
    // ROS 2
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber camera_sub_;
    image_transport::Publisher bev_image_pub_;
    image_transport::Publisher bev_canny_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr turn_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_status_pub_;

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        cv::Mat frame;

        // Konversi pesan ROS ke cv::Mat
        try {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        int height = frame.rows;
        int width = frame.cols;

        // Blur image untuk mengurangi noise
        cv::Mat frame_blurred;
        cv::GaussianBlur(frame, frame_blurred, cv::Size(5, 5), 0);

        // Convert ke HSV color space
        cv::Mat hsv;
        cv::cvtColor(frame_blurred, hsv, cv::COLOR_BGR2HSV);

         // Get road lane by inverting the green color mask
        cv::Mat mask, inverted_mask;
        cv::Scalar lower_green(33, 50, 50);
        cv::Scalar upper_green(85, 255, 255);
        cv::inRange(hsv, lower_green, upper_green, mask);
        cv::bitwise_not(mask, inverted_mask);

        // Get lane lines by converting to grayscale and applying Canny edge detection
        cv::Mat gray, edges;
        cv::cvtColor(frame_blurred, gray, cv::COLOR_BGR2GRAY);
        cv::Canny(gray, edges, 170, 255);

        // Combine the inverted green mask and Canny edges
        cv::Mat road_lane;
        cv::bitwise_and(inverted_mask, edges, road_lane);

        // BIRD'S EYE VIEW (BEV)
        cv::Point2f top_left(static_cast<float>(width * 0.2), 140.0f);
        cv::Point2f top_right(static_cast<float>(width * 0.8), 140.0f);
        cv::Point2f bottom_left(0.0f, static_cast<float>(height));
        cv::Point2f bottom_right(static_cast<float>(width), static_cast<float>(height));
            cv::Point2f src_view[4] = {top_left, top_right, bottom_right, bottom_left};
        cv::Point2f dst_view[4] = {
                cv::Point2f(0.0f, 0.0f),
                cv::Point2f(static_cast<float>(width), 0.0f),
                cv::Point2f(static_cast<float>(width), static_cast<float>(height)),
                cv::Point2f(0.0f, static_cast<float>(height))
        };

        cv::Mat M_perspective = cv::getPerspectiveTransform(src_view, dst_view);

        // Buat BEV Canny (untuk deteksi & odometri)
        cv::Mat bev_image;
        cv::warpPerspective(road_lane, bev_image, M_perspective, cv::Size(width, height));

        // Buat BEV Berwarna (untuk menggambar visualisasi)
        cv::Mat image_to_bev;
        cv::warpPerspective(frame, image_to_bev, M_perspective, cv::Size(width, height));

        //
        // Optical flow moved to odometry_node
        //

        // Deteksi kontur
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(bev_image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        int min_area_threshold = 40;
        int center_x = width / 2;
        std::vector<cv::Point> left_lane_points, right_lane_points;
        std::vector<std::vector<cv::Point>> unknown_lane_points;

        for (const auto& cnt : contours) {
            double area = cv::contourArea(cnt);
            if (area < min_area_threshold) continue;
            cv::Moments M = cv::moments(cnt);
            if (M.m00 == 0) continue;
            int cx = static_cast<int>(M.m10 / M.m00);

            if (cx <= (center_x - 175)) left_lane_points.insert(left_lane_points.end(), cnt.begin(), cnt.end());
            else if (cx >= (center_x + 175)) right_lane_points.insert(right_lane_points.end(), cnt.begin(), cnt.end());
            else if (!((center_x - 75) < cx && cx < (center_x + 75))) unknown_lane_points.push_back(cnt);
        }

          // Visualisasi
         cv::Mat contour_image = image_to_bev.clone(); // Gambar di BEV berwarna

         cv::Vec4f left_line_params, right_line_params;
         double turn_degree = 0.0;
         std::string road_status = "Lost";

         if (!left_lane_points.empty()) {
                  cv::drawContours(contour_image, std::vector<std::vector<cv::Point>>{left_lane_points}, -1, cv::Scalar(255, 0, 0), 3);
                  cv::fitLine(left_lane_points, left_line_params, cv::DIST_L2, 0, 0.01, 0.01);
                  draw_fit_line(contour_image, left_line_params, 0, height, cv::Scalar(255, 0, 0), 3);
         }

         if (!right_lane_points.empty()) {
                  cv::drawContours(contour_image, std::vector<std::vector<cv::Point>>{right_lane_points}, -1, cv::Scalar(0, 0, 255), 3);
                  cv::fitLine(right_lane_points, right_line_params, cv::DIST_L2, 0, 0.01, 0.01);
                  draw_fit_line(contour_image, right_line_params, 0, height, cv::Scalar(0, 0, 255), 3);
         }

         if (!unknown_lane_points.empty()) {
                  cv::drawContours(contour_image, unknown_lane_points, -1, cv::Scalar(0, 255, 255), 3);
         }

         // Hitung Sudut
         std::string turn_degree_msg;
         if (!left_lane_points.empty() && !right_lane_points.empty()) {
                  double left_angle = get_line_angle(left_line_params);
                  double right_angle = get_line_angle(right_line_params);
                  double avg_angle = (left_angle + right_angle) / 2.0;
                  turn_degree = avg_angle - (-90.0);
                  turn_degree_msg = "Turn Degree: " + std::to_string(turn_degree).substr(0, 5) + " deg";
                  road_status = "Detected";
         } else {
                  turn_degree_msg = "Turn degree is unknown";
                  if (!left_lane_points.empty() || !right_lane_points.empty()) {
                          road_status = "Partial";
                  }
         }

         // Tampilkan Telemetry
         cv::putText(contour_image, turn_degree_msg, cv::Point(50, 50),
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

         cv::putText(contour_image, "Road status: " + road_status, cv::Point(50, 200),
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

        // PUBLISH DATA KE ROS 2

        // Buat header dengan timestamp
        auto header = std_msgs::msg::Header();
        header.stamp = msg->header.stamp; // Teruskan timestamp dari gambar asli
        header.frame_id = "bev_frame";
        
        // 1. Publish Gambar Canny BEV (untuk odometry_node)
        sensor_msgs::msg::Image::SharedPtr bev_msg = cv_bridge::CvImage(
            header, "bgr8", contour_image).toImageMsg(); //
            bev_image_pub_.publish(bev_msg);
            
        // 2. Publish Gambar (BEV Visualisasi)
        sensor_msgs::msg::Image::SharedPtr canny_msg = cv_bridge::CvImage(
            header, "mono8", bev_image).toImageMsg(); // bev_image adalah mono8
        bev_canny_pub_.publish(canny_msg);

        // 3. Publish Telemetri Sudut
        auto angle_msg = std_msgs::msg::Float32();
        angle_msg.data = turn_degree;
        turn_angle_pub_->publish(angle_msg);

        // 4. Publish Status Jalan
        auto status_msg = std_msgs::msg::String();
        status_msg.data = road_status;
        status_pub_->publish(status_msg);

        // 5. Publish Status Obstacle
        auto obstacle_msg = std_msgs::msg::String();
        if (!unknown_lane_points.empty()) {
            obstacle_msg.data = "Detected";
        } else {
            obstacle_msg.data = "None";
        }
        obstacle_status_pub_->publish(obstacle_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LaneDetectorNode>();
    node->init_publishers_and_subscribers();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
