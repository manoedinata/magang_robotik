#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <nlohmann/json.hpp>

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
    LaneDetectorNode() : Node("lane_detector_node") {
        this->declare_parameter<double>("canny.low_threshold", 170.0);
        this->declare_parameter<double>("canny.high_threshold", 255.0);
        this->declare_parameter<double>("bev.top_y_left_percent", 0.45);      // 45% down from the top (was 220.0f)
        this->declare_parameter<double>("bev.top_y_right_percent", 0.45);      // 45% down from the top (was 220.0f)
        this->declare_parameter<double>("bev.top_x_left_percent", 0.3);  // 40% from the left
        this->declare_parameter<double>("bev.top_x_right_percent", 0.9); // 60% from the left
        this->declare_parameter<double>("bev.bottom_x_left_percent", 0.0); // 10% from the left
        this->declare_parameter<double>("bev.bottom_x_right_percent", 2.0); // 90% from the left
        this->declare_parameter<double>("cntr.left_lane_limit_center", 175.0);

        // Baca nilai default tersebut ke variabel anggota
        // Kita kunci mutex saat inisialisasi untuk keamanan
        std::lock_guard<std::mutex> lock(param_mutex_);
        canny_low_threshold_ = this->get_parameter("canny.low_threshold").as_double();
        canny_high_threshold_ = this->get_parameter("canny.high_threshold").as_double();
        bev_top_y_left_percent_ = this->get_parameter("bev.top_y_left_percent").as_double();
        bev_top_y_right_percent_ = this->get_parameter("bev.top_y_right_percent").as_double();
        bev_top_x_left_percent_ = this->get_parameter("bev.top_x_left_percent").as_double();
        bev_top_x_right_percent_ = this->get_parameter("bev.top_x_right_percent").as_double();
        bev_bottom_x_left_percent_ = this->get_parameter("bev.bottom_x_left_percent").as_double();
        bev_bottom_x_right_percent_ = this->get_parameter("bev.bottom_x_right_percent").as_double();
        cntr_left_lane_limit_center_ = this->get_parameter("cntr.left_lane_limit_center").as_double();

        RCLCPP_INFO(this->get_logger(), "Configurable BEV parameters declared.");
    }

    void init_publishers_and_subscribers()
    {
        // Gunakan image_transport untuk mem-publish gambar (praktik terbaik)
        // image_transport sebenarnya tidak hanya membuat satu topik. Dia membuat satu set lengkap topik:
        // /camera/image_processed (untuk gambar raw, tipe sensor_msgs/msg/Image)
        // /camera/image_processed/compressed (untuk gambar terkompresi, tipe sensor_msgs/msg/CompressedImage)
        // /camera/image_processed/theora
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        bev_canny_pub_ = it_->advertise("camera/image_processed", 1);

        // Publisher untuk data telemetri
        turn_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("vehicle/steering", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("vehicle/status", 10);
        combined_pub_ = this->create_publisher<std_msgs::msg::String>("vehicle/telemetry", 10); // Combined topic for turn angle and status in JSON format

        // Subscriber untuk konfigurasi parameter (jika diperlukan)
        config_sub_ = this->create_subscription<std_msgs::msg::String>(
            "threshold_config",
            10,
            std::bind(&LaneDetectorNode::config_callback, this, std::placeholders::_1)
        );

        // Subscribe ke topik yang di-publish oleh camera_publisher_node
        camera_sub_ = it_->subscribe("camera/image_raw", 1, 
            std::bind(&LaneDetectorNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Lane Detector Node started. Subscribing to /camera/image_raw");
    }

private:
    // ROS 2
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber camera_sub_;
    image_transport::Publisher bev_canny_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr turn_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr combined_pub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_sub_;
    // Variabel anggota untuk menyimpan nilai parameter
    // Lindungi dengan mutex untuk thread-safety
    std::mutex param_mutex_;
    double canny_low_threshold_;
    double canny_high_threshold_;
    double bev_top_y_left_percent_;
    double bev_top_y_right_percent_;
    double bev_top_x_left_percent_;
    double bev_top_x_right_percent_;
    double bev_bottom_x_left_percent_;
    double bev_bottom_x_right_percent_;
    double cntr_left_lane_limit_center_;

    void config_callback(const std_msgs::msg::String::ConstSharedPtr msg)
    {
        try {
            auto json_data = nlohmann::json::parse(msg->data);
            RCLCPP_INFO(this->get_logger(), "Received new config: %s", msg->data.c_str());

            // Kunci mutex sebelum mengubah nilai
            std::lock_guard<std::mutex> lock(param_mutex_);

            // Gunakan .value() untuk mengambil nilai jika ada, 
            // atau gunakan nilai yang sudah ada jika tidak.
            bev_top_y_left_percent_ = json_data.value("bev.top_y_left_percent", bev_top_y_left_percent_);
            bev_top_y_right_percent_ = json_data.value("bev.top_y_right_percent", bev_top_y_right_percent_);
            bev_top_x_left_percent_ = json_data.value("bev.top_x_left_percent", bev_top_x_left_percent_);
            bev_top_x_right_percent_ = json_data.value("bev.top_x_right_percent", bev_top_x_right_percent_);
            bev_bottom_x_left_percent_ = json_data.value("bev.bottom_x_left_percent", bev_bottom_x_left_percent_);
            bev_bottom_x_right_percent_ = json_data.value("bev.bottom_x_right_percent", bev_bottom_x_right_percent_);
            canny_low_threshold_ = json_data.value("canny.low_threshold", canny_low_threshold_);
            canny_high_threshold_ = json_data.value("canny.high_threshold", canny_high_threshold_);
            cntr_left_lane_limit_center_ = json_data.value("cntr.left_lane_limit_center", cntr_left_lane_limit_center_);

        } catch (const nlohmann::json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse config JSON: %s", e.what());
        }
    }

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

        // Salin nilai parameter saat ini secara atomik
        // Ini adalah cara cepat dan aman untuk mendapatkan nilai-nilai
        // tanpa mengunci mutex untuk waktu yang lama.
        double canny_low, canny_high;
        double top_y_left_p, top_y_right_p, top_x_left_p, top_x_right_p, bottom_x_left_p, bottom_x_right_p;
        {
            std::lock_guard<std::mutex> lock(param_mutex_);
            canny_low = canny_low_threshold_;
            canny_high = canny_high_threshold_;
            top_y_left_p = bev_top_y_left_percent_;
            top_y_right_p = bev_top_y_right_percent_;
            top_x_left_p = bev_top_x_left_percent_;
            top_x_right_p = bev_top_x_right_percent_;
            bottom_x_left_p = bev_bottom_x_left_percent_;
            bottom_x_right_p = bev_bottom_x_right_percent_;
        }

        // Get lane lines by converting to grayscale and applying Canny edge detection
        cv::Mat gray, edges;
        cv::cvtColor(frame_blurred, gray, cv::COLOR_BGR2GRAY);
        cv::Canny(
            gray,
            edges,
            canny_low,
            canny_high
        );

        // Combine the inverted green mask and Canny edges
        cv::Mat road_lane;
        cv::bitwise_and(inverted_mask, edges, road_lane);

        // BIRD'S EYE VIEW (BEV)
        // Get dynamic BEV parameters from ROS 2 parameters
        cv::Point2f top_left(
            static_cast<float>(width * top_x_left_p),
            static_cast<float>(height * top_y_left_p)
        );

        cv::Point2f top_right(
            static_cast<float>(width * top_x_right_p),
            static_cast<float>(height * top_y_right_p)
        );

        cv::Point2f bottom_left(
            static_cast<float>(width * bottom_x_left_p),
            static_cast<float>(height) // Bottom Y is always max height
        );

        cv::Point2f bottom_right(
            static_cast<float>(width * bottom_x_right_p),
            static_cast<float>(height) // Bottom Y is always max height
        );

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

        // Deteksi kontur
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(bev_image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        int min_area_threshold = 40;
        int center_x = width / 2;
        std::vector<cv::Point> left_lane_points;

        double left_lane_limit_center;
        {
            std::lock_guard<std::mutex> lock(param_mutex_);
            left_lane_limit_center = cntr_left_lane_limit_center_;
        }

        for (const auto& cnt : contours) {
            double area = cv::contourArea(cnt);
            if (area < min_area_threshold) continue;
            cv::Moments M = cv::moments(cnt);
            if (M.m00 == 0) continue;
            int cx = static_cast<int>(M.m10 / M.m00);

            if (cx <= (center_x - left_lane_limit_center)) left_lane_points.insert(left_lane_points.end(), cnt.begin(), cnt.end());
            if(left_lane_points.empty()) {
                // If no left lane points found far left, try directly to center
                if (cx <= (center_x)) left_lane_points.insert(left_lane_points.end(), cnt.begin(), cnt.end());
            }
        }

         // Hitung Sudut
        cv::Vec4f left_line_params;
        double turn_degree = 0.0;
        std::string road_status = "Lost";
        std::string turn_degree_msg;

        // 1. LAKUKAN FITTING GARIS KE TITIK-TITIK LANE KIRI
        if (!left_lane_points.empty()) {
            // FIT_LINE_RANSAC is generally robust for lane lines
            cv::fitLine(left_lane_points, left_line_params, cv::DIST_L2, 0, 0.01, 0.01);
            
            // Asumsi: Di labirin ini hanya ada satu garis (kiri) untuk diikuti.
            // Jika ada garis kanan, Anda harus menghitungnya juga.
            
            // 2. HITUNG SUDUT GARIS YANG TELAH DI-FIT
            double line_angle = get_line_angle(left_line_params);
            
            // 3. HITUNG TURN DEGREE BERDASARKAN DEVIASI SUDUT
            // Sudut 90 derajat (atau -90 derajat) mewakili garis lurus vertikal.
            // Kita anggap 0.0 deg adalah sudut lurus ke depan.
            // Jika get_line_angle mengembalikan -90.0 deg (vertikal ke atas),
            // maka turn_degree = -90.0 - (-90.0) = 0.0 (tidak ada belokan).
            // Jika get_line_angle mengembalikan -85.0 deg (miring ke kanan),
            // maka turn_degree = -85.0 - (-90.0) = 5.0 (belok ke kanan).
            // Jika get_line_angle mengembalikan -95.0 deg (miring ke kiri),
            // maka turn_degree = -95.0 - (-90.0) = -5.0 (belok ke kiri).
            
            // Menggunakan line_angle saja mengukur *arah* jalan.
            // Untuk kontrol yang baik, Anda juga perlu *offset* dari tengah.
            // Namun, untuk algoritma sederhana, hanya sudut bisa cukup.
            turn_degree = line_angle - (-90.0); // Perbedaan dari -90 derajat (vertikal)
            
            road_status = "Detected";
            turn_degree_msg = "Turn Degree: " + std::to_string(turn_degree).substr(0, 5) + " deg";

            // Opsi Tambahan: Gambar garis yang di-fit ke BEV image untuk debugging visual
            draw_fit_line(bev_image, left_line_params, 0, height, cv::Scalar(255), 2);

        } else {
            // Jika tidak ada garis, gunakan sudut 0 (lurus) dan berikan status "Lost"
            turn_degree = 0.0; 
            turn_degree_msg = "Turn degree is unknown";
            road_status = "Lost";
        }

        // PUBLISH DATA KE ROS 2

        // Buat header dengan timestamp
        auto header = std_msgs::msg::Header();
        header.stamp = msg->header.stamp; // Teruskan timestamp dari gambar asli
        header.frame_id = "bev_frame";

        // 1. Publish Gambar Canny BEV (untuk odometry_node)
        sensor_msgs::msg::Image::SharedPtr canny_msg = cv_bridge::CvImage(
            header, "mono8", bev_image).toImageMsg(); // bev_image adalah mono8
        bev_canny_pub_.publish(canny_msg);

        // 2. Publish Telemetri Sudut
        auto angle_msg = std_msgs::msg::Float32();
        angle_msg.data = turn_degree;
        turn_angle_pub_->publish(angle_msg);

        // 3. Publish Status Jalan
        auto status_msg = std_msgs::msg::String();
        status_msg.data = road_status;
        status_pub_->publish(status_msg);

        // 4. Publish Combined Telemetry in JSON format
        nlohmann::json telemetry_json;
        telemetry_json["turn_angle"] = turn_degree;
        telemetry_json["road_status"] = road_status;

        auto combined_msg = std_msgs::msg::String();
        combined_msg.data = telemetry_json.dump();
        combined_pub_->publish(combined_msg);
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
