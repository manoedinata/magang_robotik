#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>

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


cv::Mat prev_gray_bev;
std::vector<cv::Point2f> prev_points;
auto last_frame_time = std::chrono::high_resolution_clock::now();

double kecepatan_optical_flow_cms = 0.0;
double total_distance_meters = 0.0;
long current_frame = 0;

const double CM_PER_PIXEL_Y = 10.0 / 100.0; // 10cm di dunia nyata = 100px di BEV

// Parameter Optical Flow (LK)
cv::Size lk_win_size(15, 15);
int lk_max_level = 2;
cv::TermCriteria lk_criteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 0.03);

// Parameter Good Features to Track
int feature_max_corners = 100;
double feature_quality_level = 0.3;
double feature_min_distance = 7;
int feature_block_size = 7;

std::string file = "./FIRA_ARENA.mp4";
// std::string file = "http://10.7.101.143:8080/video";

int main() {
    cv::VideoCapture cap(file);
    if (!cap.isOpened()) {
        std::cerr << "Error opening video file: " << file << std::endl;
        return -1;
    }

    cv::Mat frame;
    while(cap.isOpened())
    {
        cap >> frame;
        if (frame.empty()) {
            std::cout << "End of video." << std::endl;
            break;
        }

        // Convert to 640x360 for consistency
        if(frame.cols != 640 || frame.rows != 360)
            cv::resize(frame, frame, cv::Size(640, 360));

        current_frame++;
        int height = frame.rows;
        int width = frame.cols;

        // Blur image to reduce noise
        cv::Mat frame_blurred;
        cv::GaussianBlur(frame, frame_blurred, cv::Size(5, 5), 0);

        // Convert to HSV color space
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
        cv::imshow("Road Lane (Perspective Canny)", road_lane);

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
    
        // Buat BEV Canny (untuk deteksi)
            cv::Mat bev_image;
        cv::warpPerspective(road_lane, bev_image, M_perspective, cv::Size(width, height));
        cv::imshow("Bird's Eye View (Canny)", bev_image);
        // Buat BEV Berwarna (untuk menggambar)
        cv::Mat image_to_bev;
        cv::warpPerspective(frame, image_to_bev, M_perspective, cv::Size(width, height));

        // Optical Flow
        // Get current time and delta time
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dt_duration = current_time - last_frame_time;
        double dt = dt_duration.count();
        double current_frame_distance = 0.0;

        cv::Mat current_gray_bev = bev_image;

        // Jika kita punya frame lama DAN poin lama, lacak!
        if (prev_points.empty() || prev_points.size() < 20) {
            if (!prev_gray_bev.empty()) {
                cv::goodFeaturesToTrack(prev_gray_bev, prev_points,
                                        feature_max_corners,
                                        feature_quality_level,
                                        feature_min_distance,
                                        cv::Mat(), // no mask
                                        feature_block_size);
            }
        }

        // Lacak fitur jika kita punya semua data
        // Gunakan hanya setiap 30 frame untuk meredakan delay
        if (!prev_gray_bev.empty() && !prev_points.empty() && dt > 0 && (current_frame % 30 == 0)) {
            std::vector<cv::Point2f> new_points;
            std::vector<uchar> status;
            std::vector<float> error;

            cv::calcOpticalFlowPyrLK(
                prev_gray_bev, current_gray_bev, prev_points,
                new_points, status, error, lk_win_size, lk_max_level, lk_criteria
            );

            std::vector<cv::Point2f> good_new;
            std::vector<cv::Point2f> good_old;
            double sum_dy = 0.0;

            for (size_t i = 0; i < status.size(); ++i) {
                if (status[i]) {
                    good_new.push_back(new_points[i]);
                    good_old.push_back(prev_points[i]);
                    sum_dy += (new_points[i].y - prev_points[i].y);
                }
            }

            if (!good_new.empty()) {
                double avg_dy_pixels = sum_dy / good_new.size();
                current_frame_distance = avg_dy_pixels * CM_PER_PIXEL_Y;
                kecepatan_optical_flow_cms = current_frame_distance / dt;
                prev_points = good_new;
            } else {
                prev_points.clear(); // Kehilangan semua titik, cari ulang
            }
        }

        // Akumulasi Jarak
        // Ubah ke meter
        total_distance_meters += current_frame_distance / 100.0;

        // Update state untuk frame berikutnya
        prev_gray_bev = current_gray_bev.clone(); // Penting: .clone() untuk deep copy
        last_frame_time = current_time;

        // Deteksi kontur
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(bev_image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        int min_area_threshold = 40;
        int center_x = width / 2;

        std::vector<cv::Point> left_lane_points;
        std::vector<cv::Point> right_lane_points;
        std::vector<std::vector<cv::Point>> unknown_lane_points;

        for (const auto& cnt : contours) {
            double area = cv::contourArea(cnt);
            if (area < min_area_threshold) {
                continue;
            }

            cv::Moments M = cv::moments(cnt);
            if (M.m00 == 0) {
                continue;
            }
            int cx = static_cast<int>(M.m10 / M.m00);

            // Filter jalur
            if (cx <= (center_x - 175)) {
                left_lane_points.insert(left_lane_points.end(), cnt.begin(), cnt.end());
                continue;
            }
            
            if (cx >= (center_x + 175)) {
                right_lane_points.insert(right_lane_points.end(), cnt.begin(), cnt.end());
                continue;
            }
            
            if (!((center_x - 75) < cx && cx < (center_x + 75))) {
                unknown_lane_points.push_back(cnt);
                continue;
            }
        }

        // Visualisasi
        cv::Mat contour_image = image_to_bev.clone(); // Gambar di BEV berwarna

        cv::Vec4f left_line_params, right_line_params;
        double turn_degree = 0.0;
        std::string road_status = "Lost";

        if (!left_lane_points.empty()) {
            // Bungkus dalam std::vector lain untuk drawContours
            cv::drawContours(contour_image, std::vector<std::vector<cv::Point>>{left_lane_points}, -1, cv::Scalar(255, 0, 0), 3);
            cv::fitLine(left_lane_points, left_line_params, cv::DIST_L2, 0, 0.01, 0.01);
            // y_top di BEV adalah 0, y_bottom adalah 'height'
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
        
        // Format string untuk float
        std::string speed_str = "Speed: " + std::to_string(kecepatan_optical_flow_cms).substr(0, 5) + " cm/s";
        cv::putText(contour_image, speed_str, cv::Point(50, 100),
            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            
        std::string dist_str = "Jarak: " + std::to_string(total_distance_meters).substr(0, 5) + " m";
        cv::putText(contour_image, dist_str, cv::Point(50, 150),
            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

        cv::putText(contour_image, "Road status: " + road_status, cv::Point(50, 200),
            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

        cv::imshow("Contours (BEV)", contour_image);

        // Tekan ESC untuk keluar
        if (cv::waitKey(30) == 27) {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
