import cv2
import numpy as np
import time
import math

def draw_fit_line(img, line_params, y_top, y_bottom, color, thickness):
    """
    Menggambar garis hasil 'fitLine' ke sebuah gambar.
    """
    if line_params is None or len(line_params) < 4:
        return

    # Ekstrak nilai dari array NumPy
    vx = line_params[0][0]
    vy = line_params[1][0]
    x = line_params[2][0]
    y = line_params[3][0]

    # sanity check: hindari pembagian dengan nol
    if vy == 0: return

    x1 = int(x + (y_bottom - y) * vx / vy)
    y1 = y_bottom
    x2 = int(x + (y_top - y) * vx / vy)
    y2 = y_top

    cv2.line(img, (x1, y1), (x2, y2), color, thickness)

def get_line_angle(line_params):
    """
    Menghitung sudut garis dalam derajat dari -90 (lurus)
    """
    if line_params is None or len(line_params) < 4:
        return 0.0  # Kembalikan float

    # Ekstrak nilai dari array NumPy
    vx = line_params[0][0]
    vy = line_params[1][0]

    # Standarisasi: vy selalu menunjuk ke ATAS (negatif y)
    if vy > 0:
        vx = -vx
        vy = -vy

    # Hitung sudut (hasilnya float)
    angle_deg = np.degrees(np.arctan2(vy, vx))
    return angle_deg

prev_gray_bev = None
prev_points = None
last_frame_time = time.time()
kecepatan_optical_flow_cms = 0.0
total_distance_meters = 0.0

# Jarak (cm) di dunia nyata / Jarak (pixel) di gambar BEV
CM_PER_PIXEL_Y = 10.0 / 100.0

# Optical Flow parameters (Lucas-Kanade)
lk_params = dict(
    winSize = (15,15), # Ukuran window pelacakan
    maxLevel = 2,       # Level piramida
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
)

# Good Features to Track parameters
feature_params = dict(
    maxCorners = 100,    # Jumlah maks fitur
    qualityLevel = 0.3,  # Kualitas minimum
    minDistance = 7,     # Jarak minimum antar fitur
    blockSize = 7
)

# Count current frame
current_frame = 0

# file = "./FIRA_ARENA.mp4"
file = "http://10.7.101.143:8080/video"
# file = "http://10.7.101.192:8080/video"
# file = "http://10.137.196.11:8080/video"
cap = cv2.VideoCapture(file)
if not cap.isOpened():
    raise FileNotFoundError(f"Video {file} not found or cannot be opened.")

while True:
    ret, image = cap.read()
    if not ret:
        break

    # Force to 640x480 for faster processing
    # height, width = image.shape[:2]
    # if height >= 480 or width >= 640:
    #     image = cv2.resize(image, (int(640), int(480)), interpolation=cv2.INTER_AREA)

    # Count current frame
    current_frame += 1

    height, width = image.shape[:2]

    # Blur image to reduce noise
    image = cv2.GaussianBlur(image, (5, 5), 0)

    # Convert to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # cv2.imshow("HSV Image", hsv)

    # Get road lane by inverting the green color mask
    lower_green = (33, 50, 50)
    upper_green = (85, 255, 255)
    mask = cv2.inRange(hsv, lower_green, upper_green)
    inverted_mask = cv2.bitwise_not(mask)
    # cv2.imshow("Inverted Green Mask", inverted_mask)

    # Get lane lines by converting to grayscale and applying Canny edge detection
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 170, 255)
    # cv2.imshow("Canny Edges", edges)

    # Combine the inverted green mask and Canny edges
    road_lane = cv2.bitwise_and(inverted_mask, edges)
    cv2.imshow("Road Lane", road_lane)

    # ROI to focus on the road area
    # edges = road that has been canny edge detected
    mask_roi = np.zeros_like(edges)

    # UNTUK 1280x720 VIDEO
    bottom_left  = (0, height)        # Kiri bawah
    bottom_right = (width, height)       # Kanan bawah
    top_left     = (int(width * 0.2), 280)
    top_right    = (int(width * 0.8), 280)
    y_top_roi = top_right[1]

    # UNTUK 640x480 VIDEO (BELUM STABIL)
    # bottom_left  = (0, height)
    # bottom_right = (width, height)
    # top_left     = (130, 160)
    # top_right    = (600, 200)
    # y_top_roi = top_right[1]

    # Bird's Eye View
    src_view = np.float32([top_left, top_right, bottom_right, bottom_left])
    dst_view = np.float32([[0,0], [width,0], [width,height], [0,height]])
    M_perspective = cv2.getPerspectiveTransform(src_view, dst_view)
    bev_image = cv2.warpPerspective(road_lane, M_perspective,
                                        (bottom_right[0], bottom_right[1]))
    cv2.imshow("Bird's Eye View", bev_image)

    # Do BEV on original image as well, for contour drawing
    image_to_bev = cv2.warpPerspective(image, M_perspective,
                                        (bottom_right[0], bottom_right[1]))

    # Speed calculation using Optical Flow
    # In each 15 frames, calculate speed once
    current_time = time.time() # Catat waktu frame ini
    dt = current_time - last_frame_time # 1. Hitung selisih waktu (dt)
    current_frame_distance = 0.0         # Jarak khusus di frame ini

    current_gray_bev = bev_image
    # PyrLK Optical Flow
    if prev_points is None or len(prev_points) < 20:
        if prev_gray_bev is None:
            # Ini frame pertama, belum ada 'prev_gray_bev'
            pass
        # Cari fitur bagus di frame sebelumnya
        prev_points = cv2.goodFeaturesToTrack(prev_gray_bev, mask=None, **feature_params)

    # Jika kita punya frame lama DAN poin lama, lacak!
    if prev_gray_bev is not None and prev_points is not None and dt > 0 and current_frame % 30 == 0:
        # Hitung optical flow
        new_points, status, error = cv2.calcOpticalFlowPyrLK(
            prev_gray_bev,
            current_gray_bev,
            prev_points,
            None,
            **lk_params
        )

        if new_points is not None:
            # Filter poin yang berhasil dilacak (status == 1)
            good_new = new_points[status == 1]
            good_old = prev_points[status == 1]

            if len(good_new) > 0:
                # Hitung pergerakan Y rata-rata
                dy_vector = good_new[:, 1] - good_old[:, 1]
                avg_dy_pixels = np.mean(dy_vector)

                # Hitung Kecepatan (Jarak / Waktu)
                current_frame_distance = avg_dy_pixels * CM_PER_PIXEL_Y
                kecepatan_optical_flow_cms = current_frame_distance / dt

            # Update poin untuk dilacak di iterasi berikutnya
            prev_points = good_new.reshape(-1, 1, 2)
        else:
            prev_points = None # Gagal melacak, cari ulang di frame berikutnya

    # Jarak
    total_distance_meters += current_frame_distance / 100.0

    # Simpan frame & waktu saat ini untuk iterasi berikutnya
    prev_gray_bev = current_gray_bev
    last_frame_time = current_time

    # Find contours in the masked
    contours, _ = cv2.findContours(bev_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    min_area_threshold = 20   # VERY SMALL contours (noise)
    max_area_threshold = 450  # Unwanted bigger noise

    center_x = width // 2
    # center_x = 300  # Manually set after ROI observation

    left_lane_points = []
    right_lane_points = []
    unknown_lane_points = []
    for cnt in contours:
        # 1. Filter based on Area
        area = cv2.contourArea(cnt)
        # if area < min_area_threshold or area > max_area_threshold:
        # if area > max_area_threshold:
        if area < min_area_threshold:
            continue

        # 2. Filter based on left/right position
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue

        # Dapatkan titik tengah kontur
        cx = int(M["m10"] / M["m00"])

        # Left lane = Too far left
        if cx <= (center_x - 350):
            left_lane_points.extend(cnt)
            continue

        # Right lane = Too far right
        if cx >= (center_x + 350):
            right_lane_points.extend(cnt)
            continue

        # Obstacle lane = Near center
        if not ((center_x - 100) < cx < (center_x + 100)):
            unknown_lane_points.append(cnt)
            continue

    contour_image = image_to_bev.copy()

    left_line_params = None
    right_line_params = None
    left_angle = -90.0  # Default lurus
    right_angle = -90.0 # Default lurus

    if left_lane_points:
        left_contour = np.array(left_lane_points)
        cv2.drawContours(contour_image, [left_contour], -1, (255, 0, 0), 3)

        # Gambar garis biru
        left_line_params = cv2.fitLine(left_contour, cv2.DIST_L2, 0, 0.01, 0.01)
        draw_fit_line(contour_image, left_line_params, y_top_roi, height, (255, 0, 0), 3)

    if right_lane_points:
        right_contour = np.array(right_lane_points)
        cv2.drawContours(contour_image, [right_contour], -1, (0, 0, 255), 3)

        # Gambar garis merah
        right_line_params = cv2.fitLine(right_contour, cv2.DIST_L2, 0, 0.01, 0.01)
        draw_fit_line(contour_image, right_line_params, y_top_roi, height, (0, 0, 255), 3)

    if unknown_lane_points:
        cv2.drawContours(contour_image, unknown_lane_points, -1, (0, 255, 255), 3)

    cv2.putText(contour_image, f"Speed: {kecepatan_optical_flow_cms:.2f} cm/s", (50, 100),
        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    # Calculate angles
    if left_line_params is None or right_line_params is None:
        turn_degree = 0.0
        turn_degree_msg = "Turn degree is unknown right now vro"
    else:
        left_angle = get_line_angle(left_line_params)
        right_angle = get_line_angle(right_line_params)

        avg_angle = (left_angle + right_angle) / 2.0
        turn_degree = avg_angle - (-90.0) # Deviasi from horizontal straight (-90 deg)
        turn_degree_msg = f"Turn Degree: {turn_degree:.2f} deg"

    cv2.putText(contour_image, turn_degree_msg, (50, 50),
        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.putText(contour_image, f"Jarak: {total_distance_meters:.2f} m", (50, 150),
        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    # Determine road status
    road_status = 0 # 0=empty, 1=half, 2=full
    if left_lane_points and right_lane_points:
        road_status = 2
        cv2.putText(contour_image, f"Road status: Full", (50, 200),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    elif left_lane_points or right_lane_points:
        road_status = 1
        cv2.putText(contour_image, f"Road status: Half", (50, 200),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    else:
        road_status = 0
        cv2.putText(contour_image, f"Road status: Lost", (50, 200),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow("Contours", contour_image)

    # Press escape to exit
    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
