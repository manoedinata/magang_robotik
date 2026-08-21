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


PANJANG_GARIS_PUTUS_CM = 20.0
JARAK_ANTAR_GARIS_PUTUS_CM = 15.0
JARAK_SIKLUS_CM = PANJANG_GARIS_PUTUS_CM + JARAK_ANTAR_GARIS_PUTUS_CM

waktu_mulai = 0
sedang_di_atas_garis = False
kecepatan_terakhir_cm_s = 0.0
current_middle_lane_total = 0
total_distance_cm = 0.0

file = "./FIRA_ARENA.mp4"
# file = "http://10.7.101.192:8080/video"
# file = "http://10.137.196.11:8080/video"
cap = cv2.VideoCapture(file)
if not cap.isOpened():
    raise FileNotFoundError(f"Video {file} not found or cannot be opened.")

while True:
    ret, image = cap.read()
    if not ret:
        break

    # Blur image to reduce noise
    image = cv2.GaussianBlur(image, (5, 5), 0)

    # Convert to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    height, width = hsv.shape[:2]

    # UNTUK 1280x720 VIDEO
    bottom_left  = (0, height)        # Kiri bawah
    bottom_right = (width, height)       # Kanan bawah
    top_left     = (int(width * 0.2), 280) # Kiri atas
    top_right    = (int(width * 0.8), 280) # Kanan atas

    # UNTUK 640x480 VIDEO (BELUM STABIL)
    # bottom_left  = (0, height)
    # bottom_right = (width, height)
    # top_left     = (110, 160)
    # top_right    = (600, 200)

    y_top_roi = top_right[1]

    # Bird's eye view
    src_view = np.float32([top_left, top_right, bottom_right, bottom_left])
    dst_view = np.float32([[0,0], [width,0], [width,height], [0,height]])
    M_perspective = cv2.getPerspectiveTransform(src_view, dst_view)
    bev_image = cv2.warpPerspective(image, M_perspective,
                                        (bottom_right[0], bottom_right[1]))
    cv2.imshow("Bird's Eye View", bev_image)

    # Get road lane by inverting the green color mask
    lower_green = (33, 50, 50)
    upper_green = (85, 255, 255)
    mask = cv2.inRange(bev_image, lower_green, upper_green)
    inverted_mask = cv2.bitwise_not(mask)
    # cv2.imshow("Inverted Green Mask", inverted_mask)

    # Get lane lines by converting to grayscale and applying Canny edge detection
    gray = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 100, 150)
    # cv2.imshow("Canny Edges", edges)

    # Combine the inverted green mask and Canny edges
    road_lane = cv2.bitwise_and(inverted_mask, edges)
    cv2.imshow("Road Lane", road_lane)


    # ROI to focus on the road area
    # edges = road that has been canny edge detected
    # mask_roi = np.zeros_like(edges)

    # roi_corners = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    # cv2.fillPoly(mask_roi, roi_corners, 255)

    # masked_image = cv2.bitwise_and(road_lane, mask_roi)
    # cv2.imshow("ROI Mask", masked_image)
    masked_image = road_lane # No ROI for BEV

    # Find contours in the masked
    contours, _ = cv2.findContours(masked_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    min_area_threshold = 20   # VERY SMALL contours (noise)
    max_area_threshold = 450  # Unwanted bigger noise

    center_x = width // 2
    # center_x = 300  # Manually set after ROI observation

    left_lane_points = []
    right_lane_points = []
    middle_lane_points = []
    obstacle_lane_points = []
    for cnt in contours:
        # 1. Filter based on Area
        area = cv2.contourArea(cnt)

        # 2. Filter based on left/right position
        M = cv2.moments(cnt)

        if M["m00"] == 0:
            continue

        cx = int(M["m10"] / M["m00"]) # Dapatkan titik tengah kontur

        # Add contours to list
        # Middle lines contour
        if cx >= center_x - 100 and cx <= center_x + 100:
            if area >= 60:
                middle_lane_points.append(cnt)
                continue

        # Left or Right lines contour
        else:
                # if area < min_area_threshold or area > max_area_threshold:
                # if area > max_area_threshold:
                if area < min_area_threshold:
                    continue

                if cx >= center_x - 150 and cx <= center_x + 150:
                    obstacle_lane_points.append(cnt)
                    continue

                if cx < center_x:
                    left_lane_points.extend(cnt)
                else:
                    right_lane_points.extend(cnt)

                # elif cx < center_x:
                #     if cx < center_x - 70:
                #         obstacle_lane_points.append(cnt)
                #     else:
                #         left_lane_points.extend(cnt)

                # else:
                #     if cx > center_x + 70:
                #         obstacle_lane_points.append(cnt)
                #     else:
                #         right_lane_points.extend(cnt)

    contour_image = bev_image.copy()

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

    if obstacle_lane_points:
        cv2.drawContours(contour_image, obstacle_lane_points, -1, (0, 255, 255), 3)

    garis_terdeteksi = False
    if middle_lane_points:
        garis_terdeteksi = True

        cv2.drawContours(contour_image, middle_lane_points, -1, (0, 255, 0), 3)

    # Reset middle lanes if anomaly detected
    # if len(middle_lane_points) < current_middle_lane_total:
    #     print("Anomaly detected in middle lane contours, resetting count.")
    #     current_middle_lane_total = len(middle_lane_points)

    if garis_terdeteksi and not sedang_di_atas_garis:
        sedang_di_atas_garis = True

        if waktu_mulai == 0:
            # Ini adalah garis PERTAMA yang kita lihat
            waktu_mulai = time.time()
            print("TIMER DIMULAI: Melewati garis 1")
        # else:

    # New line detected
    if len(middle_lane_points) != current_middle_lane_total:
        if len(middle_lane_points) < current_middle_lane_total:
            pass  # Ignore if less (noise)
        else:
            print("Resetting sedang_di_atas_garis")
            print("Middle lane sebelumnya: ", current_middle_lane_total)
            print("Middle lane sekarang: ", len(middle_lane_points))
            current_middle_lane_total = len(middle_lane_points)

            # Ini adalah garis KEDUA (atau ketiga, dst.)
            waktu_selesai = time.time()
            durasi = waktu_selesai - waktu_mulai

            # Safety check (hindari durasi 0 atau terlalu cepat)
            if durasi > 0.1:
                kecepatan_terakhir_cm_s = JARAK_SIKLUS_CM / durasi
                print(f"TIMER HIT: Durasi: {durasi:.2f} s, Kecepatan: {kecepatan_terakhir_cm_s:.2f} cm/s")
                total_distance_cm += JARAK_SIKLUS_CM

            # Reset timer untuk pengukuran berikutnya
            waktu_mulai = waktu_selesai
            sedang_di_atas_garis = False # Reset status
            garis_terdeteksi = False
        # -----------------------------------------------

    cv2.putText(contour_image, f"Speed: {kecepatan_terakhir_cm_s:.2f} cm/s", (50, 100),
        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.putText(contour_image, f"Green Contours: {len(middle_lane_points)}", (50, 150),
        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.putText(contour_image, f"Total distance: {total_distance_cm:.2f}", (50, 200),
        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    # Calculate angles
    if left_line_params is None or right_line_params is None:
        left_angle = None
        right_angle = None

        turn_degree = None
        turn_degree_msg = "Turn degree is unknown right now vro"
    else:
        left_angle = get_line_angle(left_line_params)
        right_angle = get_line_angle(right_line_params)

        avg_angle = (left_angle + right_angle) / 2.0
        turn_degree = avg_angle - (-90.0) # Deviasi from horizontal straight (-90 deg)
        turn_degree_msg = f"Turn Degree: {turn_degree:.2f} deg"

    cv2.putText(contour_image, turn_degree_msg, (50, 50),
        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.imshow("Contours", contour_image)

    # if left_lane_points and right_lane_points:
    #     print("ROAD STATUS: FULL")
    # elif left_lane_points or right_lane_points:
    #     print("ROAD STATUS: HALF")
    # else:
    #     print("ROAD STATUS: EMPTY")

    # Press escape to exit
    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
