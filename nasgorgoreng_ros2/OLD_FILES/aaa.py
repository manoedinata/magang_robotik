import cv2
import numpy as np
import math
import time  # <-- Tambahkan ini untuk timer

# --- Helper Functions (Fungsi Bantu) ---
# (Salin fungsi draw_fit_line dan get_line_angle dari jawaban sebelumnya ke sini)
def draw_fit_line(img, line_params, y_top, y_bottom, color, thickness):
    if line_params is None or len(line_params) < 4: return
    vx = line_params[0][0]; vy = line_params[1][0]
    x = line_params[2][0]; y = line_params[3][0]
    if vy == 0: return
    x1 = int(x + (y_bottom - y) * vx / vy); y1 = y_bottom
    x2 = int(x + (y_top - y) * vx / vy); y2 = y_top
    cv2.line(img, (x1, y1), (x2, y2), color, thickness)

def get_line_angle(line_params):
    if line_params is None or len(line_params) < 4: return 0.0
    vx = line_params[0][0]; vy = line_params[1][0]
    if vy > 0: vx = -vx; vy = -vy
    angle_deg = np.degrees(np.arctan2(vy, vx))
    return angle_deg

# --- KONSTANTA BARU UNTUK KECEPATAN (METODE 1) ---
# !!! SESUAIKAN NILAI INI !!!
JARAK_ANTAR_GARIS_CM = 50.0  # (Contoh: 50 cm)
LOWER_MARKER = (20, 100, 100) # (Contoh: HSV untuk Kuning)
UPPER_MARKER = (30, 255, 255) # (Contoh: HSV untuk Kuning)
MIN_AREA_MARKER = 500        # Area minimum kontur marker agar terdeteksi

# --- Inisialisasi Variabel State (Status) ---
timer_dimulai = False
sedang_di_atas_marker = False
waktu_mulai = 0
kecepatan_terakhir = 0.0

# --- Buka Video Capture ---
# Ganti 0 dengan nama file video jika Anda memproses file, misal "video.mp4"
cap = cv2.VideoCapture("./FIRA_ARENA.mp4")
if not cap.isOpened():
    raise IOError("Tidak bisa membuka webcam or video file")

# --- LOOP UTAMA (Memproses Video Frame demi Frame) ---
while True:
    ret, frame = cap.read()
    if not ret:
        print("Video selesai atau frame tidak bisa dibaca.")
        break

    # Ganti nama 'image' menjadi 'frame' untuk semua proses
    # Dapatkan height/width dari frame
    height, width = frame.shape[:2]
    
    # --- Bagian 1: Persiapan Gambar ---
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # --- LOGIKA DETEKSI KECEPATAN (METODE 1) ---
    # Buat mask KHUSUS untuk marker (garis start/finis)
    mask_marker = cv2.inRange(hsv, LOWER_MARKER, UPPER_MARKER)
    contours_marker, _ = cv2.findContours(mask_marker, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    marker_terdeteksi_sekarang = False
    for cnt in contours_marker:
        if cv2.contourArea(cnt) > MIN_AREA_MARKER:
            marker_terdeteksi_sekarang = True
            break # Cukup temukan satu

    # Logika State Machine Sederhana
    if marker_terdeteksi_sekarang and not sedang_di_atas_marker:
        # Event: Robot BARU SAJA MEMASUKI marker
        sedang_di_atas_marker = True
        
        if not timer_dimulai:
            # Ini adalah marker START
            timer_dimulai = True
            waktu_mulai = time.time()
            print("TIMER DIMULAI!")
        else:
            # Ini adalah marker FINISH
            waktu_selesai = time.time()
            durasi = waktu_selesai - waktu_mulai
            
            if durasi > 0: # Hindari pembagian dengan nol
                kecepatan_terakhir = JARAK_ANTAR_GARIS_CM / durasi
                print(f"FINISH! Durasi: {durasi:.2f} s, Kecepatan: {kecepatan_terakhir:.2f} cm/s")
            
            # Reset timer untuk pengukuran berikutnya
            timer_dimulai = False 
            
    elif not marker_terdeteksi_sekarang:
        # Event: Robot TIDAK di atas marker
        sedang_di_atas_marker = False
    # -----------------------------------------------

    # --- Bagian 2: Deteksi Jalur (Kode Anda sebelumnya) ---
    lower_green = (33, 50, 50)
    upper_green = (85, 255, 255)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    inverted_mask = cv2.bitwise_not(mask_green)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 220, 255)
    road_lane = cv2.bitwise_and(inverted_mask, edges)
    # cv2.imshow("1. Road Lane", road_lane) # Nyalakan untuk debug

    # --- Bagian 3: ROI ---
    mask_roi = np.zeros_like(road_lane)
    bottom_left  = (400, 700)
    bottom_right = (900, 700)
    top_left     = (480, 400) 
    top_right    = (750, 400)
    y_top_roi = 400 
    roi_corners = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    cv2.fillPoly(mask_roi, roi_corners, 255)
    masked_image = cv2.bitwise_and(road_lane, mask_roi)
    # cv2.imshow("2. ROI Mask", masked_image) # Nyalakan untuk debug

    # --- Bagian 4: Filter Kontur & Hitung Sudut ---
    contours, _ = cv2.findContours(masked_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    left_lane_points = []
    right_lane_points = []
    min_area_threshold = 500
    center_x = width / 2

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area_threshold: continue
        M = cv2.moments(cnt)
        if M["m00"] == 0: continue
        cx = int(M["m10"] / M["m00"])
        if cx < center_x:
            left_lane_points.extend(cnt)
        else:
            right_lane_points.extend(cnt)

    contour_image = frame.copy() # Gambar final adalah 'frame'
    left_line_params = None
    right_line_params = None
    left_angle = -90.0
    right_angle = -90.0

    if left_lane_points:
        left_points_np = np.array(left_lane_points)
        left_line_params = cv2.fitLine(left_points_np, cv2.DIST_L2, 0, 0.01, 0.01)
        draw_fit_line(contour_image, left_line_params, y_top_roi, height, (255, 0, 0), 3)

    if right_lane_points:
        right_points_np = np.array(right_lane_points)
        right_line_params = cv2.fitLine(right_points_np, cv2.DIST_L2, 0, 0.01, 0.01)
        draw_fit_line(contour_image, right_line_params, y_top_roi, height, (0, 0, 255), 3)

    if left_line_params is not None: left_angle = get_line_angle(left_line_params)
    if right_line_params is not None: right_angle = get_line_angle(right_line_params)
    
    avg_angle = (left_angle + right_angle) / 2.0
    turn_degree = avg_angle - (-90.0)

    # --- Bagian 5: Visualisasi Akhir ---
    # Tampilkan Turn Degree
    cv2.putText(contour_image, f"Turn: {turn_degree:.2f} deg", (50, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    
    # Tampilkan Kecepatan Terakhir
    cv2.putText(contour_image, f"Speed: {kecepatan_terakhir:.2f} cm/s", (50, 100), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    
    # Tampilkan status timer
    if timer_dimulai:
        cv2.putText(contour_image, "TIMER RUNNING", (width - 300, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    cv2.imshow("Final Result (Speed + Turn)", contour_image)

    # Keluar dari loop jika menekan 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
