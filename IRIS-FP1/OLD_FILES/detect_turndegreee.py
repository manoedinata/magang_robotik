import cv2
import numpy as np
import math

file = "./FIRA_ARENA.mp4"
cap = cv2.VideoCapture(file)
if not cap.isOpened():
    raise FileNotFoundError(f"Video {file} not found or cannot be opened.")

while True:
    ret, image = cap.read()
    if not ret:
        break

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
    edges = cv2.Canny(gray, 220, 255)
    # cv2.imshow("Canny Edges", edges)

    # Combine the inverted green mask and Canny edges
    road_lane = cv2.bitwise_and(inverted_mask, edges)
    cv2.imshow("Road Lane", road_lane)


    # ROI to focus on the road area
    # edges = road that has been canny edge detected
    height, width = edges.shape[:2]
    mask_roi = np.zeros_like(edges)

    bottom_left  = (0, 700)        # Kiri bawah
    bottom_right = (width, 700)       # Kanan bawah
    top_left     = (int(width * 0.2), 280) # Kiri atas
    top_right    = (int(width * 0.83), 280) # Kanan atas

    roi_corners = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    cv2.fillPoly(mask_roi, roi_corners, 255)

    masked_image = cv2.bitwise_and(road_lane, mask_roi)
    cv2.imshow("ROI Mask", masked_image)

    # Find contours in the masked
    contours, _ = cv2.findContours(masked_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    min_area_threshold = 60   # VERY SMALL contours (noise)
    max_area_threshold = 450  # Unwanted bigger noise

    # center_x = width // 2
    center_x = 630  # Manually set after ROI observation

    left_lane_points = []
    right_lane_points = []
    for cnt in contours:
        # 1. Filter based on Area
        area = cv2.contourArea(cnt)
        if area < min_area_threshold or area > max_area_threshold:
            continue

        # 2. Filter based on left/right position
        M = cv2.moments(cnt)

        if M["m00"] == 0:
            continue

        cx = int(M["m10"] / M["m00"]) # Dapatkan titik tengah kontur
        print(f"Contour Area: {area}, Centroid X: {cx}")

        # Add contours to list
        if cx < center_x:
            left_lane_points.extend(cnt)
        else:
            right_lane_points.extend(cnt)

    contour_image = image.copy()

    if left_lane_points and right_lane_points:
        print("ROAD STATUS: FULL")
    elif left_lane_points or right_lane_points:
        print("ROAD STATUS: HALF")
    else:
        print("ROAD STATUS: EMPTY")

    if left_lane_points:
        left_contour = np.array(left_lane_points)
        cv2.drawContours(contour_image, [left_contour], -1, (255, 0, 0), 3)
    if right_lane_points:
        right_contour = np.array(right_lane_points)
        cv2.drawContours(contour_image, [right_contour], -1, (0, 0, 255), 3)

    cv2.imshow("Contours", contour_image)

    # Press escape to exit
    if cv2.waitKey(30) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
