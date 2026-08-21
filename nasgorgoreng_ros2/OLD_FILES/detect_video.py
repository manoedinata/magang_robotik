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

    masked_image = cv2.bitwise_and(edges, mask_roi)
    cv2.imshow("ROI Mask", masked_image)

    # Find contours in the masked
    contours, _ = cv2.findContours(masked_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contour_image = image.copy()
    cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 3)
    cv2.imshow("Contours", contour_image)


    # Press escape to exit
    if cv2.waitKey(30) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
