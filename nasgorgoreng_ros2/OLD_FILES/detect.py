import cv2
import numpy as np
import math

file = "./image.png"
image = cv2.imread(file)
if image is None:
    raise FileNotFoundError(f"Image {file} file not found.")

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
bottom_right = (1080, 700)       # Kanan bawah
top_left     = (390, 280) # Kiri atas
top_right    = (1000, 280) # Kanan atas

roi_corners = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
cv2.fillPoly(mask_roi, roi_corners, 255)

masked_image = cv2.bitwise_and(edges, mask_roi)
cv2.imshow("ROI Mask", masked_image)

# Find contours in the masked
contours, _ = cv2.findContours(masked_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

contour_image = image.copy()
cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 3)
cv2.imshow("Contours", contour_image)

cv2.waitKey(0)
cv2.destroyAllWindows()