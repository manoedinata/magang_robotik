import cv2
import numpy as np
import math

import json
import websockets
import base64
import asyncio

ws_url = "ws://localhost:8765"

async def sendImageToWebsocketServer(websocket_instance, image, jsonBody, uri=ws_url):
    # Encode image as Base64
    # Jalankan operasi encoding yang (sedikit) CPU-bound ini di thread juga
    def encode_image(img):
        _, buffer = cv2.imencode('.jpg', img)
        return base64.b64encode(buffer).decode('utf-8')

    encoded_jpg = await asyncio.to_thread(encode_image, image)

    # Send message
    jsonBody['data'] = encoded_jpg
    await websocket_instance.send(json.dumps(jsonBody))


def process_frame(image):
    # Convert to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Get road lane by inverting the green color mask
    lower_green = (33, 50, 50)
    upper_green = (85, 255, 255)
    mask = cv2.inRange(hsv, lower_green, upper_green)
    inverted_mask = cv2.bitwise_not(mask)

    # Get lane lines by converting to grayscale and applying Canny edge detection
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 220, 255)

    # Combine the inverted green mask and Canny edges
    road_lane = cv2.bitwise_and(inverted_mask, edges)
    
    # ROI to focus on the road area
    # edges = road that has been canny edge detected
    height, width = edges.shape[:2]
    mask_roi = np.zeros_like(edges)

    bottom_left  = (0, 700)
    bottom_right = (width, 700)
    top_left     = (int(width * 0.2), 280)
    top_right    = (int(width * 0.83), 280)

    roi_corners = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    cv2.fillPoly(mask_roi, roi_corners, 255)

    masked_image = cv2.bitwise_and(edges, mask_roi)

    contours, _ = cv2.findContours(masked_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour_image = image.copy()
    cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 3)

    # PENTING: cv2.imshow harus dijalankan di thread utama
    # Jika ingin menampilkannya, jangan jalankan di dalam 'process_frame'
    # Tapi ini akan memblokir lagi.
    # !!! Sebaiknya jangan tampilkan saat streaming. !!!

    return masked_image, contour_image

async def main():
    async with websockets.connect(ws_url) as websocket:
        file = "./FIRA_ARENA.mp4"
        cap = cv2.VideoCapture(file)
        if not cap.isOpened():
            raise FileNotFoundError(f"Video {file} not found or cannot be opened.")

        print("Video opened, starting stream...")
        while True:
            # Jalankan I/O blocking (cap.read) di thread terpisah
            ret, image = await asyncio.to_thread(cap.read)
            if not ret:
                print("End of video.")
                break

            # Kirim gambar mentah (sudah async)
            await sendImageToWebsocketServer(websocket, image, {
                "type": "image_raw",
                "width": image.shape[1],
                "height": image.shape[0]
            })

            # Jalankan CPU blocking (process_frame) di thread terpisah
            masked_image, contour_image = await asyncio.to_thread(process_frame, image)
            
            # Kirim gambar hasil olah (sudah async)
            await sendImageToWebsocketServer(websocket, masked_image, {
                "type": "image_processed",
                "width": contour_image.shape[1],
                "height": contour_image.shape[0]
            })

        print("Stream finished.")
        cap.release()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Stream stopped by user.")
    except Exception as e:
        print(f"An error occurred: {e}")
