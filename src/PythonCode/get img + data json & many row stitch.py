import requests
import json
from io import BytesIO
from PIL import Image
import os
import time
import cv2
import numpy as np

# The IP address of the ESP32 (replace with your actual ESP32 IP address)
esp32_ip = "http://172.20.10.3/capture"

# Ensure the output directory exists
output_dir = "captured_data"
os.makedirs(output_dir, exist_ok=True)

# Function to make a request to ESP32 and process the response
def get_sensor_data_and_image():
    try:
        # Send GET request to the ESP32 /capture endpoint
        response = requests.get(esp32_ip, stream=True)

        # Check if the request was successful
        if response.status_code == 200:
            # Initialize variables
            boundary = b'--frame'
            parts = response.content.split(boundary)

            # Search for the JSON part
            json_part, image_part = None, None

            # Loop through each part of the response
            for part in parts:
                if b'Content-Type: application/json' in part:
                    json_part = part.strip()
                elif b'Content-Type: image/jpeg' in part:
                    image_part = part.strip()

            # Process the JSON part
            if json_part:
                json_data = json_part.split(b'\r\n\r\n')[-1].decode('utf-8')  # Get JSON content after the header
                sensor_data = json.loads(json_data)
                distance_cm = sensor_data.get("distance_cm", 999)
                displacement_y = sensor_data.get("displacement_cm_y", 0)
                return distance_cm, displacement_y, sensor_data, image_part
            else:
                print("No sensor data found.")
                return None, None, None, None
        else:
            print(f"Failed to get response from ESP32. Status: {response.status_code}")
            return None, None, None, None
    except Exception as e:
        print(f"Error: {e}")
        return None, None, None, None

# Parameters
max_rows = 5
max_images_per_row = 20  # safety limit per row
row_images = []
all_stitched_rows = []

print("Waiting for object to come closer (< 60 cm)...")
capturing = False
current_row = 0
image_count = 0
row_image_paths = []
prev_displacement_y = 0

while True:
    distance_cm, displacement_y, sensor_data, image_part = get_sensor_data_and_image()
    if distance_cm is None:
        time.sleep(0.1)
        continue

    print(f"Distance: {distance_cm:.1f} cm, Displacement Y: {displacement_y:.1f} cm")

    if not capturing:
        if distance_cm < 60:
            print("Object detected! Starting capture...")
            capturing = True
        else:
            time.sleep(0.2)
            continue

    if capturing:
        if distance_cm < 60:
            image_count += 1

            # Save sensor data
            sensor_path = os.path.join(output_dir, f"sensor_data_row{current_row}_img{image_count}.json")
            with open(sensor_path, 'w') as f:
                json.dump(sensor_data, f, indent=4)

            # Save image
            image_data = image_part.split(b'\r\n\r\n')[-1]
            image = Image.open(BytesIO(image_data))
            image_path = os.path.join(output_dir, f"captured_row{current_row}_img{image_count}.jpg")
            image.save(image_path)
            row_image_paths.append(image_path)

            print(f"Saved image #{image_count} in row {current_row} at distance {distance_cm:.1f} cm")

            # Check for new row start
            if displacement_y > 17:
                print(f"New row detected (Y displacement {displacement_y:.1f} cm). Moving to next row...")
                row_images.append(row_image_paths)
                row_image_paths = []
                current_row += 1
                image_count = 0

                if current_row >= max_rows:
                    print("Max row count reached.")
                    break

            time.sleep(0.5)
        else:
            # Object moved away, stop if already captured something
            if row_image_paths:
                row_images.append(row_image_paths)
            print("Object moved away. Ending capture.")
            break

# Stitching each row separately
stitched_rows = []
print("\n--- Stitching Rows ---")
for row_idx, paths in enumerate(row_images):
    images = [cv2.imread(p) for p in paths]
    stitcher = cv2.Stitcher_create()
    status, stitched = stitcher.stitch(images)

    if status == cv2.Stitcher_OK:
        print(f"Row {row_idx} stitched successfully.")
        stitched_rows.append(stitched)
        cv2.imwrite(os.path.join(output_dir, f"stitched_row_{row_idx}.jpg"), stitched)

        # Delete captured images after stitching
        for i in range(1, image_count + 1):
            img_path = os.path.join(output_dir, f"captured_{i}.jpg")
            if os.path.exists(img_path):
                os.remove(img_path)
        print("Captured images deleted after stitching.")
        
    else:
        print(f"Error stitching row {row_idx}. Status: {status}")

# Combine stitched rows vertically into a final A4 layout
if stitched_rows:
    final = cv2.vconcat(stitched_rows)
    cv2.imshow("Final A4 Image", final)
    cv2.imwrite("stitched_result_A4.jpg", final)

cv2.waitKey(0)
cv2.destroyAllWindows()
