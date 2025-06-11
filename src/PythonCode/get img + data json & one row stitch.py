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
def get_sensor_data_and_image(iteration):
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
                return distance_cm, sensor_data, image_part
            else:
                print("No sensor data found.")
                return None, None, None
        else:
            print(f"Failed to get response from ESP32. Status: {response.status_code}")
            return None, None, None
    except Exception as e:
        print(f"Error: {e}")
        return None, None, None

# Wait until object gets close
print("Waiting for object to come closer (< 60 cm)...")
capturing = False
image_count = 0
max_images = 20  # safety limit

while True:
    distance_cm, sensor_data, image_part = get_sensor_data_and_image(image_count + 1)
    if distance_cm is None:
        time.sleep(0.1)
        continue

    print(f"Distance: {distance_cm:.1f} cm")

    if not capturing:
        if distance_cm < 60:
            print("Object detected! Starting capture...")
            capturing = True
        else:
            time.sleep(0.2)
            continue

    # If currently capturing and object is still close, save image
    if capturing and distance_cm < 60:
        image_count += 1

        # Save sensor data
        with open(f"{output_dir}/sensor_data_{image_count}.json", 'w') as f:
            json.dump(sensor_data, f, indent=4)

        # Save image
        image_data = image_part.split(b'\r\n\r\n')[-1]
        image = Image.open(BytesIO(image_data))
        image_path = os.path.join(output_dir, f"captured_{image_count}.jpg")
        image.save(image_path)
        print(f"Saved image #{image_count} at distance {distance_cm:.1f} cm")
        print(f"X_Y_Z displacements in cm:  #{image_count} #{image_count} #{image_count}")

        if image_count >= max_images:
            print("Max image count reached.")
            break

        time.sleep(0.5)

    # Stop capturing if object has moved away
    elif capturing and distance_cm >= 60:
        print("Object moved away. Ending capture.")
        break

# Stitching captured images
print("\n--- Stitching ---")
images = [cv2.imread(os.path.join(output_dir, f"captured_{i}.jpg")) for i in range(1, image_count + 1)]

a4_width = 3508
a4_height = 2480

# Create a Stitcher object (for stitching the images)
stitcher = cv2.Stitcher_create()  

# Perform the stitching
status, stitched = stitcher.stitch(images)

# Check if stitching was successful
if status == cv2.Stitcher_OK:
    print("Stitching successful!")
    stitched_resized = stitched 
    # Show and save the final result
    cv2.imshow("Stitched A4 Image", stitched_resized)
    cv2.imwrite("stitched_result_rone_row.jpg", stitched_resized)

    # Delete captured images after stitching
    for i in range(1, image_count + 1):
        img_path = os.path.join(output_dir, f"captured_{i}.jpg")
        if os.path.exists(img_path):
            os.remove(img_path)
    print("Captured images deleted after stitching.")

else:
    # Delete captured images after stitching
    for i in range(1, image_count + 1):
        img_path = os.path.join(output_dir, f"captured_{i}.jpg")
        if os.path.exists(img_path):
            os.remove(img_path)
    print("Captured images deleted after stitching.")
    print("Error during stitching. Status code:", status)

cv2.waitKey(0)
cv2.destroyAllWindows()
