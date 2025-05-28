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
            content_type = response.headers.get('Content-Type', '')
            parts = response.content.split(boundary)
            
            # Search for the JSON part
            json_part = None
            image_part = None

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
                print(f"Sensor Data (iteration {iteration}):", sensor_data)
                
                # Save sensor data to a file
                sensor_file_path = os.path.join(output_dir, f"sensor_data_{iteration}.json")
                with open(sensor_file_path, 'w') as json_file:
                    json.dump(sensor_data, json_file, indent=4)
                print(f"Sensor data saved as '{sensor_file_path}'")
            else:
                print("No sensor data found in the response.")

            # Process the Image part
            if image_part:
                image_data = image_part.split(b'\r\n\r\n')[-1]  # Get image content after the header
                image = Image.open(BytesIO(image_data))
                
                # Save the image to a file
                image_file_path = os.path.join(output_dir, f"captured_{iteration}.jpg")

                # # crop middle
                # image_np = np.array(image)
                # h, w = image_np.shape[:2]
                # crop = image_np[h//5:4*h//5, w//5:4*w//5]
                # crop_image = Image.fromarray(crop)

                image.save(image_file_path)
                print(f"Image saved as '{image_file_path}'")
            else:
                print("No image found in the response.")
        else:
            print(f"Failed to get response from ESP32. Status code: {response.status_code}")
    
    except Exception as e:
        print(f"An error occurred: {e}")

# Run the function 8 times and save the results in files
for i in range(1, 13):
    print(f"\n--- Iteration {i} ---")
    get_sensor_data_and_image(i)
    time.sleep(1)
    print(f"Iteration {i} complete.\n")

##########################################
##########################################

# take images into a list, delete first unstable 3, leave 2
images = [cv2.imread(f"captured_data/captured_{i}.jpg") for i in range(4, 13)]


# Define the target A4 size at 300 DPI (3508x2480 pixels)
a4_width = 3508
a4_height = 2480

# Create a Stitcher object (for stitching the images)
stitcher = cv2.Stitcher_create()  

# Perform the stitching
status, stitched = stitcher.stitch(images)

# Check if stitching was successful
if status == cv2.Stitcher_OK:
    print("Stitching successful!")

    # # Get the dimensions of the stitched result
    # stitched_height, stitched_width = stitched.shape[:2]

    # # Ensure the stitched image fits within the A4 dimensions
    # if stitched_width > a4_width or stitched_height > a4_height:
    #     print("Stitched image is too large. Resizing to A4.")
        
    #     # Resize the stitched image to fit within A4 size (3508x2480)
    #     stitched_resized = cv2.resize(stitched, (a4_width, a4_height))
    # else:
    #     stitched_resized = stitched  # No resizing needed if already A4
    stitched_resized = stitched 
    # Show and save the final result
    cv2.imshow("Stitched A4 Image", stitched_resized)
    cv2.imwrite("stitched_result_A4.jpg", stitched_resized)

else:
    print("Error during stitching. Status code:", status)

cv2.waitKey(0)
cv2.destroyAllWindows()
