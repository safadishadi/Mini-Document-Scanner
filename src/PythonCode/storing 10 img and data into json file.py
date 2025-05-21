import requests
import json
from io import BytesIO
from PIL import Image
import os
import time

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
                image.save(image_file_path)
                print(f"Image saved as '{image_file_path}'")
            else:
                print("No image found in the response.")
        else:
            print(f"Failed to get response from ESP32. Status code: {response.status_code}")
    
    except Exception as e:
        print(f"An error occurred: {e}")

# Run the function 10 times and save the results in files
for i in range(1, 26):
    print(f"\n--- Iteration {i} ---")
    get_sensor_data_and_image(i)
    time.sleep(1)
    print(f"Iteration {i} complete.\n")
