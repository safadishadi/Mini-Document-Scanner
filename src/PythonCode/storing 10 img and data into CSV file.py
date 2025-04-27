import requests
import json
from io import BytesIO
from PIL import Image
import os
import csv

########################################33
#CHECK: is there a need to save full path insted?
#######################################

# The IP address of the ESP32 (replace with your actual ESP32 IP address)
esp32_ip = "http://172.20.10.3/capture"

# Ensure the output directory for CSVs exists
csv_folder = "captured_csv_files"
os.makedirs(csv_folder, exist_ok=True)

# Directory to save images separately
image_dir = "saved_images"
os.makedirs(image_dir, exist_ok=True)

# CSV file columns
csv_columns = ['Image_Index', 'Sensor_Data', 'Image_File_Path']

# Function to process and save the sensor data and image
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
            
            # Search for the JSON part and the image part
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
                
                # Convert sensor data to string (to store in CSV)
                sensor_data_str = json.dumps(sensor_data)
            else:
                print("No sensor data found in the response.")
                sensor_data_str = "No data"

            # Process the Image part
            if image_part:
                image_data = image_part.split(b'\r\n\r\n')[-1]  # Get image content after the header
                image = Image.open(BytesIO(image_data))
                
                # Save the image to disk as a .jpg file in the 'saved_images' directory
                image_save_path = os.path.join(image_dir, f"captured_{iteration}.jpg")
                image.save(image_save_path)
                print(f"Image saved as '{image_save_path}'")
            else:
                print("No image found in the response.")
                image_save_path = "No image"

            # Save data to a new CSV file for this iteration in the 'captured_csv_files' directory
            csv_file_path = os.path.join(csv_folder, f"capture_{iteration}.csv")
            
            # Write data to the CSV file
            with open(csv_file_path, mode='w', newline='') as file:
                writer = csv.DictWriter(file, fieldnames=csv_columns)
                writer.writeheader()  # Write the header for this CSV
                writer.writerow({
                    'Image_Index': iteration,
                    'Sensor_Data': sensor_data_str,
                    'Image_File_Path': image_save_path
                })
            print(f"Data for iteration {iteration} saved to '{csv_file_path}'.")

        else:
            print(f"Failed to get response from ESP32. Status code: {response.status_code}")
    
    except Exception as e:
        print(f"An error occurred: {e}")

# Run the function 10 times and save the results in separate CSV files
for i in range(1, 11):
    print(f"\n--- Iteration {i} ---")
    get_sensor_data_and_image(i)
    print(f"Iteration {i} complete.\n")
