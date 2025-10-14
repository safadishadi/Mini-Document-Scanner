import cv2
import numpy as np

# Load the images
image_files = ["image1.jpg", "image2.jpg"]
images = [cv2.imread(file) for file in image_files]

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

    # Get the dimensions of the stitched result
    stitched_height, stitched_width = stitched.shape[:2]

    # Ensure the stitched image fits within the A4 dimensions
    if stitched_width > a4_width or stitched_height > a4_height:
        print("Stitched image is too large. Resizing to A4.")
        
        # Resize the stitched image to fit within A4 size (3508x2480)
        stitched_resized = cv2.resize(stitched, (a4_width, a4_height))
    else:
        stitched_resized = stitched  # No resizing needed if already A4

    # Show and save the final result
    cv2.imshow("Stitched A4 Image", stitched_resized)
    cv2.imwrite("stitched_result_A4.jpg", stitched_resized)

else:
    print("Error during stitching. Status code:", status)

cv2.waitKey(0)
cv2.destroyAllWindows()
