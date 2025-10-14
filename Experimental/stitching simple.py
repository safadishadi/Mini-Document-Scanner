import cv2
import os

#print("Current working directory:", os.getcwd())

# check to see if the images exist:
image_files = ["image1.png", "image2.png", "image3.png"]

for image_path in image_files:
    if not os.path.exists(image_path):
        print(f"File not found: {image_path}")
    else:
        print(f"File found: {image_path}")


# Load your images
image_files = ["image1.png", "image2.png", "image3.png"] #, "image3.jpg"]
images = [cv2.imread(file) for file in image_files]

# Initialize the stitcher
stitcher = cv2.Stitcher_create()

# Stitch the images
status, stitched = stitcher.stitch(images)

if status == cv2.Stitcher_OK:
    print("Stitching successful!")
    a4_width = 2480
    a4_height = 3508
    stitched_resized = cv2.resize(stitched, (a4_width, a4_height))
    cv2.imshow("Stitched Image", stitched)
    cv2.imwrite("stitched_result.png", stitched)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print(f"Stitching failed with status code {status}")
