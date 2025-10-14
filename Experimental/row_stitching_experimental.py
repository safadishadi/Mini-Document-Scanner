import cv2
import glob
import os

# Change working directory to where the script itself is
os.chdir(os.path.dirname(os.path.abspath(__file__)))
print("Current working directory:", os.getcwd())

# Load all TIFF images in current directory
image_paths = sorted(glob.glob(os.path.join(os.getcwd(), "*.tiff")))

# Limit to first 4 if more exist
image_paths = image_paths[:4]

if not image_paths:
    print("No TIFF images found in current directory.")
    exit()

# Read and rotate each image 90° counterclockwise
images = [cv2.rotate(cv2.imread(p), cv2.ROTATE_90_COUNTERCLOCKWISE) for p in image_paths]

# Create stitcher and combine images horizontally (since we rotated)
stitcher = cv2.Stitcher_create()
status, stitched_rotated = stitcher.stitch(images)

if status == cv2.Stitcher_OK:
    # Rotate back 90° clockwise
    full_scan = cv2.rotate(stitched_rotated, cv2.ROTATE_90_CLOCKWISE)
    cv2.imwrite("stitched_experiment_result.tiff", full_scan)
    print("✅ Successfully created stitched_experiment_result.tiff")
else:
    print(f"❌ Stitching failed. Status code: {status}")
