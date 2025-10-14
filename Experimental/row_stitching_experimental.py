import cv2
import glob
import os
import numpy as np

def crop_to_strict_nonblack_rectangle(img, threshold=30, black_ratio_limit=0.35):
    """
    Crops aggressively to remove all black edges, even uneven ones.
    Allows up to black_ratio_limit fraction of dark pixels per row/column.
    Ensures final crop rectangle contains no visible black borders.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

    h, w = mask.shape

    # Compute fraction of black pixels per row and column
    row_black_ratio = np.mean(mask == 0, axis=1)
    col_black_ratio = np.mean(mask == 0, axis=0)

    # Find first/last rows and cols that are mostly white (few black pixels)
    top = next((i for i in range(h) if row_black_ratio[i] < black_ratio_limit), 0)
    bottom = next((i for i in range(h-1, -1, -1) if row_black_ratio[i] < black_ratio_limit), h-1)
    left = next((i for i in range(w) if col_black_ratio[i] < black_ratio_limit), 0)
    right = next((i for i in range(w-1, -1, -1) if col_black_ratio[i] < black_ratio_limit), w-1)

    # Ensure valid crop box
    top = max(top, 0)
    left = max(left, 0)
    bottom = min(bottom, h - 1)
    right = min(right, w - 1)

    if right <= left or bottom <= top:
        print("⚠️ No valid crop region found, skipping.")
        return img

    cropped = img[top:bottom, left:right]
    return cropped







# Change working directory to where the script itself is
os.chdir(os.path.dirname(os.path.abspath(__file__)))
print("Current working directory:", os.getcwd())

# Load all TIFF images in current directory
image_paths = sorted(glob.glob("*.tiff"))
print("Found TIFFs:", image_paths)

# Limit to first 4 if more exist
image_paths = image_paths[:4]

if not image_paths:
    print("No TIFF images found in current directory.")
    exit()

# Read and rotate each image 90° counterclockwise (for normal stitching)
rotated_images = [cv2.rotate(cv2.imread(p), cv2.ROTATE_90_COUNTERCLOCKWISE) for p in image_paths]
print("Image order:", [os.path.basename(p) for p in image_paths])

# Read, crop, and rotate each image
images = []
for p in image_paths:
    img = cv2.imread(p)
    cropped = crop_to_strict_nonblack_rectangle(img) # 🟢 remove black frame

    # 🟡 Save cropped version next to original
    cropped_path = os.path.splitext(p)[0] + "_cropped.tiff"
    cv2.imwrite(cropped_path, cropped)
    print(f"Saved cropped version: {cropped_path}")

    rotated = cv2.rotate(cropped, cv2.ROTATE_90_COUNTERCLOCKWISE)
    images.append(rotated)


# Try to stitch using OpenCV
stitcher = cv2.Stitcher_create()
status, stitched_rotated = stitcher.stitch(images)

if status == cv2.Stitcher_OK:
    print("✅ Stitching succeeded using OpenCV.")
    # Rotate back 90° clockwise
    full_scan = cv2.rotate(stitched_rotated, cv2.ROTATE_90_CLOCKWISE)
else:
    print(f"⚠️ Stitching failed (Status: {status}). Using top-to-bottom stacking fallback...")

    # 🟢 Flip back to upright orientation first
    upright_images = [cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE) for img in images]

    # Resize to same width for clean stacking
    min_width = min(img.shape[1] for img in upright_images)
    resized_images = [
        cv2.resize(img, (min_width, int(img.shape[0] * min_width / img.shape[1])))
        for img in upright_images
    ]

    # Stack vertically (top-to-bottom)
    stacked = cv2.vconcat(resized_images)

    # This is already upright, no need to rotate again
    full_scan = stacked

# Save result (stitched or stacked)
cv2.imwrite("stitched_experiment_result.tiff", full_scan)
print("✅ Final result saved as stitched_experiment_result.tiff")
