import cv2
import numpy as np
import os

def stitch_pair(img1, img2):
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    orb = cv2.ORB_create(5000)
    kp1, des1 = orb.detectAndCompute(gray1, None)
    kp2, des2 = orb.detectAndCompute(gray2, None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    matches = bf.knnMatch(des1, des2, k=2)

    # Ratio test
    good_matches = []
    for m, n in matches:
        if m.distance < 0.75 * n.distance:
            good_matches.append(m)

    if len(good_matches) < 10:
        print("[WARN] Not enough matches, returning base image.")
        return img1

    pts1 = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    pts2 = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

    H, mask = cv2.findHomography(pts2, pts1, cv2.RANSAC)

    if H is None:
        print("[ERROR] Homography failed.")
        return img1

    h1, w1 = img1.shape[:2]
    h2, w2 = img2.shape[:2]
    corners2 = np.float32([[0,0],[0,h2],[w2,h2],[w2,0]]).reshape(-1,1,2)
    warped_corners = cv2.perspectiveTransform(corners2, H)

    all_corners = np.concatenate((np.float32([[[0,0]],[[0,h1]],[[w1,h1]],[[w1,0]]]), warped_corners), axis=0)
    [xmin, ymin] = np.int32(all_corners.min(axis=0).ravel() - 0.5)
    [xmax, ymax] = np.int32(all_corners.max(axis=0).ravel() + 0.5)

    translate = [-xmin, -ymin]
    H_translate = np.array([[1, 0, translate[0]], [0, 1, translate[1]], [0, 0, 1]])

    result = cv2.warpPerspective(img2, H_translate @ H, (xmax - xmin, ymax - ymin))
    result[translate[1]:h1 + translate[1], translate[0]:w1 + translate[0]] = img1

    return result

def inpaint_black_regions(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = (gray == 0).astype(np.uint8) * 255
    return cv2.inpaint(image, mask, 3, cv2.INPAINT_TELEA)



def stitch_row(images):
    stitched = images[0]
    for img in images[1:]:
        stitched = stitch_pair(stitched, img)
    return stitched

def stitch_image_grid(num_rows, num_cols, filename_pattern):
    all_rows = []

    for row in range(num_rows):
        row_images = []
        for col in range(num_cols):
            filename = filename_pattern.format(row=row, col=col)
            if not os.path.exists(filename):
                print(f"[ERROR] File not found: {filename}")
                return None
            img = cv2.imread(filename)
            row_images.append(img)
        stitched_row = stitch_row(row_images)
        all_rows.append(stitched_row)

    final_stitched = all_rows[0]
    for next_row in all_rows[1:]:
        final_stitched = stitch_pair(final_stitched, next_row)

    return final_stitched

# --- CONFIGURATION ---
num_rows = 4      # number of horizontal lines in the document
num_cols = 3      # number of image parts per line
filename_pattern = "grid_pieces/row{row}_col{col}.png"  # naming format

# --- RUN ---
final_result = stitch_image_grid(num_rows, num_cols, filename_pattern)

if final_result is not None:
    final_result = inpaint_black_regions(final_result)
    cv2.imwrite("stitched_document.png", final_result)
    cv2.imshow("Final Stitch", final_result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("[FAIL] Document stitching failed.")
