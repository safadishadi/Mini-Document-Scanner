import cv2
import os

def split_image_into_grid_with_dynamic_overlap(image_path, rows, cols, overlap_percent=0.3):
    img = cv2.imread(image_path)
    if img is None:
        print(f"[ERROR] Could not read image: {image_path}")
        return

    height, width = img.shape[:2]
    print(f"Original image size: {width}x{height}")

    cell_width = width // cols
    cell_height = height // rows

    overlap_x = int(cell_width * overlap_percent)
    overlap_y = int(cell_height * overlap_percent)

    print(f"Cell size: {cell_width}x{cell_height}, Overlap: {overlap_x}px horizontal, {overlap_y}px vertical")

    os.makedirs("grid_pieces", exist_ok=True)
    tile_count = 0

    for r in range(rows):
        for c in range(cols):
            x_start = max(c * cell_width - overlap_x, 0)
            y_start = max(r * cell_height - overlap_y, 0)

            x_end = min((c + 1) * cell_width + overlap_x, width)
            y_end = min((r + 1) * cell_height + overlap_y, height)

            crop = img[y_start:y_end, x_start:x_end]

            tile_index = r * cols + c
            filename = f"grid_pieces/row{r}_col{c}.png"
            cv2.imwrite(filename, crop)
            print(f"Saved {filename}, shape: {crop.shape[1]}x{crop.shape[0]}")
            tile_count += 1

    print(f"[DONE] Finished splitting image into {tile_count} grid pieces.")

# Usage example:
if __name__ == "__main__":
    image_path = "to_divide_grid.png"  # Change this to your input image     _2
    rows = 4
    cols = 3
    overlap_percent = 0.35  # 35% overlap
    split_image_into_grid_with_dynamic_overlap(image_path, rows, cols, overlap_percent)
