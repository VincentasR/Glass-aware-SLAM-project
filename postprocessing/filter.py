import cv2
import csv
import yaml
import numpy as np

# === CONFIG ===
map_yaml_path = "my_map.yaml"
csv_path = "glass_detections.csv"
filtered_csv_path = "glass_detections_filtered.csv"
output_debug = "glass_map_filtered_lines_debug.png"

# === Load map + metadata ===
with open(map_yaml_path, 'r') as f:
    meta = yaml.safe_load(f)
map_img = cv2.imread(meta['image'], cv2.IMREAD_GRAYSCALE)
res = meta['resolution']
origin = meta['origin']

h, w = map_img.shape
filtered_lines = []
rejected_lines = []
filtered_rows = []

def is_free_midpoint(xm, ym, radius=3):
    xm, ym = int(xm), int(ym)
    if not (radius <= xm < w - radius and radius <= ym < h - radius):
        return False
    region = map_img[ym-radius:ym+radius+1, xm-radius:xm+radius+1]
    edges = np.concatenate([region[0, :], region[-1, :], region[:, 0], region[:, -1]])
    center = region[radius, radius]
    wall_edge = np.sum(edges < 50) >= 5
    center_free = center > 200
    return wall_edge and center_free

def has_wall_near(x, y, radius=3):
    x, y = int(x), int(y)
    if not (radius <= x < w - radius and radius <= y < h - radius):
        return False
    region = map_img[y-radius:y+radius+1, x-radius:x+radius+1]
    return np.any(region < 50)

# === Load CSV detections ===
with open(csv_path, 'r') as f:
    reader = csv.DictReader(f)
    rows = list(reader)
    header = reader.fieldnames

    for row in rows:
        try:
            if row["label"].lower() != "glass":
                continue

            x1g = float(row["ext_x1_global"])
            y1g = float(row["ext_y1_global"])
            x2g = float(row["ext_x2_global"])
            y2g = float(row["ext_y2_global"])

            x1m = int((x1g - origin[0]) / res)
            y1m = h - int((y1g - origin[1]) / res)
            x2m = int((x2g - origin[0]) / res)
            y2m = h - int((y2g - origin[1]) / res)

            mid_x = (x1m + x2m) // 2
            mid_y = (y1m + y2m) // 2

            # Order left and right by x
            if x1m < x2m:
                left_x, left_y = x1m, y1m
                right_x, right_y = x2m, y2m
            else:
                left_x, left_y = x2m, y2m
                right_x, right_y = x1m, y1m
            wall_radius = 5
            if (
                is_free_midpoint(mid_x, mid_y)
                and has_wall_near(left_x, left_y, radius=wall_radius)
                and has_wall_near(right_x, right_y, radius=wall_radius)
            ):
                filtered_lines.append(((x1m, y1m), (x2m, y2m)))
                filtered_rows.append(row)
            else:
                rejected_lines.append(((x1m, y1m), (x2m, y2m)))

        except Exception as e:
            print(f"❌ Error: {e}")
            continue

# === Write filtered CSV ===
with open(filtered_csv_path, 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=header)
    writer.writeheader()
    for row in filtered_rows:
        writer.writerow(row)

# === Draw lines on debug map ===
debug = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)
for pt1, pt2 in filtered_lines:
    cv2.line(debug, pt1, pt2, (0, 0, 255), 1)  # red = kept
for pt1, pt2 in rejected_lines:
    cv2.line(debug, pt1, pt2, (128, 128, 128), 1)  # gray = rejected

cv2.imwrite(output_debug, debug)
print(f"✅ Saved filtered debug image: {output_debug}")
print(f"✅ Saved filtered CSV: {filtered_csv_path}")
