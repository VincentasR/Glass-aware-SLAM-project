import cv2
import csv
import yaml
import numpy as np

# === CONFIGURATION ===
map_yaml_path = "my_map.yaml"
csv_path = "glass_detections_filtered.csv"
output_map = "my_map_with_filtered_glass.pgm"
output_debug = "my_map_with_filtered_glass_debug.png"

# === LOAD MAP METADATA ===
with open(map_yaml_path, 'r') as f:
    map_metadata = yaml.safe_load(f)

map_img = cv2.imread(map_metadata['image'], cv2.IMREAD_GRAYSCALE)
resolution = map_metadata['resolution']
origin = map_metadata['origin']
debug_img = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)

# === LOAD FILTERED DETECTIONS ===
with open(csv_path, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        try:
            if row['label'].lower() != 'glass':
                continue

            x1 = float(row['ext_x1_global'])
            y1 = float(row['ext_y1_global'])
            x2 = float(row['ext_x2_global'])
            y2 = float(row['ext_y2_global'])

            x1m = int((x1 - origin[0]) / resolution)
            y1m = map_img.shape[0] - int((y1 - origin[1]) / resolution)
            x2m = int((x2 - origin[0]) / resolution)
            y2m = map_img.shape[0] - int((y2 - origin[1]) / resolution)

            if all(0 <= val < map_img.shape[1] for val in [x1m, x2m]) and all(0 <= val < map_img.shape[0] for val in [y1m, y2m]):
                if map_img[y1m, x1m] > 250 or map_img[y2m, x2m] > 250:
                    cv2.line(map_img, (x1m, y1m), (x2m, y2m), 0, 1)  # black wall line
                cv2.line(debug_img, (x1m, y1m), (x2m, y2m), (0, 0, 255), 1)
            else:
                print(f"⚠️ Out of bounds line: ({x1m}, {y1m}) to ({x2m}, {y2m})")

        except Exception as e:
            print(f"❌ Error drawing line: {e}")

cv2.imwrite(output_map, map_img)
cv2.imwrite(output_debug, debug_img)
print(f"✅ Saved: {output_map} and {output_debug}")
