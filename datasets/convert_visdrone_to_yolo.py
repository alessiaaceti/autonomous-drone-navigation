from pathlib import Path
import shutil

# ============================================================
# VisDrone to YOLO format converter
# ============================================================

PROJECT_DIR = Path.home() / "autonomous-drone-navigation"

SOURCE_DIR = (
    PROJECT_DIR
    / "datasets"
    / "VisDrone2019-DET-val"
    / "VisDrone2019-DET-val"
)

OUTPUT_DIR = (
    PROJECT_DIR
    / "datasets"
    / "visdrone_yolo"
)

IMAGE_DIR = SOURCE_DIR / "images"
ANNOTATION_DIR = SOURCE_DIR / "annotations"

OUTPUT_IMAGE_DIR = OUTPUT_DIR / "images" / "val"
OUTPUT_LABEL_DIR = OUTPUT_DIR / "labels" / "val"

# VisDrone image dimensions are read directly from each image.
# This avoids assuming that every image has the same resolution.

OUTPUT_IMAGE_DIR.mkdir(parents=True, exist_ok=True)
OUTPUT_LABEL_DIR.mkdir(parents=True, exist_ok=True)

image_files = sorted(IMAGE_DIR.glob("*.jpg"))

converted = 0

for image_path in image_files:

    annotation_path = ANNOTATION_DIR / f"{image_path.stem}.txt"

    if not annotation_path.exists():
        print(f"[WARNING] Missing annotation: {annotation_path.name}")
        continue

    # Read image dimensions using OpenCV.
    import cv2

    image = cv2.imread(str(image_path))

    if image is None:
        print(f"[WARNING] Could not read image: {image_path.name}")
        continue

    image_height, image_width = image.shape[:2]

    yolo_lines = []

    with annotation_path.open("r") as file:

        for line in file:

            line = line.strip()

            if not line:
                continue

            values = line.split(",")

            if len(values) != 8:
                continue

            x, y, width, height, score, class_id, truncation, occlusion = map(
                int,
                values
            )

            # Ignore invalid annotations.
            if width <= 0 or height <= 0:
                continue

            # Ignore regions marked as ignored by VisDrone.
            if class_id == 0:
                continue

            # VisDrone classes are 1-10.
            # YOLO classes are 0-9.
            yolo_class_id = class_id - 1

            # Convert top-left coordinates to YOLO center coordinates.
            x_center = x + width / 2
            y_center = y + height / 2

            # Normalize coordinates.
            x_center /= image_width
            y_center /= image_height
            width_normalized = width / image_width
            height_normalized = height / image_height

            yolo_lines.append(
                f"{yolo_class_id} "
                f"{x_center:.6f} "
                f"{y_center:.6f} "
                f"{width_normalized:.6f} "
                f"{height_normalized:.6f}"
            )

    output_label_path = OUTPUT_LABEL_DIR / f"{image_path.stem}.txt"

    output_label_path.write_text(
        "\n".join(yolo_lines) + "\n"
    )

    shutil.copy2(
        image_path,
        OUTPUT_IMAGE_DIR / image_path.name
    )

    converted += 1

print("==========================================")
print("VisDrone conversion completed")
print("==========================================")
print(f"Images converted: {converted}")
print(f"Output directory: {OUTPUT_DIR}")
