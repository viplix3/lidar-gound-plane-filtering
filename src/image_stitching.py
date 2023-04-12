import os
import cv2
import numpy as np

source1_dir = "../assets/random_seed/"
source1_name = "random seed"

source2_dir = "../assets/surface_normals_seed/"
source2_name = "surface normals seed"

source3_dir = "../assets/horizontal_angle_seed/"
source3_name = "horizontal angle seed"

dest_dir = "../assets/stitched/"
os.makedirs(dest_dir, exist_ok=True)

source1_files = sorted(os.listdir(source1_dir))
source2_files = sorted(os.listdir(source2_dir))
source3_files = sorted(os.listdir(source3_dir))

names = [
    "plane fitted (t)",
    "used estimated plane (t-1)",
    "used estimated plane (t-2)",
    "used estimated plane (t-3)",
    "used estimated plane (t-4)",
]


for idx, (src1, src2, src3) in enumerate(
    zip(source1_files, source2_files, source3_files)
):
    src1_img = cv2.imread(os.path.join(source1_dir, src1))
    src2_img = cv2.imread(os.path.join(source2_dir, src2))
    src3_img = cv2.imread(os.path.join(source3_dir, src3))

    cv2.putText(
        src1_img,
        f"{source1_name} {names[idx]}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (50, 255, 0),
        2,
    )
    cv2.putText(
        src2_img,
        f"{source2_name} {names[idx]}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (50, 255, 0),
        2,
    )
    cv2.putText(
        src3_img,
        f"{source3_name} {names[idx]}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (50, 255, 0),
        2,
    )

    max_shape = min(src1_img.shape, src2_img.shape, src3_img.shape)

    src1_img = cv2.resize(src1_img, (max_shape[1], max_shape[0]))
    src2_img = cv2.resize(src2_img, (max_shape[1], max_shape[0]))
    src3_img = cv2.resize(src3_img, (max_shape[1], max_shape[0]))

    merged_1_2 = np.hstack((src1_img, src2_img))
    merged_1_2_3 = np.hstack((merged_1_2, src3_img))

    merged_1_2_3 = cv2.resize(merged_1_2_3, (2296, 598))
    cv2.imwrite(
        os.path.join(dest_dir, names[idx].replace(" ", "_") + ".png"), merged_1_2_3
    )
    print(f"Saved {names[idx]}")
