import os
import subprocess

DATASETS = {
    "fr1_desk": "https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz",
    "fr1_room": "https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_room.tgz",
    "fr2_large_with_loop": "https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_large_with_loop.tgz",
    "fr2_360_kidnap": "https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_360_kidnap.tgz",
    "fr3_long_office": "https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.tgz"
}

# BASE_DIR = "/mnt/d/courses/AI/project/Datasets/TUM"
BASE_DIR = "/root/project/Datasets/TUM"
os.makedirs(BASE_DIR, exist_ok=True)
os.chdir(BASE_DIR)

print("=== ПОЧАТОК ЗАВАНТАЖЕННЯ ДАТАСЕТІВ ===")

for name, url in DATASETS.items():
    tar_file = f"{name}.tgz"
    print(f"\n[+] Завантаження: {name} ...")
    subprocess.run(f"wget -c -O {tar_file} {url}", shell=True, check=True)
    
    print(f"[+] Розпакування: {name} ...")
    subprocess.run(f"tar -xvzf {tar_file}", shell=True, check=True)
    os.remove(tar_file)

print("\n=== УСІ ДАТАСЕТИ УСПІШНО ЗАВАНТАЖЕНІ ТА РОЗПАКОВАНІ ===")
