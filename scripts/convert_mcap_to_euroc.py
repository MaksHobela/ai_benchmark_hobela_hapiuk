import csv
import cv2
import numpy as np
from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

# Шляхи
MCAP_DIR = r"D:\huggingface\data\ucu-slam-dataset-v1\sequence-1"
OUTPUT_DIR = r"D:\huggingface\data\ucu-slam-dataset-v1\sequence-1_euroc\mav0"

TOPIC_CAM = "/cam_color/image_raw"
TOPIC_IMU = "/imu"

# Ініціалізація нових типів rosbags 0.11+
typestore = get_typestore(Stores.ROS2_HUMBLE)

cam0_data_dir = Path(OUTPUT_DIR) / "cam0" / "data"
imu0_dir = Path(OUTPUT_DIR) / "imu0"
cam0_data_dir.mkdir(parents=True, exist_ok=True)
imu0_dir.mkdir(parents=True, exist_ok=True)

imu_csv_path = imu0_dir / "data.csv"
cam_csv_path = Path(OUTPUT_DIR) / "cam0" / "data.csv"

print("Початок конвертації...")

with open(imu_csv_path, 'w', newline='') as f_imu, open(cam_csv_path, 'w', newline='') as f_cam:
    writer_imu = csv.writer(f_imu)
    writer_cam = csv.writer(f_cam)

    writer_imu.writerow(["#timestamp [ns]", "w_RS_x [rad s^-1]", "w_RS_y [rad s^-1]", "w_RS_z [rad s^-1]", "a_RS_x [m s^-2]", "a_RS_y [m s^-2]", "a_RS_z [m s^-2]"])
    writer_cam.writerow(["#timestamp [ns]", "filename"])

    with Reader(MCAP_DIR) as reader:
        count_cam = 0
        count_imu = 0

        connections = [x for x in reader.connections if x.topic in (TOPIC_CAM, TOPIC_IMU)]

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

            if connection.topic == TOPIC_IMU:
                ts = timestamp
                wx, wy, wz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
                ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
                writer_imu.writerow([ts, wx, wy, wz, ax, ay, az])
                count_imu += 1

            elif connection.topic == TOPIC_CAM:
                ts = timestamp
                filename = f"{ts}.png"
                img_path = cam0_data_dir / filename

                height, width = msg.height, msg.width
                encoding = msg.encoding

                if encoding == 'rgb8':
                    img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
                    img_data = cv2.cvtColor(img_data, cv2.COLOR_RGB2BGR)
                elif encoding in ('bgr8', '8UC3'):
                    img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
                elif encoding in ('mono8', '8UC1'):
                    img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width))
                else:
                    img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, -1))

                cv2.imwrite(str(img_path), img_data)
                writer_cam.writerow([ts, filename])
                count_cam += 1

                if count_cam % 500 == 0:
                    print(f"Оброблено {count_cam} кадрів камери та {count_imu} записів IMU...")

print(f"\nУспішно! Збережено {count_cam} кадрів камери та {count_imu} записів IMU у {OUTPUT_DIR}")