from pathlib import Path
import cv2
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
# from rosbags.typesys import get_typesfrommsg, Stores, get_typestore

# Вкажіть шлях до ПАПКИ з датасетом
BAG_PATH = Path(r"D:\huggingface\data\ucu-slam-dataset-v1\sequence-1")

# Ініціалізація бази типів ROS 2 (Humble/Jazzy)
typestore = get_typestore(Stores.ROS2_HUMBLE)

with Reader(BAG_PATH) as reader:
    print("=== СПИСОК ТОПІКІВ У ДАТАСЕТІ ===")
    for connection in reader.connections:
        print(f"Topic: {connection.topic:<32} | Type: {connection.msgtype}")

    # 1. Читання зображення RGB
    cam_connections = [c for c in reader.connections if c.topic == '/cam_color/image_raw']
    if cam_connections:
        for conn, timestamp, rawdata in reader.messages(connections=cam_connections):
            # Новий спосіб десеріалізації
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            
            # Декодування пікселів
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            
            cv2.imwrite("first_frame.jpg", img_bgr)
            print("\n[УСПІХ] Перший кадр збережено як 'first_frame.jpg'!")
            break

    # 2. Читання IMU
    imu_connections = [c for c in reader.connections if c.topic == '/imu']
    if imu_connections:
        for conn, timestamp, rawdata in reader.messages(connections=imu_connections):
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            print("\n=== ДАНІ IMU (ПЕРШИЙ ЗАПИС) ===")
            print(f"Linear Accel -> X: {msg.linear_acceleration.x:.3f}, Y: {msg.linear_acceleration.y:.3f}, Z: {msg.linear_acceleration.z:.3f}")
            print(f"Gyro         -> X: {msg.angular_velocity.x:.3f}, Y: {msg.angular_velocity.y:.3f}, Z: {msg.angular_velocity.z:.3f}")
            break







# from pathlib import Path
# import cv2
# import numpy as np
# from rosbags.rosbag2 import Reader
# from rosbags.serde import deserialize_cdr

# # Вкажіть шлях до ПАПКИ, де лежать sequence-1.mcap та metadata.yaml
# BAG_PATH = Path(r"D:\huggingface\data\ucu-slam-dataset-v1\sequence-1")

# with Reader(BAG_PATH) as reader:
#     # 1. Виведення всіх топіків та кількості повідомлень у них
#     print("=== СПИСОК ТОПІКІВ У ДАТАСЕТІ ===")
#     for connection in reader.connections:
#         print(f"Topic: {connection.topic:<30} | Type: {connection.msgtype}")

#     # 2. Збереження першого кадрового зображення з RGB-камери
#     cam_connections = [c for c in reader.connections if c.topic == '/cam_color/image_raw']
#     if cam_connections:
#         for conn, timestamp, rawdata in reader.messages(connections=cam_connections):
#             msg = deserialize_cdr(rawdata, conn.msgtype)
            
#             # Декодування пікселів у матрицю OpenCV
#             img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
#             img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            
#             # Збереження картинки на диск
#             cv2.imwrite("first_frame.jpg", img_bgr)
#             print("\n[УСПІХ] Перший кадр збережено у файл 'first_frame.jpg'!")
#             break

#     # 3. Перевірка перших показників з IMU-датчика
#     imu_connections = [c for c in reader.connections if c.topic == '/imu']
#     if imu_connections:
#         for conn, timestamp, rawdata in reader.messages(connections=imu_connections):
#             msg = deserialize_cdr(rawdata, conn.msgtype)
#             print("\n=== ДАНІ IMU (ПЕРШИЙ ЗАПИС) ===")
#             print(f"Прискорення (Linear Accel)  -> X: {msg.linear_acceleration.x:.3f}, Y: {msg.linear_acceleration.y:.3f}, Z: {msg.linear_acceleration.z:.3f}")
#             print(f"Кутова швидкість (Gyro)     -> X: {msg.angular_velocity.x:.3f}, Y: {msg.angular_velocity.y:.3f}, Z: {msg.angular_velocity.z:.3f}")
#             break