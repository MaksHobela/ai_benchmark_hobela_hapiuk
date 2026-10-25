from rosbags.rosbag2 import Reader
from rosbags.typesys import get_types_from_msg, register_types

bagpath = r"D:\huggingface\data\ucu-slam-dataset-v1\sequence-1"

with Reader(bagpath) as reader:
    print("--- СПИСОК ТОПІКІВ ---")
    for connection in reader.connections:
        print(f"Topic: {connection.topic} | Type: {connection.msgtype} | Count: {connection.msgcount}")

    print("\n--- ЗЧИТУВАННЯ МЕТАДАНИХ КАМЕРИ ---")
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/cam_color/camera_info':
            # Отримуємо тип повідомлення з росбаг-схеми
            msgtype = connection.msgtype
            print(f"Знайдено CameraInfo з типом: {msgtype}")
            print(f"Розмір сирих даних (bytes): {len(rawdata)}")
            break