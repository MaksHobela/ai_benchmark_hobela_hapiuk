import numpy as np
from scipy.spatial.transform import Rotation as R
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

MCAP_DIR = r"D:\huggingface\data\ucu-slam-dataset-v1\sequence-1"
typestore = get_typestore(Stores.ROS2_HUMBLE)

found_cam = False
found_tf = False

with Reader(MCAP_DIR) as reader:
    print("=== ПОШУК КАЛІБРУВАЛЬНИХ ДАНИХ З MCAP ===\n")
    
    for connection, timestamp, rawdata in reader.messages():
        # 1. CameraInfo
        if connection.topic == '/cam_color/camera_info' and not found_cam:
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            print("--- CAMERA INTRINSICS & DISTORTION ---")
            print(f"Resolution: {msg.width} x {msg.height}")
            print(f"Distortion Model: {msg.distortion_model}")
            print(f"D (coefficients): {list(msg.d)}")
            print(f"K (flat matrix): {list(msg.k)}")
            print(f"  -> fx: {msg.k[0]}, fy: {msg.k[4]}")
            print(f"  -> cx: {msg.k[2]}, cy: {msg.k[5]}")
            found_cam = True

        # 2. TF Static (Extrinsics)
        elif connection.topic == '/tf_static' and not found_tf:
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            print("\n--- STATIC TRANSFORMS (/tf_static) ---")
            for transform in msg.transforms:
                parent = transform.header.frame_id
                child = transform.child_frame_id
                t = transform.transform.translation
                r = transform.transform.rotation
                print(f"Frame: {parent} -> {child}")
                print(f"  Translation (x,y,z): [{t.x}, {t.y}, {t.z}]")
                print(f"  Rotation Quaternion (x,y,z,w): [{r.x}, {r.y}, {r.z}, {r.w}]")
            found_tf = True

        if found_cam and found_tf:
            break