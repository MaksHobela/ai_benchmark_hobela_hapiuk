import numpy as np
from scipy.spatial.transform import Rotation as R

# Трансформація base_link (imu) -> color з вашого MCAP
t_imu_cam = np.array([0.050964124466524936, 0.049003980744463355, -0.004907455166771748])
q_imu_cam = np.array([0.018080433108374414, 0.7073564435614564, 0.706623766341941, 0.0016770603546815172]) # x,y,z,w

rot_imu_cam = R.from_quat(q_imu_cam).as_matrix()

# OpenVINS використовує T_C_to_I (Rot_CtoI, p_CinI)
rot_cam_imu = rot_imu_cam.T
p_cam_in_imu = -rot_cam_imu @ t_imu_cam

print("=== ОБЧИСЛЕНІ ЕКСТРИНСИКИ ДЛЯ OPENVINS ===")
print("R_CtoI (3x3 matrix flat):")
print(rot_cam_imu.flatten().tolist())
print("\np_CinI (translation):")
print(p_cam_in_imu.tolist())