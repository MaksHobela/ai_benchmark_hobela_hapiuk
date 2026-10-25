#!/bin/bash

# ==============================================================================
# Automated SLAM Benchmark Pipeline: DSO vs ORB-SLAM3
# ==============================================================================

PROJECT_DIR="/root/project"
RESULTS_DIR="${PROJECT_DIR}/results"

# ------------------------------------------------------------------------------
# НАЛАШТУВАННЯ ДЛЯ НОВОГО ДАТАСЕТУ / СІКВЕНСУ
# ------------------------------------------------------------------------------
SEQ_NAME="fr1_desk" # Замінити на назву вашого нового датасету/сіквенсу

# Шлях до папки з новим датасетом
DATASET_PATH="${PROJECT_DIR}/Datasets/TUM/rgbd_dataset_freiburg1_desk"

# Шляхи до файлів калібрування камери
CALIB_DSO="${PROJECT_DIR}/camera_tum_fr1.txt" # Файл калібрування для DSO
CALIB_ORBSLAM="${ORB_DIR:-/root/ORB_SLAM3}/Examples/Monocular/TUM1.yaml" # Конфіг YAML для ORB-SLAM3

# Шляхи до службових файлів всередині датасету
TIMES_FILE="${DATASET_PATH}/times.txt"
GT_FILE="${DATASET_PATH}/groundtruth.txt"
# ------------------------------------------------------------------------------

mkdir -p ${RESULTS_DIR}
export DISPLAY=${DISPLAY:-:0}

echo "=========================================="
echo " Starting Automated SLAM Benchmark Pipeline: ${SEQ_NAME}"
echo "=========================================="

# ----------------------------------------------
# 1. RUNNING DSO
# ----------------------------------------------
echo "[1/2] Running DSO on ${SEQ_NAME}..."
cd ${PROJECT_DIR}
rm -f result.txt ${RESULTS_DIR}/traj_dso_${SEQ_NAME}.txt ${RESULTS_DIR}/traj_dso_${SEQ_NAME}_formatted.txt

/root/dso/build/bin/dso_dataset \
    files=${DATASET_PATH}/rgb \
    calib=${CALIB_DSO} \
    preset=0 \
    mode=1 \
    sampleoutput=1 > ${RESULTS_DIR}/dso_${SEQ_NAME}_run.log 2>&1 || true

if [ -f "${PROJECT_DIR}/result.txt" ]; then
    cp ${PROJECT_DIR}/result.txt ${RESULTS_DIR}/traj_dso_${SEQ_NAME}.txt
    
    python3 -c "
import os

times_path = '${TIMES_FILE}'
in_path = '${RESULTS_DIR}/traj_dso_${SEQ_NAME}.txt'
out_path = '${RESULTS_DIR}/traj_dso_${SEQ_NAME}_formatted.txt'

if os.path.exists(times_path):
    with open(times_path) as f_times:
        timestamps = [line.strip().split()[0] for line in f_times if line.strip() and not line.startswith('#')]

    with open(in_path) as f_in, open(out_path, 'w') as f_out:
        for idx, line in enumerate(f_in):
            parts = line.strip().split()
            if len(parts) >= 8 and idx < len(timestamps):
                parts[0] = timestamps[idx]
                f_out.write(' '.join(parts[:8]) + '\n')
"
    echo "  -> DSO trajectory generated & formatted successfully."
else
    echo "  [ERROR] DSO failed to generate result.txt"
fi

# ----------------------------------------------
# 2. RUNNING ORB-SLAM3
# ----------------------------------------------
echo "[2/2] Running ORB-SLAM3 on ${SEQ_NAME}..."

ORB_DIR=""
if [ -d "/root/ORB_SLAM3" ]; then
    ORB_DIR="/root/ORB_SLAM3"
elif [ -d "${PROJECT_DIR}/ORB_SLAM3" ]; then
    ORB_DIR="${PROJECT_DIR}/ORB_SLAM3"
fi

if [ -n "$ORB_DIR" ]; then
    cd ${ORB_DIR}
    rm -f FrameTrajectory_TUM.txt KeyFrameTrajectory_TUM.txt KeyFrameTrajectory.txt ${RESULTS_DIR}/traj_orbslam3_${SEQ_NAME}.txt

    ./Examples/Monocular/mono_tum \
        ./Vocabulary/ORBvoc.txt \
        ${CALIB_ORBSLAM} \
        ${DATASET_PATH} > ${RESULTS_DIR}/orbslam3_${SEQ_NAME}_run.log 2>&1 || true

    ORB_OUT=""
    if [ -f "FrameTrajectory_TUM.txt" ]; then
        ORB_OUT="FrameTrajectory_TUM.txt"
    elif [ -f "KeyFrameTrajectory_TUM.txt" ]; then
        ORB_OUT="KeyFrameTrajectory_TUM.txt"
    elif [ -f "KeyFrameTrajectory.txt" ]; then
        ORB_OUT="KeyFrameTrajectory.txt"
    fi

    if [ -n "$ORB_OUT" ]; then
        cp "$ORB_OUT" ${RESULTS_DIR}/traj_orbslam3_${SEQ_NAME}.txt
        echo "  -> ORB-SLAM3 trajectory generated successfully ($ORB_OUT)."
    else
        echo "  [ERROR] ORB-SLAM3 failed to generate trajectory file."
    fi
else
    echo "  [ERROR] ORB-SLAM3 directory not found!"
fi

# ----------------------------------------------
# 3. BENCHMARKING & COMPARISON SUMMARY
# ----------------------------------------------
echo ""
echo "=========================================="
echo "          BENCHMARK RESULTS SUMMARY       "
echo "=========================================="

RMSE_DSO=""
RMSE_ORB=""

# DSO Metrics
if [ -f "${RESULTS_DIR}/traj_dso_${SEQ_NAME}_formatted.txt" ] && [ -f "${GT_FILE}" ]; then
    echo "------------------------------------------"
    echo " Algorithm: DSO (Direct Sparse Odometry)"
    echo "------------------------------------------"
    evo_ape tum ${GT_FILE} \
                ${RESULTS_DIR}/traj_dso_${SEQ_NAME}_formatted.txt \
                -va --align -s > ${RESULTS_DIR}/evo_dso_${SEQ_NAME}.txt
    
    grep -E "max|mean|median|min|rmse|sse|std" ${RESULTS_DIR}/evo_dso_${SEQ_NAME}.txt
    RMSE_DSO=$(grep "rmse" ${RESULTS_DIR}/evo_dso_${SEQ_NAME}.txt | awk '{print $2}')
fi

# ORB-SLAM3 Metrics
if [ -f "${RESULTS_DIR}/traj_orbslam3_${SEQ_NAME}.txt" ] && [ -f "${GT_FILE}" ]; then
    echo ""
    echo "------------------------------------------"
    echo " Algorithm: ORB-SLAM3 (Monocular)"
    echo "------------------------------------------"
    evo_ape tum ${GT_FILE} \
                ${RESULTS_DIR}/traj_orbslam3_${SEQ_NAME}.txt \
                -va --align -s > ${RESULTS_DIR}/evo_orbslam3_${SEQ_NAME}.txt
    
    grep -E "max|mean|median|min|rmse|sse|std" ${RESULTS_DIR}/evo_orbslam3_${SEQ_NAME}.txt
    RMSE_ORB=$(grep "rmse" ${RESULTS_DIR}/evo_orbslam3_${SEQ_NAME}.txt | awk '{print $2}')
fi

# Final Verdict
echo ""
echo "=========================================="
echo "               VERDICT                    "
echo "=========================================="

python3 -c "
rmse_dso = float('$RMSE_DSO') if '$RMSE_DSO' else None
rmse_orb = float('$RMSE_ORB') if '$RMSE_ORB' else None

if rmse_dso and rmse_orb:
    if rmse_dso < rmse_orb:
        print(f'-> DSO виявився ТОЧНІШИМ (RMSE: {rmse_dso:.6f} m проти {rmse_orb:.6f} m у ORB-SLAM3)')
    else:
        print(f'-> ORB-SLAM3 виявився ТОЧНІШИМ (RMSE: {rmse_orb:.6f} m проти {rmse_dso:.6f} m у DSO)')
elif rmse_dso:
    print(f'-> Отримано результати тільки для DSO (RMSE: {rmse_dso:.6f} m)')
elif rmse_orb:
    print(f'-> Отримано результати тільки для ORB-SLAM3 (RMSE: {rmse_orb:.6f} m)')
else:
    print('-> Не вдалося зняти результати з жодного алгоритму.')
"

echo "=========================================="





# #!/bin/bash

# # ==============================================================================
# # Automated SLAM Benchmark Pipeline: DSO vs ORB-SLAM3
# # ==============================================================================

# PROJECT_DIR="/root/project"
# DATASETS_DIR="${PROJECT_DIR}/Datasets/TUM"
# RESULTS_DIR="${PROJECT_DIR}/results"

# mkdir -p ${RESULTS_DIR}

# export DISPLAY=${DISPLAY:-:0}

# echo "=========================================="
# echo " Starting Automated SLAM Benchmark Pipeline"
# echo "=========================================="

# # ----------------------------------------------
# # 1. RUNNING DSO
# # ----------------------------------------------
# echo "[1/2] Running DSO on fr1_desk..."
# cd ${PROJECT_DIR}
# rm -f result.txt ${RESULTS_DIR}/traj_dso_fr1_desk.txt ${RESULTS_DIR}/traj_dso_fr1_desk_formatted.txt

# # Передаємо саме папку /rgb для кадрів
# /root/dso/build/bin/dso_dataset \
#     files=${DATASETS_DIR}/rgbd_dataset_freiburg1_desk/rgb \
#     calib=${PROJECT_DIR}/camera_tum_fr1.txt \
#     preset=0 \
#     mode=1 \
#     sampleoutput=1 > ${RESULTS_DIR}/dso_run.log 2>&1 || true

# if [ -f "${PROJECT_DIR}/result.txt" ]; then
#     cp ${PROJECT_DIR}/result.txt ${RESULTS_DIR}/traj_dso_fr1_desk.txt
    
#     python3 -c '
# with open("/root/project/Datasets/TUM/rgbd_dataset_freiburg1_desk/times.txt") as f_times:
#     timestamps = [line.strip().split()[0] for line in f_times if line.strip() and not line.startswith("#")]

# with open("/root/project/results/traj_dso_fr1_desk.txt") as f_in, open("/root/project/results/traj_dso_fr1_desk_formatted.txt", "w") as f_out:
#     for idx, line in enumerate(f_in):
#         parts = line.strip().split()
#         if len(parts) >= 8 and idx < len(timestamps):
#             parts[0] = timestamps[idx]
#             f_out.write(" ".join(parts[:8]) + "\n")
# '
#     echo "  -> DSO trajectory generated & formatted successfully."
# else
#     echo "  [ERROR] DSO failed to generate result.txt"
# fi

# # ----------------------------------------------
# # 2. RUNNING ORB-SLAM3
# # ----------------------------------------------
# echo "[2/2] Running ORB-SLAM3 on fr1_desk..."

# ORB_DIR=""
# if [ -d "/root/ORB_SLAM3" ]; then
#     ORB_DIR="/root/ORB_SLAM3"
# elif [ -d "${PROJECT_DIR}/ORB_SLAM3" ]; then
#     ORB_DIR="${PROJECT_DIR}/ORB_SLAM3"
# fi

# if [ -n "$ORB_DIR" ]; then
#     cd ${ORB_DIR}
#     rm -f FrameTrajectory_TUM.txt KeyFrameTrajectory_TUM.txt KeyFrameTrajectory.txt ${RESULTS_DIR}/traj_orbslam3_fr1_desk.txt

#     ./Examples/Monocular/mono_tum \
#         ./Vocabulary/ORBvoc.txt \
#         ./Examples/Monocular/TUM1.yaml \
#         ${DATASETS_DIR}/rgbd_dataset_freiburg1_desk > ${RESULTS_DIR}/orbslam3_run.log 2>&1 || true

#     # Шукаємо згенеровану траєкторію
#     ORB_OUT=""
#     if [ -f "FrameTrajectory_TUM.txt" ]; then
#         ORB_OUT="FrameTrajectory_TUM.txt"
#     elif [ -f "KeyFrameTrajectory_TUM.txt" ]; then
#         ORB_OUT="KeyFrameTrajectory_TUM.txt"
#     elif [ -f "KeyFrameTrajectory.txt" ]; then
#         ORB_OUT="KeyFrameTrajectory.txt"
#     fi

#     if [ -n "$ORB_OUT" ]; then
#         cp "$ORB_OUT" ${RESULTS_DIR}/traj_orbslam3_fr1_desk.txt
#         echo "  -> ORB-SLAM3 trajectory generated successfully ($ORB_OUT)."
#     else
#         echo "  [ERROR] ORB-SLAM3 failed to generate trajectory file."
#     fi
# else
#     echo "  [ERROR] ORB-SLAM3 directory not found!"
# fi

# # ----------------------------------------------
# # 3. BENCHMARKING & COMPARISON SUMMARY
# # ----------------------------------------------
# echo ""
# echo "=========================================="
# echo "          BENCHMARK RESULTS SUMMARY        "
# echo "=========================================="

# RMSE_DSO=""
# RMSE_ORB=""

# # DSO Metrics
# if [ -f "${RESULTS_DIR}/traj_dso_fr1_desk_formatted.txt" ]; then
#     echo "------------------------------------------"
#     echo " Algorithm: DSO (Direct Sparse Odometry)"
#     echo "------------------------------------------"
#     evo_ape tum ${DATASETS_DIR}/rgbd_dataset_freiburg1_desk/groundtruth.txt \
#                 ${RESULTS_DIR}/traj_dso_fr1_desk_formatted.txt \
#                 -va --align -s > ${RESULTS_DIR}/evo_dso_fr1_desk.txt
    
#     grep -E "max|mean|median|min|rmse|sse|std" ${RESULTS_DIR}/evo_dso_fr1_desk.txt
#     RMSE_DSO=$(grep "rmse" ${RESULTS_DIR}/evo_dso_fr1_desk.txt | awk '{print $2}')
# fi

# # ORB-SLAM3 Metrics
# if [ -f "${RESULTS_DIR}/traj_orbslam3_fr1_desk.txt" ]; then
#     echo ""
#     echo "------------------------------------------"
#     echo " Algorithm: ORB-SLAM3 (Monocular)"
#     echo "------------------------------------------"
#     evo_ape tum ${DATASETS_DIR}/rgbd_dataset_freiburg1_desk/groundtruth.txt \
#                 ${RESULTS_DIR}/traj_orbslam3_fr1_desk.txt \
#                 -va --align -s > ${RESULTS_DIR}/evo_orbslam3_fr1_desk.txt
    
#     grep -E "max|mean|median|min|rmse|sse|std" ${RESULTS_DIR}/evo_orbslam3_fr1_desk.txt
#     RMSE_ORB=$(grep "rmse" ${RESULTS_DIR}/evo_orbslam3_fr1_desk.txt | awk '{print $2}')
# fi

# # Final Verdict
# echo ""
# echo "=========================================="
# echo "               VERDICT                    "
# echo "=========================================="

# python3 -c "
# rmse_dso = float('$RMSE_DSO') if '$RMSE_DSO' else None
# rmse_orb = float('$RMSE_ORB') if '$RMSE_ORB' else None

# if rmse_dso and rmse_orb:
#     if rmse_dso < rmse_orb:
#         print(f'-> DSO виявився ТОЧНІШИМ (RMSE: {rmse_dso:.6f} m проти {rmse_orb:.6f} m у ORB-SLAM3)')
#     else:
#         print(f'-> ORB-SLAM3 виявився ТОЧНІШИМ (RMSE: {rmse_orb:.6f} m проти {rmse_dso:.6f} m у DSO)')
# elif rmse_dso:
#     print(f'-> Отримано результати тільки для DSO (RMSE: {rmse_dso:.6f} m)')
# elif rmse_orb:
#     print(f'-> Отримано результати тільки для ORB-SLAM3 (RMSE: {rmse_orb:.6f} m)')
# else:
#     print('-> Не вдалося зняти результати з жодного алгоритму.')
# "

# echo "=========================================="