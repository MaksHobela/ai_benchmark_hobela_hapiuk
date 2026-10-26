#!/bin/bash

# ==============================================================================
# Automated SLAM Benchmark Pipeline: DSO vs ORB-SLAM3
# ==============================================================================
# Works in two places without edits:
#   - Colab (root): the old paths /root/project, /root/dso, /root/ORB_SLAM3
#   - local WSL/Ubuntu (normal user): repo folder, ~/dso, ~/ORB_SLAM3, ~/datasets
# Every path can be overridden with an environment variable, e.g.
#   DATASET_PATH=~/datasets/rgbd_dataset_freiburg1_xyz SEQ_NAME=fr1_xyz bash scripts/run_benchmark.sh
# ==============================================================================

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH="${HOME}/.local/bin:${PATH}" # evo_ape from pipx, also when started from PowerShell

# Colab layout if /root/project is usable, otherwise this repository
if [ -z "${PROJECT_DIR}" ]; then
    if [ -d "/root/project" ] && [ -w "/root/project" ]; then
        PROJECT_DIR="/root/project"
    else
        PROJECT_DIR="${REPO_DIR}"
    fi
fi
RESULTS_DIR="${RESULTS_DIR:-${PROJECT_DIR}/results}"

# ------------------------------------------------------------------------------
# НАЛАШТУВАННЯ ДЛЯ НОВОГО ДАТАСЕТУ / СІКВЕНСУ
# ------------------------------------------------------------------------------
SEQ_NAME="${SEQ_NAME:-fr1_desk}" # Замінити на назву вашого нового датасету/сіквенсу

# Шлях до папки з новим датасетом (Colab: Datasets/TUM у проєкті; локально: ~/datasets)
if [ -z "${DATASET_PATH}" ]; then
    DATASET_PATH="${PROJECT_DIR}/Datasets/TUM/rgbd_dataset_freiburg1_desk"
    [ -d "${DATASET_PATH}" ] || DATASET_PATH="${HOME}/datasets/rgbd_dataset_freiburg1_desk"
fi

# Шляхи до файлів калібрування камери
if [ -z "${CALIB_DSO}" ]; then
    CALIB_DSO="${PROJECT_DIR}/camera_tum_fr1.txt"
    [ -f "${CALIB_DSO}" ] || CALIB_DSO="${REPO_DIR}/config/camera_tum_fr1.txt"
fi

# Де шукати зібрані програми
if [ -z "${ORB_DIR}" ]; then
    for d in /root/ORB_SLAM3 "${PROJECT_DIR}/ORB_SLAM3" "${HOME}/ORB_SLAM3"; do
        if [ -d "$d" ]; then ORB_DIR="$d"; break; fi
    done
fi
CALIB_ORBSLAM="${CALIB_ORBSLAM:-${ORB_DIR}/Examples/Monocular/TUM1.yaml}" # Конфіг YAML для ORB-SLAM3

if [ -z "${DSO_BIN}" ]; then
    for b in /root/dso/build/bin/dso_dataset "${HOME}/dso/build/bin/dso_dataset" /opt/slam/bin/dso_dataset; do
        if [ -x "$b" ]; then DSO_BIN="$b"; break; fi
    done
fi

# Шляхи до службових файлів всередині датасету
TIMES_FILE="${DATASET_PATH}/times.txt"
GT_FILE="${DATASET_PATH}/groundtruth.txt"
# ------------------------------------------------------------------------------

mkdir -p "${RESULTS_DIR}"
export DISPLAY=${DISPLAY:-:0}

echo "=========================================="
echo " Starting Automated SLAM Benchmark Pipeline: ${SEQ_NAME}"
echo "=========================================="
echo " project : ${PROJECT_DIR}"
echo " dataset : ${DATASET_PATH}"
echo " DSO     : ${DSO_BIN:-not found}"
echo " ORB     : ${ORB_DIR:-not found}"
echo " results : ${RESULTS_DIR}"
echo "------------------------------------------"

if [ ! -f "${GT_FILE}" ]; then
    echo "  [ERROR] No dataset at ${DATASET_PATH} (groundtruth.txt missing)."
    echo "          Download it:  python3 scripts/download_tum.py  or set DATASET_PATH=..."
fi

# times.txt is not part of TUM; DSO and the formatter below need it (one timestamp per frame)
# DSO format: "frame_id timestamp exposure". A one-column file (timestamp only) makes DSO
# read garbage times (e.g. 0.79172 instead of 1305031452.791720), so it is rewritten.
if [ -f "${DATASET_PATH}/rgb.txt" ]; then
    if [ ! -f "${TIMES_FILE}" ] || [ "$(head -n 1 "${TIMES_FILE}" | wc -w)" -lt 2 ]; then
        grep -v '^#' "${DATASET_PATH}/rgb.txt" | awk '{printf "%d %s 0.0\n", NR-1, $1}' > "${TIMES_FILE}" \
            && echo "  -> wrote ${TIMES_FILE} (DSO format: id timestamp exposure)"
    fi
fi

# ----------------------------------------------
# 1. RUNNING DSO
# ----------------------------------------------
echo "[1/2] Running DSO on ${SEQ_NAME}..."
WORK_DIR="${RESULTS_DIR}/dso_work_${SEQ_NAME}"
mkdir -p "${WORK_DIR}"
cd "${WORK_DIR}" || exit 1
rm -f result.txt "${RESULTS_DIR}/traj_dso_${SEQ_NAME}.txt" "${RESULTS_DIR}/traj_dso_${SEQ_NAME}_formatted.txt"

if [ -n "${DSO_BIN}" ]; then
    "${DSO_BIN}" \
        files="${DATASET_PATH}/rgb" \
        calib="${CALIB_DSO}" \
        preset=0 \
        mode=1 \
        nogui=1 \
        sampleoutput=1 > "${RESULTS_DIR}/dso_${SEQ_NAME}_run.log" 2>&1 || true
else
    echo "  [ERROR] dso_dataset not found (build it: bash scripts/build_dso.sh, or set DSO_BIN=...)"
fi

if [ -f "${WORK_DIR}/result.txt" ]; then
    cp "${WORK_DIR}/result.txt" "${RESULTS_DIR}/traj_dso_${SEQ_NAME}.txt"

    python3 -c "
import bisect
import os

times_path = '${TIMES_FILE}'
in_path = '${RESULTS_DIR}/traj_dso_${SEQ_NAME}.txt'
out_path = '${RESULTS_DIR}/traj_dso_${SEQ_NAME}_formatted.txt'

# real frame timestamps: 2nd column of a DSO-format times.txt, else the 1st
stamps = []
if os.path.exists(times_path):
    with open(times_path) as f_times:
        for line in f_times:
            p = line.split()
            if p and not line.startswith('#'):
                stamps.append(p[1] if len(p) >= 2 else p[0])
vals = [float(t) for t in stamps]

# DSO writes only keyframes, so line N is NOT frame N: match each pose to the
# frame whose timestamp equals the one DSO printed
with open(in_path) as f_in, open(out_path, 'w') as f_out:
    for line in f_in:
        parts = line.split()
        if len(parts) < 8 or not vals:
            continue
        t = float(parts[0])
        i = min(max(bisect.bisect_left(vals, t), 1), len(vals) - 1)
        j = i if abs(vals[i] - t) < abs(vals[i - 1] - t) else i - 1
        if abs(vals[j] - t) < 1e-3:
            parts[0] = stamps[j]
            f_out.write(' '.join(parts[:8]) + chr(10))
"
    echo "  -> DSO trajectory generated & formatted successfully."
elif [ -n "${DSO_BIN}" ]; then
    echo "  [ERROR] DSO failed to generate result.txt (see ${RESULTS_DIR}/dso_${SEQ_NAME}_run.log)"
fi

# ----------------------------------------------
# 2. RUNNING ORB-SLAM3
# ----------------------------------------------
echo "[2/2] Running ORB-SLAM3 on ${SEQ_NAME}..."

if [ -n "$ORB_DIR" ] && [ -x "${ORB_DIR}/Examples/Monocular/mono_tum" ]; then
    cd "${ORB_DIR}" || exit 1
    rm -f FrameTrajectory_TUM.txt KeyFrameTrajectory_TUM.txt KeyFrameTrajectory.txt "${RESULTS_DIR}/traj_orbslam3_${SEQ_NAME}.txt"

    ./Examples/Monocular/mono_tum \
        ./Vocabulary/ORBvoc.txt \
        "${CALIB_ORBSLAM}" \
        "${DATASET_PATH}" > "${RESULTS_DIR}/orbslam3_${SEQ_NAME}_run.log" 2>&1 || true

    ORB_OUT=""
    if [ -f "FrameTrajectory_TUM.txt" ]; then
        ORB_OUT="FrameTrajectory_TUM.txt"
    elif [ -f "KeyFrameTrajectory_TUM.txt" ]; then
        ORB_OUT="KeyFrameTrajectory_TUM.txt"
    elif [ -f "KeyFrameTrajectory.txt" ]; then
        ORB_OUT="KeyFrameTrajectory.txt"
    fi

    if [ -n "$ORB_OUT" ]; then
        cp "$ORB_OUT" "${RESULTS_DIR}/traj_orbslam3_${SEQ_NAME}.txt"
        echo "  -> ORB-SLAM3 trajectory generated successfully ($ORB_OUT)."
    else
        echo "  [ERROR] ORB-SLAM3 failed to generate trajectory file (see ${RESULTS_DIR}/orbslam3_${SEQ_NAME}_run.log)"
    fi
else
    echo "  [ERROR] ORB-SLAM3 not found (looked in /root/ORB_SLAM3, ${PROJECT_DIR}/ORB_SLAM3, ~/ORB_SLAM3; or set ORB_DIR=...)"
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

if ! command -v evo_ape > /dev/null 2>&1; then
    echo "  [ERROR] evo_ape not found (pipx install evo, or run with: sudo env \"PATH=\$PATH\" ...)"
fi

# DSO Metrics
if [ -f "${RESULTS_DIR}/traj_dso_${SEQ_NAME}_formatted.txt" ] && [ -f "${GT_FILE}" ]; then
    echo "------------------------------------------"
    echo " Algorithm: DSO (Direct Sparse Odometry)"
    echo "------------------------------------------"
    evo_ape tum "${GT_FILE}" \
                "${RESULTS_DIR}/traj_dso_${SEQ_NAME}_formatted.txt" \
                -va --align -s > "${RESULTS_DIR}/evo_dso_${SEQ_NAME}.txt"

    grep -E "max|mean|median|min|rmse|sse|std" "${RESULTS_DIR}/evo_dso_${SEQ_NAME}.txt"
    RMSE_DSO=$(grep "rmse" "${RESULTS_DIR}/evo_dso_${SEQ_NAME}.txt" | awk '{print $2}')
fi

# ORB-SLAM3 Metrics
if [ -f "${RESULTS_DIR}/traj_orbslam3_${SEQ_NAME}.txt" ] && [ -f "${GT_FILE}" ]; then
    echo ""
    echo "------------------------------------------"
    echo " Algorithm: ORB-SLAM3 (Monocular)"
    echo "------------------------------------------"
    evo_ape tum "${GT_FILE}" \
                "${RESULTS_DIR}/traj_orbslam3_${SEQ_NAME}.txt" \
                -va --align -s > "${RESULTS_DIR}/evo_orbslam3_${SEQ_NAME}.txt"

    grep -E "max|mean|median|min|rmse|sse|std" "${RESULTS_DIR}/evo_orbslam3_${SEQ_NAME}.txt"
    RMSE_ORB=$(grep "rmse" "${RESULTS_DIR}/evo_orbslam3_${SEQ_NAME}.txt" | awk '{print $2}')
fi

# Final Verdict
echo ""
echo "=========================================="
echo "               VERDICT                    "
echo "=========================================="

python3 -c "
rmse_dso = float('$RMSE_DSO') if '$RMSE_DSO' else None
rmse_orb = float('$RMSE_ORB') if '$RMSE_ORB' else None

if rmse_dso is not None and rmse_orb is not None:
    if rmse_dso < rmse_orb:
        print(f'-> DSO виявився ТОЧНІШИМ (RMSE: {rmse_dso:.6f} m проти {rmse_orb:.6f} m у ORB-SLAM3)')
    else:
        print(f'-> ORB-SLAM3 виявився ТОЧНІШИМ (RMSE: {rmse_orb:.6f} m проти {rmse_dso:.6f} m у DSO)')
elif rmse_dso is not None:
    print(f'-> Отримано результати тільки для DSO (RMSE: {rmse_dso:.6f} m)')
elif rmse_orb is not None:
    print(f'-> Отримано результати тільки для ORB-SLAM3 (RMSE: {rmse_orb:.6f} m)')
else:
    print('-> Не вдалося зняти результати з жодного алгоритму.')
"

echo "=========================================="
