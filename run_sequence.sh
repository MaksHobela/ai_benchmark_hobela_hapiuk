#!/usr/bin/env bash
# =============================================================================
#  Повний цикл для однієї послідовності UCU SLAM Dataset:
#    1. скачує sequence-N з Hugging Face (якщо ще не розпакована)
#    2. розпаковує .mcap у формат TUM
#    3. проганяє ORB-SLAM3 Monocular RUNS разів
#    4. рахує RMSE кожного прогону і робить спільне відео
#
#  Використання:
#    ./run_sequence.sh N [RUNS]
#    ./run_sequence.sh 1        # послідовність 1, 5 прогонів
#    ./run_sequence.sh 5 3      # послідовність 5, 3 прогони
#
#  Змінні оточення (необов'язково):
#    ORB_DIR   шлях до ORB_SLAM3            (за замовч. ~/ORB_SLAM3)
#    DATA_DIR  папка з даними               (за замовч. ~/datasets)
#    VENV      Python-середовище            (за замовч. ~/venvs/ucu)
#    STEP      кожен N-й кадр у відео       (за замовч. 3)
#    FPS       кадрів/с у відео             (за замовч. 15)
#    FIT       gt | all — масштаб мапи      (за замовч. gt)
#    KEEP_MCAP 1 = не видаляти .mcap після розпаковки
# =============================================================================
set -uo pipefail

N="${1:?Вкажи номер послідовності, напр.: ./run_sequence.sh 1}"
RUNS="${2:-5}"
ORB_DIR="${ORB_DIR:-$HOME/ORB_SLAM3}"
DATA_DIR="${DATA_DIR:-$HOME/datasets}"
VENV="${VENV:-$HOME/venvs/ucu}"
STEP="${STEP:-3}"
FPS="${FPS:-15}"
FIT="${FIT:-gt}"
HERE="$(cd "$(dirname "$0")" && pwd)"

PY="$VENV/bin/python"
BIN="$ORB_DIR/Examples/Monocular/mono_tum"
VOC="$ORB_DIR/Vocabulary/ORBvoc.txt"
YAML="$HERE/config/UCU_mono.yaml"
SEQ="$DATA_DIR/ucu_seq$N"
RAW="$DATA_DIR/ucu-slam-dataset-v1"
OUT_DIR="$HERE/results/seq$N"

die() { echo "[✗] $*"; exit 1; }
[[ -x "$PY" ]]  || die "Немає $PY — спершу запусти ./setup.sh"
[[ -x "$BIN" ]] || die "Немає $BIN — ORB-SLAM3 не зібраний (див. README, розділ 3)"
[[ -f "$VOC" ]] || die "Немає $VOC"
mkdir -p "$DATA_DIR" "$OUT_DIR"

# ---------- 1-2. дані ----------
if [[ -f "$SEQ/rgb.txt" ]]; then
  echo "[✓] Послідовність $N вже розпакована: $SEQ"
else
  MCAP="$RAW/sequence-$N/sequence-$N.mcap"
  if [[ ! -f "$MCAP" ]]; then
    echo "[*] Скачую sequence-$N (кілька ГБ)..."
    hf download ucu-autonomous-ugv/ucu-slam-dataset-v1 --repo-type dataset \
      --include "sequence-$N/*" --local-dir "$RAW" || die "Не вдалося скачати (hf auth login? доступ до датасету?)"
  fi
  echo "[*] Розпаковую $MCAP ..."
  "$PY" "$HERE/scripts/extract_ucu_mcap.py" "$MCAP" "$SEQ" || die "Розпаковка не вдалася"
  if [[ "${KEEP_MCAP:-0}" != "1" ]]; then
    rm -rf "$RAW/sequence-$N"
    echo "[*] .mcap видалено, щоб звільнити місце (KEEP_MCAP=1 — щоб залишати)"
  fi
fi

# ---------- 3. прогони ORB-SLAM3 ----------
cd "$ORB_DIR"
for i in $(seq 1 "$RUNS"); do
  echo "[*] Прогін $i/$RUNS ... (5–15 хв, на екрані тихо)"
  rm -f KeyFrameTrajectory.txt
  "$BIN" "$VOC" "$YAML" "$SEQ" > "$OUT_DIR/run$i.log" 2>&1
  if [[ -s KeyFrameTrajectory.txt ]]; then
    cp KeyFrameTrajectory.txt "$OUT_DIR/KF_run$i.txt"
    echo "    ключових кадрів: $(grep -vc '^#' "$OUT_DIR/KF_run$i.txt")"
  else
    echo "    [!] траєкторію не збережено (див. $OUT_DIR/run$i.log)"
  fi
done
cd - >/dev/null

# ---------- 4. RMSE + відео ----------
ls "$OUT_DIR"/KF_run*.txt >/dev/null 2>&1 || die "Жодного вдалого прогону"
echo "[*] Роблю відео..."
"$PY" "$HERE/scripts/make_slam_video.py" "$SEQ" "$OUT_DIR/seq${N}_all_runs.mp4" "$OUT_DIR"/KF_run*.txt \
  --step "$STEP" --fps "$FPS" --fit "$FIT" | tee "$OUT_DIR/rmse_summary.txt"

echo
echo "[✓] Готово. Результати: $OUT_DIR"
echo "    відео:  $OUT_DIR/seq${N}_all_runs.mp4"
echo "    RMSE:   $OUT_DIR/rmse_summary.txt"
echo "    відкрити у Windows:  explorer.exe \"\$(wslpath -w $OUT_DIR/seq${N}_all_runs.mp4)\""
