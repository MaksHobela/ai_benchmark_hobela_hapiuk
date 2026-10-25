#!/usr/bin/env bash
# =============================================================================
#  ORB-SLAM3 RGB-D sanity check на датасеті TUM
#
#  Що робить:
#    1. Перевіряє, що ORB-SLAM3 зібраний і evo встановлений
#    2. Завантажує послідовність TUM (якщо її ще немає)
#    3. Готує associations-файл (RGB <-> depth)
#    4. Запускає rgbd_tum
#    5. Рахує ATE (evo_ape, SE(3) alignment) проти ground truth
#    6. Перевіряє покриття траєкторії та порівнює RMSE з порогом
#    7. Зберігає звіт і всі артефакти в results/<послідовність>_<дата>/
#
#  Використання:
#    ./sanity_check.sh                 # fr1_xyz, поріг 0.03 м
#    ./sanity_check.sh fr1_desk        # інша послідовність
#    ./sanity_check.sh fr2_xyz 0.02    # своя послідовність і поріг
#
#  Змінні оточення (необов'язково):
#    ORB_DIR      шлях до ORB_SLAM3        (за замовч. ~/ORB_SLAM3)
#    DATA_DIR     папка з датасетами       (за замовч. ~/datasets)
#    MIN_COVERAGE мін. частка кадрів з позою (за замовч. 0.90)
#
#  Код виходу: 0 = PASS, 1 = FAIL, 2 = помилка середовища
# =============================================================================
set -uo pipefail

SEQ="${1:-fr1_xyz}"
THRESHOLD="${2:-0.03}"
ORB_DIR="${ORB_DIR:-$HOME/ORB_SLAM3}"
DATA_DIR="${DATA_DIR:-$HOME/datasets}"
MIN_COVERAGE="${MIN_COVERAGE:-0.90}"

# ---------- допоміжне ----------
log()  { echo -e "\033[1;34m[*]\033[0m $*"; }
ok()   { echo -e "\033[1;32m[✓]\033[0m $*"; }
fail() { echo -e "\033[1;31m[✗]\033[0m $*"; }
die()  { fail "$*"; exit 2; }

# ---------- 0. розбір назви послідовності ----------
# fr1_xyz -> freiburg1, xyz ; fr3_long_office_household -> freiburg3, long_office_household
if [[ ! "$SEQ" =~ ^fr([123])_(.+)$ ]]; then
  die "Назва послідовності має бути у форматі frN_name, напр. fr1_xyz"
fi
FR_NUM="${BASH_REMATCH[1]}"
SEQ_NAME="${BASH_REMATCH[2]}"
SEQ_DIR_NAME="rgbd_dataset_freiburg${FR_NUM}_${SEQ_NAME}"
SEQ_PATH="$DATA_DIR/$SEQ_DIR_NAME"
YAML="$ORB_DIR/Examples/RGB-D/TUM${FR_NUM}.yaml"
URL="https://cvg.cit.tum.de/rgbd/dataset/freiburg${FR_NUM}/${SEQ_DIR_NAME}.tgz"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$(pwd)/results/${SEQ}_${STAMP}"
mkdir -p "$OUT_DIR"
REPORT="$OUT_DIR/report.txt"

echo "=================================================="
echo " ORB-SLAM3 RGB-D sanity check"
echo " Послідовність : $SEQ"
echo " Поріг RMSE    : $THRESHOLD м"
echo " Результати    : $OUT_DIR"
echo "=================================================="

# ---------- 1. перевірка середовища ----------
log "Перевірка середовища..."
BIN="$ORB_DIR/Examples/RGB-D/rgbd_tum"
VOC="$ORB_DIR/Vocabulary/ORBvoc.txt"
[[ -x "$BIN" ]]  || die "Не знайдено $BIN — ORB-SLAM3 не зібраний"
[[ -f "$VOC" ]]  || die "Не знайдено $VOC — розпакуй Vocabulary/ORBvoc.txt.tar.gz"
[[ -f "$YAML" ]] || die "Не знайдено $YAML"
command -v evo_ape >/dev/null || die "evo не встановлено: sudo apt install -y pipx && pipx install evo"
command -v python3 >/dev/null || die "python3 не знайдено"
ok "Середовище в порядку"

# ---------- 2. датасет ----------
if [[ -f "$SEQ_PATH/groundtruth.txt" ]]; then
  ok "Датасет вже є: $SEQ_PATH"
else
  log "Завантаження $URL ..."
  mkdir -p "$DATA_DIR"
  wget -q --show-progress -O "$DATA_DIR/${SEQ_DIR_NAME}.tgz" "$URL" || die "Не вдалося завантажити датасет"
  tar -xzf "$DATA_DIR/${SEQ_DIR_NAME}.tgz" -C "$DATA_DIR" || die "Не вдалося розпакувати"
  rm -f "$DATA_DIR/${SEQ_DIR_NAME}.tgz"
  ok "Датасет завантажено"
fi

# ---------- 3. associations ----------
ASSOC="$ORB_DIR/Examples/RGB-D/associations/${SEQ}.txt"
if [[ -f "$ASSOC" ]]; then
  ok "Використовую готовий associations: $ASSOC"
else
  ASSOC="$OUT_DIR/associations.txt"
  log "Генерую associations (макс. різниця 0.02 с)..."
  python3 - "$SEQ_PATH" "$ASSOC" <<'PY'
import sys, os
seq, out = sys.argv[1], sys.argv[2]
def read(f):
    d = {}
    for line in open(os.path.join(seq, f)):
        if line.startswith('#') or not line.strip(): continue
        t, p = line.split()[:2]; d[float(t)] = p
    return d
rgb, dep = read('rgb.txt'), read('depth.txt')
dk = sorted(dep)
import bisect
pairs, used = [], set()
for t in sorted(rgb):
    i = bisect.bisect_left(dk, t)
    best = min((dk[j] for j in (i-1, i) if 0 <= j < len(dk)), key=lambda x: abs(x-t), default=None)
    if best is not None and abs(best-t) < 0.02 and best not in used:
        used.add(best); pairs.append((t, rgb[t], best, dep[best]))
with open(out, 'w') as f:
    for a, b, c, d in pairs: f.write(f"{a:.6f} {b} {c:.6f} {d}\n")
print(len(pairs))
PY
  [[ -s "$ASSOC" ]] || die "Не вдалося згенерувати associations"
  ok "Associations згенеровано"
fi
N_FRAMES=$(wc -l < "$ASSOC")

# ---------- 4. запуск ORB-SLAM3 ----------
log "Запуск ORB-SLAM3 ($N_FRAMES кадрів)..."
cd "$OUT_DIR"
START=$(date +%s)
"$BIN" "$VOC" "$YAML" "$SEQ_PATH" "$ASSOC" > orbslam.log 2>&1
RC=$?
END=$(date +%s)
cd - >/dev/null

if [[ $RC -ne 0 ]]; then
  fail "ORB-SLAM3 завершився з кодом $RC (див. $OUT_DIR/orbslam.log)"
  tail -n 15 "$OUT_DIR/orbslam.log"
  exit 1
fi
[[ -s "$OUT_DIR/CameraTrajectory.txt" ]] || { fail "CameraTrajectory.txt не створено"; exit 1; }
ok "ORB-SLAM3 відпрацював за $((END-START)) с"

MEAN_TT=$(grep -oP 'mean tracking time: \K[0-9.e-]+' "$OUT_DIR/orbslam.log" || echo "n/a")
N_POSES=$(grep -cv '^#' "$OUT_DIR/CameraTrajectory.txt")
N_MAPS=$(grep -c 'Creation of new map' "$OUT_DIR/orbslam.log" || true)
COVERAGE=$(python3 -c "print(f'{$N_POSES/$N_FRAMES:.3f}')")

# ---------- 5. ATE ----------
log "Обчислення ATE (evo_ape, SE(3) alignment)..."
evo_ape tum "$SEQ_PATH/groundtruth.txt" "$OUT_DIR/CameraTrajectory.txt" -a \
  --save_results "$OUT_DIR/ate.zip" --no_warnings > "$OUT_DIR/ate.txt" 2>&1 \
  || { fail "evo_ape впав (див. $OUT_DIR/ate.txt)"; exit 1; }

get() { awk -v k="$1" '$1==k {print $2}' "$OUT_DIR/ate.txt"; }
RMSE=$(get rmse); MEAN=$(get mean); MEDIAN=$(get median); MAX=$(get max)

# ---------- 6. вердикт ----------
PASS_RMSE=$(python3 -c "print(int($RMSE <= $THRESHOLD))")
PASS_COV=$(python3 -c "print(int($COVERAGE >= $MIN_COVERAGE))")

{
  echo "ORB-SLAM3 RGB-D sanity check report"
  echo "Дата            : $(date '+%Y-%m-%d %H:%M:%S')"
  echo "Система         : $(lsb_release -ds 2>/dev/null || uname -a)"
  echo "ORB-SLAM3 commit: $(git -C "$ORB_DIR" rev-parse --short HEAD 2>/dev/null || echo n/a)"
  echo "Послідовність   : $SEQ_DIR_NAME"
  echo "Калібрування    : $(basename "$YAML")"
  echo "--------------------------------------------------"
  echo "Кадрів           : $N_FRAMES"
  echo "Поз у траєкторії : $N_POSES  (покриття $COVERAGE, мін $MIN_COVERAGE)"
  echo "Створено карт    : $N_MAPS  (1 = без втрат трекінгу)"
  echo "Сер. час трекінгу: $MEAN_TT с/кадр"
  echo "--------------------------------------------------"
  echo "ATE RMSE   : $RMSE м   (поріг $THRESHOLD м)"
  echo "ATE mean   : $MEAN м"
  echo "ATE median : $MEDIAN м"
  echo "ATE max    : $MAX м"
  echo "--------------------------------------------------"
  if [[ $PASS_RMSE -eq 1 && $PASS_COV -eq 1 ]]; then
    echo "ВЕРДИКТ: PASS"
  else
    echo "ВЕРДИКТ: FAIL"
    [[ $PASS_RMSE -eq 0 ]] && echo "  - RMSE перевищує поріг"
    [[ $PASS_COV  -eq 0 ]] && echo "  - замало кадрів з позою (трекінг губився)"
  fi
} | tee "$REPORT"

echo
echo "Артефакти: $OUT_DIR"
echo "  report.txt, orbslam.log, CameraTrajectory.txt, KeyFrameTrajectory.txt, ate.txt, ate.zip"
echo "Графік:  evo_traj tum $OUT_DIR/CameraTrajectory.txt --ref $SEQ_PATH/groundtruth.txt -a -p"

[[ $PASS_RMSE -eq 1 && $PASS_COV -eq 1 ]] && exit 0 || exit 1
