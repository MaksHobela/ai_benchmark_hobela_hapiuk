# UCU SLAM Toolkit — ORB-SLAM3 на UCU SLAM Dataset

Набір скриптів, щоб прогнати ORB-SLAM3 (Monocular) на послідовностях UCU SLAM Dataset,
порахувати похибку відносно GNSS і зробити відео: кадр з камери + траєкторія на мапі.

Все запускається в **Ubuntu (WSL на Windows або звичайний Linux)**, команди вводяться в термінал Ubuntu
(рядок починається з `ім'я@комп'ютер:~$`, а не з `PS C:\>`).

---

## Що всередині

| Файл | Для чого |
|---|---|
| `setup.sh` | ставить Python-середовище, evo і клієнт Hugging Face (один раз) |
| `run_sequence.sh` | **усе одним рядком**: скачати → розпакувати → N прогонів ORB-SLAM3 → RMSE → відео |
| `scripts/extract_ucu_mcap.py` | розпаковує ROS 2 `.mcap` у формат TUM (`rgb/`, `rgb.txt`, `imu.txt`, `groundtruth.txt`) |
| `scripts/make_slam_video.py` | робить `.mp4`: кадр з камери + мапа (GNSS vs прогони ORB-SLAM3) + RMSE |
| `scripts/sanity_check.sh` | перевірка збірки ORB-SLAM3 на датасеті TUM (очікувано ~1 см) |
| `config/UCU_mono.yaml` | калібрування камери RealSense D435 для ORB-SLAM3 (з `calibration.yaml` датасету) |
| `results/` | сюди складаються результати; можуть уже лежати готові `KF_run*.txt` |

---

## 1. Підготовка (один раз)

```bash
unzip ucu_slam_toolkit.zip -d ~
cd ~/ucu_slam_toolkit
sed -i 's/\r$//' *.sh scripts/*.sh     # якщо архів відкривали у Windows
chmod +x *.sh scripts/*.sh
./setup.sh
```

Потім **перевідкрий термінал** і увійди в Hugging Face. Датасет закритий: треба на сторінці
https://huggingface.co/datasets/ucu-autonomous-ugv/ucu-slam-dataset-v1 прийняти умови доступу,
а на huggingface.co → Settings → Access Tokens створити токен типу **Read**.

```bash
hf auth login --token hf_ТВІЙ_ТОКЕН
hf auth whoami          # має показати твій логін
```

> Вставити в термінал WSL: правий клік або `Ctrl+Shift+V`.

---

## 2. Варіант А — тільки зробити відео з готових результатів (без ORB-SLAM3)

Підходить, якщо в `results/seqN/` уже лежать файли `KF_run*.txt` (траєкторії ORB-SLAM3).

```bash
N=1
mkdir -p ~/datasets && cd ~/datasets

# скачати й розпакувати послідовність (кілька ГБ)
hf download ucu-autonomous-ugv/ucu-slam-dataset-v1 --repo-type dataset \
  --include "sequence-$N/*" --local-dir ucu-slam-dataset-v1
~/venvs/ucu/bin/python ~/ucu_slam_toolkit/scripts/extract_ucu_mcap.py \
  ucu-slam-dataset-v1/sequence-$N/sequence-$N.mcap ~/datasets/ucu_seq$N

# зробити відео
~/venvs/ucu/bin/python ~/ucu_slam_toolkit/scripts/make_slam_video.py \
  ~/datasets/ucu_seq$N  ~/seq${N}_video.mp4  ~/ucu_slam_toolkit/results/seq$N/KF_run*.txt

# відкрити у Windows
explorer.exe "$(wslpath -w ~/seq${N}_video.mp4)"
```

### Параметри `make_slam_video.py`

```
make_slam_video.py  ПАПКА_ПОСЛІДОВНОСТІ  ВИХІД.mp4  ТРАЄКТОРІЯ [ТРАЄКТОРІЯ ...]  [опції]
```

| Параметр | За замовч. | Що робить |
|---|---|---|
| `ПАПКА_ПОСЛІДОВНОСТІ` | — | розпакована послідовність (`rgb.txt`, `rgb/`, `groundtruth.txt`) |
| `ВИХІД.mp4` | — | куди зберегти відео |
| `ТРАЄКТОРІЯ ...` | — | один або кілька файлів TUM (`KF_run1.txt KF_run2.txt …` або `KF_run*.txt`); кожен — своїм кольором |
| `--step N` | `3` | брати кожен N-й кадр: `1` — всі кадри (повільно, плавно), `6` — швидкий перегляд |
| `--fps N` | `15` | кадрів за секунду у відео (`--step 3 --fps 15` ≈ 1.7× швидше за реальний час) |
| `--fit gt` / `--fit all` | `gt` | `gt` — мапа за маршрутом GNSS (траєкторії, що відлетіли, виходять за край); `all` — вміщує все |
| `--gt ФАЙЛ` | `ПАПКА/groundtruth.txt` | інший файл ground truth |
| `--t-max-diff S` | `0.2` | макс. різниця часу (с) при зіставленні з GNSS |

На початку скрипт друкує для кожної траєкторії: кількість ключових кадрів, **ATE RMSE (xy)** і максимальну похибку
(так само, як `evo_ape ... -as --project_to_plane xy`).

Що на відео: ліворуч кадр з камери і час; праворуч мапа згори — **сіра** лінія GNSS, **кольорові** — прогони ORB-SLAM3,
над мапою для кожного прогону його RMSE і поточна похибка. `no keyframe Ns` — трекінг, ймовірно, загубився.

---

## 3. Варіант Б — повний цикл з ORB-SLAM3

### 3.1 Зібрати ORB-SLAM3 (один раз, 1–2 години)

Перевірено на Ubuntu 24.04 (WSL2), gcc 13. Що довелося виправляти — вже враховано нижче.

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake git pkg-config libeigen3-dev libopencv-dev \
  libglew-dev libboost-all-dev libssl-dev libepoxy-dev unzip wget

# Pangolin v0.8 + фікс для нового компілятора
cd ~ && git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && git checkout v0.8
./scripts/install_prerequisites.sh recommended
grep -rlE 'u?int(8|16|32|64)_t' components --include='*.cpp' --include='*.h' --include='*.hpp' \
  | xargs grep -L '#include <cstdint>' | xargs -r sed -i '1i #include <cstdint>'
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j2
sudo cmake --install build && sudo ldconfig

# ORB-SLAM3
cd ~ && git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git && cd ORB_SLAM3
sed -i 's/++11/++14/g' CMakeLists.txt                       # Pangolin потребує C++14
sed -i 's/-O3/-O1/g; s/-march=native//g' CMakeLists.txt      # менше пам'яті при збірці
sed -i 's/-march=native//g' Thirdparty/g2o/CMakeLists.txt Thirdparty/DBoW2/CMakeLists.txt  # інакше Segmentation fault
sed -i 's/make -j[0-9]*/make -j1/g' build.sh                  # 1 потік — щоб не закінчилась пам'ять
sed -i 's/MONOCULAR,true)/MONOCULAR,false)/' Examples/Monocular/mono_tum.cc  # без вікна (у WSL зависає)
sed -i 's/RGBD,true)/RGBD,false)/' Examples/RGB-D/rgbd_tum.cc                # те саме для TUM-тесту
chmod +x build.sh
nohup ./build.sh > build.log 2>&1 &
tail -f build.log          # Ctrl+C — вийти з перегляду, збірка продовжиться
```

Готово, коли існує файл `~/ORB_SLAM3/Examples/Monocular/mono_tum`.

**Якщо збірка падає з `Terminated` / `Killed`** — не вистачає пам'яті. У PowerShell (Windows):
```powershell
New-Item -ItemType Directory -Force -Path D:\wsl | Out-Null
@"
[wsl2]
swap=10GB
swapFile=D:\\wsl\\swap.vhdx
"@ | Set-Content -Path "$env:USERPROFILE\.wslconfig" -Encoding ascii
wsl --shutdown
```
і знову запусти `./build.sh`. Ніколи не запускай кілька збірок одночасно (`pgrep -a make` має бути порожнім перед стартом).

### 3.2 Перевірити збірку на TUM (рекомендовано)

```bash
~/ucu_slam_toolkit/scripts/sanity_check.sh        # fr1_xyz, очікувано RMSE ≈ 0.01 м → PASS
```
(цей тест використовує RGB-D приклад `rgbd_tum`, він збирається разом з усім ORB-SLAM3)

### 3.3 Прогнати послідовність одним рядком

```bash
cd ~/ucu_slam_toolkit
./run_sequence.sh 1          # послідовність 1, 5 прогонів  (≈ 1 година)
./run_sequence.sh 5 3        # послідовність 5, 3 прогони
STEP=6 FIT=all ./run_sequence.sh 8     # з налаштуваннями відео
```

Результат у `results/seqN/`: `KF_run*.txt`, `run*.log`, `rmse_summary.txt`, `seqN_all_runs.mp4`.
Готову розпаковку скрипт не скачує повторно; `.mcap` після розпаковки видаляє (залишити: `KEEP_MCAP=1`).

---

## 4. Послідовності

| N | Маршрут | Умови |
|---|---|---|
| 1 | Church loop, кампус | день |
| 2 / 3 | Середня петля, кампус (3 — зворотний напрямок) | день |
| 4 / 5 | «Вісімка», кампус | 4 — день, **5 — сутінки** |
| 6 / 7 | Середня петля, Стрийський парк (7 — зворотний) | день |
| 8 | Петля в парку | **сутінки** |
| 9 | Петля з перепадом висоти, парк | день |
| 10 | «Вісімка», парк | день |
| 11 / 12 | Лінійний кампус ↔ парк | день |
| 13 | Лінійний кампус → парк | **сутінки** |

Пари «день ↔ сутінки» на тому ж маршруті: **4 ↔ 5**, **6 ↔ 8**, **11 ↔ 13**.

---

## 5. Що важливо знати про результати

- **Глибина у v1 датасету зламана** → використовуємо Monocular (тільки RGB), не RGB-D.
- **Ground truth — GNSS без RTK**: тільки позиція, точність ~метри, висота найгірша. Тому дивимось RMSE у площині xy.
- **Monocular не знає масштабу** → вирівнювання Sim(3) (`-as`). Буває **дрейф масштабу**: прогони на тих самих даних
  дають дуже різний результат (напр. seq 1: 1.1 м і 12.7 м). Тому — кілька прогонів і медіана.
- Приклад (seq 1, день): вдалий прогін — ATE RMSE 1.57 м (3D) / 1.11 м (xy), петля не замкнулась (дрейф ~4 м).

---

## 6. Часті проблеми

| Симптом | Рішення |
|---|---|
| `command not found: Write-Host` тощо | команду PowerShell вставили в Ubuntu — і навпаки. PowerShell: `PS C:\>`, Ubuntu: `ім'я@...:~$` |
| `Illegal header value b'Bearer '` при `hf auth login` | токен не вставився; використай `hf auth login --token hf_...` |
| `Too few matched poses` у скрипті відео | файл траєкторії перезаписаний невдалим/обірваним прогоном; прожени ще раз |
| WSL не відкривається, диск C заповнений | перенести Ubuntu на D: `wsl --export Ubuntu D:\wsl\ubuntu.tar`, `wsl --unregister Ubuntu`, `wsl --import Ubuntu D:\wsl\Ubuntu D:\wsl\ubuntu.tar` (unregister — лише коли .tar точно створився) |
| Відео не відкривається стандартним плеєром | відкрий через VLC |
| `KF_run*.txt` майже порожній на сутінкових послідовностях | ORB-SLAM3 не ініціалізувався — це результат, а не помилка |
