#!/usr/bin/env bash
# Створює Python-середовище для скриптів (розпаковка .mcap, відео) і ставить evo.
# Запуск: ./setup.sh
set -e
VENV="${VENV:-$HOME/venvs/ucu}"

echo "[*] Системні пакети (потрібен пароль sudo)..."
sudo apt update
sudo apt install -y python3-venv python3-pip pipx

echo "[*] Python-середовище: $VENV"
python3 -m venv "$VENV"
"$VENV/bin/pip" install --upgrade pip
"$VENV/bin/pip" install mcap mcap-ros2-support numpy opencv-python-headless

echo "[*] evo (оцінка траєкторій) і клієнт Hugging Face..."
pipx install evo || true
pipx install "huggingface_hub[cli]" || true
pipx ensurepath
evo_config set plot_backend Agg 2>/dev/null || "$HOME/.local/bin/evo_config" set plot_backend Agg || true

echo
echo "[✓] Готово. Перевідкрий термінал, потім увійди в Hugging Face:"
echo "    hf auth login --token hf_ТВІЙ_ТОКЕН"
