#!/bin/sh
set -eu

APP_DIR=/home/orangepi/isv2-controller
SERVICE_FILE=/etc/systemd/system/isv2-controller.service
ENV_FILE=/etc/default/isv2-controller

if [ "$(id -u)" -ne 0 ]; then
    echo "Run this installer with sudo." >&2
    exit 1
fi

for required in \
    "$APP_DIR/isv2_tool.py" \
    "$APP_DIR/robot_control.py" \
    "$APP_DIR/orange_pi_server.py" \
    "$APP_DIR/requirements.txt" \
    "$APP_DIR/deploy/isv2-controller.service" \
    "$APP_DIR/deploy/isv2-controller.env.example"
do
    if [ ! -f "$required" ]; then
        echo "Missing deployment file: $required" >&2
        exit 1
    fi
done

chown -R orangepi:orangepi "$APP_DIR"

if [ ! -x "$APP_DIR/.venv/bin/python" ]; then
    if ! sudo -u orangepi python3 -m venv "$APP_DIR/.venv"; then
        rm -rf "$APP_DIR/.venv"
        sudo -u orangepi python3 -m venv --without-pip "$APP_DIR/.venv"
    fi
fi

if "$APP_DIR/.venv/bin/python" -m pip --version >/dev/null 2>&1; then
    sudo -u orangepi "$APP_DIR/.venv/bin/python" -m pip install -r "$APP_DIR/requirements.txt"
else
    wheel=$(find "$APP_DIR/deploy/wheels" -maxdepth 1 -type f -name 'pyserial-*.whl' 2>/dev/null | head -n 1 || true)
    if [ -z "$wheel" ]; then
        echo "The venv has no pip and no offline pyserial wheel was found in deploy/wheels." >&2
        exit 1
    fi
    sudo -u orangepi "$APP_DIR/.venv/bin/python" - "$wheel" <<'PY'
import sys
import sysconfig
import zipfile

wheel_path = sys.argv[1]
site_packages = sysconfig.get_paths()["purelib"]
with zipfile.ZipFile(wheel_path) as archive:
    archive.extractall(site_packages)
print("Installed offline wheel:", wheel_path)
PY
fi

sudo -u orangepi "$APP_DIR/.venv/bin/python" -c 'import serial; print("pyserial", serial.__version__)'
install -m 0644 "$APP_DIR/deploy/isv2-controller.service" "$SERVICE_FILE"
if [ ! -f "$ENV_FILE" ]; then
    install -m 0644 "$APP_DIR/deploy/isv2-controller.env.example" "$ENV_FILE"
    echo "Created $ENV_FILE from the example configuration."
else
    echo "Preserved existing $ENV_FILE."
fi

systemctl daemon-reload
systemctl enable isv2-controller.service
systemctl restart isv2-controller.service
systemctl --no-pager --full status isv2-controller.service || true

echo "Installation complete. Health URL: http://192.168.2.5:8000/api/v1/health"
