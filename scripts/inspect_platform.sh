#!/usr/bin/env bash
set -u
echo 'L4T:'; cat /etc/nv_tegra_release 2>&1 || true
echo 'L4T core:'; dpkg-query --show nvidia-l4t-core 2>&1 || true
echo 'JetPack:'; apt-cache show nvidia-jetpack 2>/dev/null | grep -m 1 Version || true
echo 'Python:'; python3 --version
echo 'JetBot:'; python3 -m pip show jetbot 2>&1 || true
echo 'TensorRT:'; python3 -c 'import tensorrt; print(tensorrt.__version__)' 2>&1 || true
echo 'PyTorch:'; python3 -c 'import torch; print(torch.__version__)' 2>&1 || true
echo 'trt_pose:'; python3 -c 'import trt_pose; print(trt_pose.__file__)' 2>&1 || true
echo 'Power mode:'; sudo nvpmodel -q 2>&1 || true
echo 'WiFi:'; nmcli device status 2>&1 || true
echo 'IP:'; hostname -I 2>&1 || true
echo 'I2C bus 1:'; i2cdetect -y -r 1 2>&1 || true

