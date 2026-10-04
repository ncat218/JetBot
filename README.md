# JetBot HRI delivery baseline

This source baseline implements the frozen Wave → approach Owner → load → Point LEFT/RIGHT → pulse scan → Owner raised-arm confirmation → deliver → stop protocol. All motor commands pass through `SafeMotorController`. Dry-run uses no JetBot packages. Robot mode requires a measured calibration and model files; it is **not validated on a physical JetBot yet**.

## Repository map

```text
jetbot_hri/
├── config/{default,calibration,software_versions}.yaml
├── jetbot_hri/{types,configuration,platform,perception,interaction,control,main}.py
├── scripts/{inspect_platform,scan_i2c}.sh
├── scripts/{setup_wifi.md,test_wifi,test_battery,test_camera,test_motor,test_pose,test_collision,run_full_pipeline}.py
├── tests/test_pipeline.py
├── requirements.txt
└── requirements-hardware.txt
```

`default.yaml` contains **simulation-only** values. Robot mode rejects missing calibration and any `TBD`. The IMX219 160° specification is diagonal FOV; usable horizontal view must be measured. Bbox height is a relative stop cue, not metric distance. Pulse turns do not assume encoder/IMU yaw.

## Step 0 — install and dry-run (laptop or JetBot)

From this directory:

```bash
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install -e .
python3 -m jetbot_hri.main --mode dry-run
python3 -m unittest discover -s tests -v
```

Expected: transitions through `COMPLETE`; 16 unit tests pass; `logs/run.jsonl` contains one JSON object per tick. If Python or PyYAML is missing, inspect the active interpreter and reinstall requirements. The source uses Python 3.6 syntax; the package installs the `dataclasses` backport on Python 3.6. JetPack 4.x system Python and NVIDIA wheels may not work in a new venv; use its compatible environment after the platform inspection.

## Step 1 — inspect stack (JetBot)

```bash
bash scripts/inspect_platform.sh | tee platform.txt
```

Expected: L4T/JetPack, Python, JetBot, TensorRT, PyTorch, power mode, IP, and I2C results. Missing commands are printed. Return `platform.txt` to the team before choosing NVIDIA wheels, `trt_pose` weights, or a camera pipeline. Do not assume a desktop-built TensorRT engine will run on the Nano.

## Step 2 — WiFi and IP (JetBot)

Follow [setup_wifi.md](scripts/setup_wifi.md): use `nmcli` interactively and verify reconnect after reboot.

```bash
python3 -m scripts.test_wifi
hostname -I
```

Expected: connected status, SSID/connection name, and WiFi IPv4 address. If `nmcli` is unavailable, the reader returns an error without crashing local safety. `wifi.stop_on_wifi_loss` defaults to false. If the team relies on WiFi emergency control, change the policy only after validating that control path.

## Step 3 — I2C and INA219 (JetBot)

```bash
sudo apt-get install i2c-tools
bash scripts/scan_i2c.sh
python3 -m pip install -r requirements-hardware.txt
python3 -m scripts.test_battery --mock
python3 -m scripts.test_battery --config config/calibration.yaml
```

Expected: `i2cdetect` identifies the actual INA219 address (often `0x40` or `0x41`); put it in `battery.i2c_address` before the live test. The live test prints pack voltage near a plausible 3S range. If it errors, check wiring, address, I2C permissions, and library compatibility. `pi-ina219` is tried first, then `smbus2` raw registers. The raw fallback reads bus voltage register `0x02` and shunt register `0x01`; current and power depend on the calibrated shunt resistance. Percentage is not used for safety.

Battery policy uses a five-sample median and a sustained low interval. Below 10.2 V warns; filtered voltage below 10.0 V for three seconds latches critical. Sensor errors latch as well. Motion stays disabled until a manual reset after charging/checking the pack. Never intentionally discharge a real pack to test this: `python3 -m unittest discover -s tests -v` covers simulated sag, sustained low voltage, and latch behavior.

## Step 4 — camera (JetBot)

```bash
python3 -m scripts.test_camera
```

Expected: one frame shape. If capture fails, validate the CSI ribbon, camera support and `camera.source` for the actual GStreamer or V4L2 pipeline. Camera failure stops the robot loop. Frame age over `safety.max_frame_age_sec` closes the motion gate.

## Step 5 — motor STOP and low-speed pulse (JetBot)

**Lift the wheels off the ground or secure the robot in a clear area before the first motor test. Keep a physical power cut within reach.** Calibrate the battery address and maximum speed first.

```bash
python3 -m scripts.test_motor --config config/calibration.yaml --wheels-lifted
```

Expected: a 0.2-second slow pulse followed by STOP. If a wheel direction is wrong, correct the motor wiring/backend before autonomous tests. The TTL stops output if a command is not refreshed within `motor_command_ttl_sec`. Python's TTL thread cannot guarantee STOP after a hard kernel/process hang; use a physical power cut and consider an external motor enable watchdog for deployment.

## Step 6 — pose and collision (JetBot)

Calibrate `pose.model_path`, `pose.human_pose_json`, `pose.input_width/height`, `collision.model_path`, `collision.roi`, and threshold. The pose adapter expects a `torch2trt.TRTModule` state dictionary with `cmap,paf` outputs and uses the **local** `human_pose.json` keypoint order. The collision adapter expects a TorchScript binary classifier returning one blocked logit for a 224×224 RGB ROI. These tensor contracts need validation on the installed JetPack and chosen weights. Missing models fail explicitly; there is no fake live inference.

```bash
python3 -m scripts.test_pose
python3 -m scripts.test_collision
python3 -m jetbot_hri.main --mode perception --config config/calibration.yaml
```

Expected: adapters load; perception-only shows the camera overlay and never commands real motors. Press `e` for a latched local emergency STOP or `q` to exit. For a headless session use `--no-display` and inspect `logs/run.jsonl`. If a model fails to load or output poses look wrong, validate the exact model architecture, preprocessing, topology, keypoint naming and installed CUDA stack before robot mode.

Collision data must cover: empty path FREE; person at normal interaction distance FREE; person too close BLOCKED; box/chair/object in corridor BLOCKED; person behind an obstacle BLOCKED. The pretrained JetBot classifier is a starting point, not a deployment certificate. `AVOID` can turn in place when the forward corridor is blocked; a blocked forward command remains stopped. `PULSE_SCAN` blocked enters `SCAN_BLOCKED` and does not invoke avoidance.

## Step 7 — calibrate and robot mode (JetBot)

Replace **every `TBD`** in `config/calibration.yaml` using staged measurements. Record software versions in `software_versions.yaml`. Then, with clear space and physical power cut:

```bash
python3 -m jetbot_hri.main --mode robot --config config/calibration.yaml
```

Expected: no startup if calibration, models, camera, battery, or motor backend is unavailable. Live runs append telemetry to `logs/run.jsonl`. Watch the overlay/log for state, transition, FPS, frame age, battery, WiFi, Owner/candidate/Receiver boxes, gesture, collision, desired motion, actual command and abort reason. Press `e` to latch emergency STOP. `ABORT` requires restarting/manual FSM reset; there is no automatic restart. Always inspect the environment before resetting.

The loopback dashboard listens on `127.0.0.1:8765` on the JetBot. On a laptop, open an SSH tunnel with `ssh -L 8765:127.0.0.1:8765 <user>@<jetbot-ip>`, then visit `http://127.0.0.1:8765/`. Its STOP button posts to `/estop` and latches the local supervisor. `--dashboard-port 0` disables it. Verify the remote STOP before driving; a lost WiFi link means this channel cannot reach the robot. Keep the physical power cut available.

First physical session order: inspect stack → WiFi → I2C/battery → STOP and raised-wheel motor pulse → camera → collision with a person and obstacle → pose/gesture confusion tests → slow Owner approach → pulse/FOV → candidate decision → delivery → avoidance → full run. Do not start with a person-to-person full run.

## Calibration still required

`config/calibration.yaml` marks all unknowns `TBD`: INA219 address; motor speeds; gesture wave, point and confirm thresholds/windows; static wrist tolerance; scan pulse and settle timing; usable FOV and candidate zone; relative bbox stop ratios; servo gains; collision ROI, threshold and model; pose weights/topology/input dimensions; decision, avoidance direction/timings and recovery timings. Binary collision does not reveal obstacle side, so `avoid.turn_side` must be chosen and tested for the actual demo route. Inspect the Nano before choosing power mode, tracker tuning, and detector strategy. If distant people lack pose recall, add a compatible detector backend rather than treating old Object Following TensorRT weights as portable.

The overlay is the camera UI; the tunnel dashboard provides status and STOP. It deliberately binds only to loopback and has no reset endpoint.
