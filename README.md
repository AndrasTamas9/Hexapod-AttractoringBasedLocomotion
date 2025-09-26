# Hexapod-AttractoringBasedLocomotion

This repository provides a minimal toolchain to (1) run a **neural torque controller** for a hexapod on an Arbotix-M, (2) start/stop an experiment from a Raspberry Pi while streaming and logging telemetry, and (3) post-process the logs into heatmaps and joint-tracking plots.

- **Firmware (Arduino / Arbotix-M):** `Hexapod_TorqueControl.ino` — neural controller + servo management
- **Raspberry Pi client:** `PiSerial.py` — interactive serial handshake, parameter upload, logging
- **Offline analysis:** `heatmap_with_joint_positions.py` — kinematics + heatmap/trace plots from `.txt` logs

---

## 1) What each component does

### A) `Hexapod_TorqueControl.ino` — Neural torque control firmware (Arbotix-M)

This Arduino sketch runs on the **Arbotix-M** controller and implements a **bio-inspired neural controller** that outputs **joint torques** (or torque-equivalent setpoints) for the servomotors.

Core features:
- **Neural control loop**
  - Activation **sigmoid** for neuron outputs.
  - Computes **neuronal activity** and **feedback** from joints.
  - Derives joint **torques `M`** from the model’s theoretical relations.
- **Servo management**
  - Sets **JOINT** / **WHEEL** modes.
  - Sets **maximum allowable torque** (EEPROM, one-time per servo).
  - Sets **Return Delay Time** (EEPROM).
  - Per-servo **speed/torque M** updates and a **synchronous** broadcast variant.
- **Sensing & telemetry**
  - Reports **current joint positions**, **temporarily saved positions**, **joint torques**, and **battery voltage** over serial.
  - Measures and reports **servo speeds**.
  - Corrects **measurement errors** for current position readings.
- **Safety**
  - Enforces **safe range checks** on motion.
  - Computes **coxa/femur range limits** (called once in `setup`).
  - Converts radian angles ↔ encoder ticks for commanded positions.
- **I/O protocol**
  - Maintains a lightweight serial protocol with the Raspberry Pi:
    - Waits for `connect` and responds with `CONNECTED!`.
    - Accepts a packed parameter frame (`a…b…c…d…z`) and replies `GET!`.
    - Streams telemetry lines until stopped.
    - On stop (`S\n` from Pi), **halts the robot**.

> Upload this sketch to the **Arbotix-M** using the Arduino IDE before running experiments.

---

### B) `PiSerial.py` — Raspberry Pi serial launcher & logger

A Python 3 script that orchestrates the experiment lifecycle:

- Prompts for:
  - `FILE name` → output log base name (creates `<FILE name>.txt`)
  - control parameters: `w_y`, `w_s`, `w_k`, `rand_par`
- Opens `/dev/ttyUSB0` @ 115200 baud and performs a **3-state handshake**:
  1. **Connect**: repeatedly sends `connect\n` until the controller responds `CONNECTED!`
  2. **Configure**: sends parameter frame  
     `a<w_y>b<w_s>c<w_k>d<rand_par>z\n` and waits for `GET!`
  3. **Run/Log**: reads lines continuously from serial, **prints** and **appends** to `<FILE name>.txt`
- On **Ctrl+C**, sends `S\n` (stop) to the controller, closes the log cleanly, and exits.

**Restart behavior:** If you re-run `PiSerial.py`, it re-establishes the handshake and the robot **starts a new run** with the newly entered parameters, logging to a fresh text file.

---

### C) `heatmap_with_joint_positions.py` — Offline analysis & visualization

Processes `.txt` logs produced by the Pi:

- Converts encoder ticks → radians.
- Simple forward kinematics for each leg (**coxa, femur, tibia**) to obtain end-point coordinates.
- Renders, for the last time window:
  - A **heatmap** of the **z** end-point for all 6 legs vs time (RF, RM, RB, LF, LM, LB).
  - Two time plots comparing **current vs target** angles for the first two **coxa** and **femur** joints.
- Batch mode: scans all `*.txt` files in the working directory and writes a PNG per input:

