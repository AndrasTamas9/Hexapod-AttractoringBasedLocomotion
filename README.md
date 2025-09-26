# Hexapod-AttractoringBasedLocomotion

This repository contains two main components for controlling and analyzing a hexapod robot:

---

## 1. `Hexapod_TorqueControl.ino`

An **Arduino sketch** (for the Arbotix-M controller) that manages **servo torque control and motion execution** for the hexapod robot.

- Sets joint modes (`JOINT` / `WHEEL`).
- Handles target positions and torque limits for each servo.
- Communicates with a Raspberry Pi via serial connection, receiving commands and sending feedback.
- Implements safety checks to ensure the servos operate within allowed ranges.
- Provides functions for calibration, encoder conversion, and activation of movements.
- Supports **torque-based control** of the joints for more stable walking and testing of bio-inspired control strategies.

---

## 2. `heatmap_with_joint_positions.py`

A **Python script** for analyzing joint movement data and visualizing it.

- Loads recorded `.txt` data files containing joint encoder values and target positions.
- Converts encoder values to joint angles (in radians).
- Computes kinematics for each leg (coxa, femur, tibia).
- Generates:
  - A **heatmap** of the vertical (z-axis) end-point positions of all six legs over time.
  - Time plots comparing **measured vs. target joint angles** for the coxa and femur joints.
- Automatically processes all `.txt` files in the working directory and saves figures as `.png`.

---

