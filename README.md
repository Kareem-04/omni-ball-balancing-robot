# 🤖 Omni-Directional Mobile Platform with Ball Balancing System

A **three-wheeled holonomic robot** with an integrated **ball balancing system** mounted on top. The mobile base moves in any direction using omni-wheels and inverse kinematics, while the upper platform uses a **resistive touch panel** and two **RC servo motors** to keep a ball stabilized at a target position — all running in real time.

> 📚 Course: MTE 434 — Sensors and Actuators  
> 🏫 Egypt Japan University of Science and Technology (E-JUST) · Fall 2025

**🎬 Demo Video:** [Watch on LinkedIn](https://www.linkedin.com/posts/mohammed-abdelsabour-1b519321a_omni-directional-mobile-platform-with-ball-ugcPost-7411772018309742592-Spz3)

---

## 📋 Overview

This project integrates two distinct control challenges into a single robotic platform. The **mobile base** provides full holonomic movement — it can translate in any direction and rotate simultaneously — teleoperated over ROS2 from a keyboard. On top of it, the **ball balancing system** uses a 4-wire resistive touch panel to detect ball position in real time and tilts the plate via servo motors to continuously correct the ball's trajectory using a PID controller.

The control stack is layered: ROS2 handles high-level velocity commands and odometry on a Raspberry Pi, which communicates over serial to an Arduino responsible for real-time PWM generation and the 50 Hz PID loop.

---

## ✨ Features

- 🔄 **Holonomic movement** — full 2D translation + rotation with no turning radius constraint
- 📡 **ROS2 teleoperation** — keyboard-controlled via `teleop_twist_keyboard`, publishes `/cmd_vel`
- 🧮 **Inverse kinematics** — Raspberry Pi converts body velocity to per-wheel angular speeds
- 📍 **Odometry & TF** — pose integration published to `/odom`, broadcast to `base_link`
- ⚖️ **Real-time PID control** — decoupled X/Y PID at 50 Hz for ball stabilization
- 🖐️ **Resistive touch panel** — low-latency 2D ball position sensing, immune to lighting conditions
- 📉 **Moving average filter** — 10-sample window smooths ADC noise before PID computation
- 🛡️ **Anti-windup & safety** — integral clamping, auto-return to flat on contact loss
- 🎯 **Trajectory modes** — point hold, circle, ellipse, and four-corners path following

---

## 🏗️ System Architecture

```
  [Keyboard]
      │
      ▼
teleop_twist_keyboard
      │  /cmd_vel (Twist)
      ▼
┌─────────────────────────────────────┐
│         Raspberry Pi (ROS2)         │
│  OmniController Node                │
│  ├─ Inverse Kinematics → ω1,ω2,ω3  │
│  ├─ Odometry estimation → /odom    │
│  └─ TF broadcast (odom→base_link)  │
└──────────────┬──────────────────────┘
               │ Serial @ 115200 baud
               │ 3 bytes per cycle (one per wheel)
               ▼
┌─────────────────────────────────────┐
│              Arduino                │
│  ├─ Wheels: setMotor() via L298N   │
│  └─ Plate:  PID → servo angles     │
└───────┬─────────────────────┬───────┘
        │                     │
        ▼                     ▼
  [L298N Driver]        [Touch Panel]
  3× DC Motors          → filteredX, filteredY
  (Omni Base)           → PID → xServo / yServo
                        (Ball Balancing Plate)
```

---

## 🧰 Hardware Components

| Component | Model / Type | Purpose |
|-----------|-------------|---------|
| Microcontroller (high-level) | Raspberry Pi | ROS2 host, kinematics, odometry |
| Microcontroller (low-level) | Arduino Mega | Real-time PWM, PID loop |
| Wheels | Omni-wheels × 3 | Holonomic motion |
| DC Motors | 3× geared DC motors | Drive omni-wheels |
| Motor Driver | L298N H-Bridge | Bidirectional speed control via PWM |
| Position Sensor | 4-wire Resistive Touch Panel | Ball X-Y coordinate detection |
| Actuators (plate) | RC Servo Motors × 2 | Plate pitch (X-axis) and roll (Y-axis) |
| Chassis | 3D-printed + aluminium extrusion | Structural frame for both subsystems |

---

## 📐 Pin Configuration

### Arduino — Wheels Controller

| Arduino Pin | Connected To | Function |
|-------------|-------------|---------|
| D3, D6, D9 | L298N ENA/ENB | PWM speed control (motors 1, 2, 3) |
| D4, D7, D10 | L298N IN1/IN3 | Motor direction pin A |
| D5, D8, D11 | L298N IN2/IN4 | Motor direction pin B |

### Arduino — Ball Balancing

| Arduino Pin | Connected To | Function |
|-------------|-------------|---------|
| A2 (YP) | Touch Panel | Y+ analog read |
| A3 (XM) | Touch Panel | X− analog read |
| D8 (YM) | Touch Panel | Y− digital |
| D9 (XP) | Touch Panel | X+ digital |
| D10 | X-axis Servo | Plate pitch control |
| D11 | Y-axis Servo | Plate roll control |

---

## 🧮 Kinematics

### Inverse Kinematics (Raspberry Pi → wheel speeds)

The three wheels are spaced 120° apart. Given a desired body velocity `[vx, vy, ωz]`, the required wheel angular velocities are:

```
ω₁ (Back)        = ( vy + L·ωz ) / r
ω₂ (Front-Right) = ( (√3/2)·vx − 0.5·vy + L·ωz ) / r
ω₃ (Front-Left)  = ( −(√3/2)·vx − 0.5·vy + L·ωz ) / r
```

Where `r = 0.034 m` (wheel radius) and `L = 0.09 m` (center-to-wheel distance).

### Forward Kinematics (odometry estimation)

```
vx = (r/3)   · ( 2·ω₃ − ω₁ − ω₂ )
vy = (r/√3)  · ( ω₁ − ω₂ )
ωz = (r/3·L) · ( ω₁ + ω₂ + ω₃ )
```

Pose is integrated at 50 Hz and published to `/odom`.

---

## ⚖️ Ball Balancing — PID Controller

The Arduino runs a decoupled PID controller for X and Y axes at **50 Hz** (20 ms period).

### PID Gains

| Axis | Kp  | Ki   | Kd  |
|------|-----|------|-----|
| X    | 0.5 | 0.04 | 0.3 |
| Y    | 0.4 | 0.05 | 0.2 |

### Signal Pipeline

```
Touch Panel (ADC 0–1024)
    → map() to physical mm coords
    → Moving Average Filter (N = 10 samples)
    → error = setpoint − filtered_position
    → PID: P + I (clamped) + D
    → map() to servo offset (±50° X / ±40° Y)
    → xServo.write(80 + offset)
       yServo.write(90 + offset)
```

### Trajectory Modes

| Mode | Setpoint Behaviour |
|------|-------------------|
| `0` | Hold center `(0, 0)` — default |
| `1` | Circular path, radius = 10 mm |
| `2` | Four-corners patrol (switches every 2 s) |
| `3` | Ellipse path, a = 15 mm, b = 10 mm |

---

## 📡 Serial Communication Protocol

Raspberry Pi sends **3 bytes per cycle** over `/dev/ttyUSB0` at 115200 baud — one byte per wheel:

```python
# Raspberry Pi (Python)
byte = int((ω / 10.0) * 127 + 128)   # maps −10..+10 rad/s → 0..255
ser.write(bytes([w1, w2, w3]))
```

```cpp
// Arduino (C++)
speed = Serial.read() - 128;          // recovers signed −127..+127
```

128 is the neutral (zero speed) value, allowing a single unsigned byte to carry signed speed information.

---

## 💻 Software & Libraries

| Library | Platform | Purpose |
|---------|----------|---------|
| `rclpy` | Raspberry Pi (Python) | ROS2 Python client |
| `geometry_msgs`, `nav_msgs` | Raspberry Pi | Twist, Odometry message types |
| `tf2_ros` | Raspberry Pi | TF transform broadcasting |
| `pyserial` | Raspberry Pi | Serial communication to Arduino |
| `TouchScreen.h` | Arduino (C++) | Resistive touch panel ADC abstraction |
| `Servo.h` | Arduino (C++) | RC servo PWM signal generation |

---

## 📁 Repository Structure

```
omni-ball-balancing-robot/
├── README.md
├── arduino/
│   ├── sensitive_plate/
│   │   └── sensitive_plate.ino       ← Ball balancing PID + trajectory control
│   └── wheels_controlling/
│       └── wheels_controlling.ino    ← L298N motor driver + serial interface
├── ros2/
│   └── omni_controller/
│       └── kinematics_node.py        ← ROS2 node: kinematics, odometry, TF
└── docs/
    ├── Report.pdf                    ← Full scientific report
    └── Presentation.pdf              ← Course slides
```

---

## ⚠️ Notes

- The serial port in `kinematics_node.py` is hardcoded to `/dev/ttyUSB0` — update this to match your Raspberry Pi's actual Arduino port.
- Touch panel ADC mappings (`map(p.x, 0, 1024, -82, 82)`) are calibrated for the specific panel used — adjust these values for a different-sized panel.
- PID gains were tuned manually for this hardware. Retuning will likely be needed if servo placement or plate dimensions change.

---

## 🔮 Future Improvements

- Add **IMU feed-forward** to compensate for inertial disturbances during base movement
- Replace open-loop odometry with **wheel encoder feedback** for accurate localization
- Implement **LQR controller** for optimal ball stabilization
- Add **ROS2 parameter server** support for runtime PID gain tuning
- Integrate a **SLAM module** on the Raspberry Pi for autonomous navigation

---

## 👥 Team

**Kareem Shaban Eid** — Mechatronics Engineering Student, E-JUST  
[LinkedIn](https://linkedin.com/in/kareem-04-soliman) · [GitHub](https://github.com/Kareem-04)

**Mohamed Abdelsabour**  — Mechatronics Engineering Student, E-JUST 

[LinkedIn](https://www.linkedin.com/in/mohammed-abdelsabour-1b519321a/) 

**Mohamed Gamal Zain**  — Mechatronics Engineering Student, E-JUST

**Mohammed Hamada**  — Mechatronics Engineering Student, E-JUST

*Supervised by Prof. Mohamed Adel & Eng. Gana Elemam*
