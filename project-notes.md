# 🚗 Mini Self-Driving Car Project

A compact, camera-based self-driving car project using Raspberry Pi 4 and Arduino Nano, with motor control, obstacle detection using YOLOv5, encoder feedback, and MPU6050-based turning.

---

## 🧰 Hardware Components

- **Raspberry Pi 4** (for vision + AI tasks)
- **Pi Camera 2** (used with YOLOv5)
- **Arduino Nano** (real-time control)
- **L298N Motor Driver**
- **4× DC Gear Motors** with encoder disks
- **2× IR Speed Measurement Sensors** (Groove-type)
- **MPU6050** IMU (gyro + accelerometer)
- **Power System**:
  - 4× 18650 batteries for motors + Nano
  - USB power bank for Raspberry Pi

---

## 🧠 System Architecture

| Task                        | Handled By     | Notes                                |
|-----------------------------|----------------|--------------------------------------|
| Obstacle detection (YOLOv5) | Raspberry Pi   | Uses Pi Camera and object detection  |
| Navigation logic            | Raspberry Pi   | Decides when/how much to turn        |
| Motor control (PWM)         | Arduino Nano   | Via L298N, real-time response        |
| Encoder tick counting       | Arduino Nano   | Using interrupts                     |
| Precise turning             | Arduino Nano   | Via MPU6050 `gyroZ` data             |
| Communication               | USB Serial     | Between Pi and Nano                  |

---

## 🔄 Communication Protocol

Raspberry Pi communicates with Nano via USB serial (`/dev/ttyUSB0` or `/dev/ttyACM0`).

### Serial Commands Format (Sent from Pi to Nano)

| Command     | Action                         |
|-------------|--------------------------------|
| `F`         | Move forward                   |
| `B`         | Move backward                  |
| `S`         | Stop movement                  |
| `T30`       | Turn right 30 degrees          |
| `T-45`      | Turn left 45 degrees           |
| `G`         | Request RPM data               |

### Response from Nano

Nano responds with messages like:
L:125.6,R:129.3

yaml
Copy
Edit

---

## ⚙️ Encoders (IR Slot Sensors)

- Use **interrupts on D2 (INT0)** and **D3 (INT1)** on Nano.
- Recommended edge: `RISING` or `FALLING` (not `CHANGE`)
- RPM Calculation:
  ```cpp
  RPM = (ticks / pulsesPerRevolution) * 60;
Use slotted disk with known number of holes (e.g., 20 pulses/rev).

🧭 MPU6050 Integration
Use only raw gyro data, not DMP mode (too heavy for Nano)

Read using:

cpp
Copy
Edit
mpu.getRotation(&gx, &gy, &gz);
float zRate = gz / 131.0; // degrees/second
Sample every 20–40ms

Use gyroZ to perform smooth angle-based turns:

Read initial gyroZ

Rotate until estimated delta angle = target

🧩 Smart Turning Logic
Nano should not assume “left” = 90°

Raspberry Pi decides how much to turn (e.g., 25°, 60°, etc.)

Pi sends command like: T-30 (turn left 30°)

Nano executes turn using gyro feedback

✅ Nano Design Philosophy
Task	Status / Recommendation
Motor PWM	✅ Use analogWrite()
Encoder reading	✅ Use attachInterrupt()
MPU6050 integration	✅ Use raw gyro only
Float math	⚠️ Limit usage, avoid complex ops
Serial parsing	✅ Use lightweight string/char logic
DMP / Quaternion	❌ Avoid — not Nano-friendly

🔧 Future Improvements
Add encoder-based PID control

Use encoder + gyro for simple odometry

Improve turn accuracy with angle smoothing

Add battery level sensing

Use CRC/checksum for robust command transfer

Visualize path planning via Pi web interface

🧠 Raspberry Pi Multitasking and Integration
The Raspberry Pi runs several demanding tasks simultaneously:

YOLOv5-based obstacle detection (heavy computation)

Real-time camera feed with OpenCV

Serial communication with Arduino Nano

Calculations for decision-making (turn angles, speeds)

Can the Pi handle this?
Yes, but careful programming is essential to avoid blocking and lag:

Use multithreading or asynchronous programming to separate tasks.

Avoid blocking calls like waiting on serial or camera frame reads.

Consider running vision in a dedicated thread or process.

Share data between threads using thread-safe variables or locks.

If YOLOv5 slows down the loop, use a lighter model or lower frame rate.

Example Python Script Structure
A typical approach is to run separate threads for:

Serial communication with the Nano (reading sensor data, sending commands)

Camera capture and YOLO detection

Control logic that decides car movement based on sensor and vision data

python
Copy
Edit
import threading
import serial
import time
import cv2

# ... (example code in the next section)
Refer to the "Example Python Script" section below.

🧩 Example Python Script Skeleton
python
Copy
Edit
import threading
import serial
import time
import cv2

# ========== Setup Serial Communication ==========
try:
    nano = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    time.sleep(2)  # Wait for Nano to reboot
    print("[INFO] Connected to Arduino Nano")
except:
    print("[ERROR] Could not connect to Nano")

# ========== Shared Variables ==========
imu_data = {}
rpm_data = {}
object_detected = None
lock = threading.Lock()

# ========== Serial Reading Thread ==========
def parse_line(line):
    if line.startswith("IMU:"):
        parts = line.replace("IMU:", "").split(",")
        return {"gz": float(parts[2])}
    elif line.startswith("RPM:"):
        parts = line.replace("RPM:", "").split(",")
        return {"left": float(parts[0]), "right": float(parts[1])}
    return {}

def read_nano():
    global imu_data, rpm_data
    while True:
        try:
            line = nano.readline().decode().strip()
            if not line:
                continue
            lock.acquire()
            if "IMU:" in line:
                imu_data = parse_line(line)
            elif "RPM:" in line:
                rpm_data = parse_line(line)
            lock.release()
        except Exception as e:
            print(f"[Serial Error] {e}")
        time.sleep(0.01)

# ========== Vision Detection Thread ==========
def dummy_yolo(frame):
    # Fake detection: return obstacle on left or right randomly
    import random
    direction = random.choice(["left", "right", "center", None])
    return direction

def vision_thread():
    global object_detected
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        direction = dummy_yolo(frame)
        lock.acquire()
        object_detected = direction
        lock.release()
        # Optional: show video
        # cv2.imshow("Cam", frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break
        time.sleep(0.1)

# ========== Control Decision Thread ==========
def control_loop():
    while True:
        lock.acquire()
        direction = object_detected
        lock.release()

        if direction == "left":
            print("[CTRL] Obstacle on LEFT → Turning RIGHT")
            nano.write(b'T30\n')
        elif direction == "right":
            print("[CTRL] Obstacle on RIGHT → Turning LEFT")
            nano.write(b'T-30\n')
        elif direction == "center":
            print("[CTRL] Obstacle AHEAD → Stopping")
            nano.write(b'S\n')
        else:
            print("[CTRL] Clear → Forward")
            nano.write(b'F\n')

        time.sleep(0.2)

# ========== Thread Start ==========
if __name__ == "__main__":
    threading.Thread(target=read_nano, daemon=True).start()
    threading.Thread(target=vision_thread, daemon=True).start()
    control_loop()
🗂️ File Structure Suggestion (for GitHub Repo)
bash
Copy
Edit
/mini-self-driving-car/
│
├── /nano/
│   └── nano_motor_control.ino
│
├── /pi/
│   ├── serial_control.py
│   └── yolov5_obstacle.py
│
├── /docs/
│   └── project-notes.md
│
└── README.md
🤝 Credits & Contributions
System architecture: [Your Name]

Arduino logic & testing: [Your Name]

YOLOv5 model integration: [To be done]

Inspired by open-source robotics and DIY AI community

📅 Status
Project is in early development. Hardware integration completed, basic motion verified. Vision model and smarter control logic in progress.
