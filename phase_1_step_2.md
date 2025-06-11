
🗺️ Phase 1 Goal: Basic Road Following
🎯 Objective
Use OpenCV on Raspberry Pi to:

Detect the white center line and left border line on a black chart

Keep the car centered between the left and center line

Move forward until the end of the chart

🛣️ Road Design (Your Description)
Element	Color	Description
Background	Black	Main road chart
Left border	White	Continuous white strip on left edge
Center line	White	Continuous white strip (1 cm) in center
Right border	White	Continuous white strip on right edge
Total lanes	2	A two-way road, but center line is continuous, not dashed

📌 For this phase, the car is only navigating between left border and center line.

🧠 Phase 1 Control Logic
Input	From	Action
Camera frame	Pi Camera	Detect white lines
Center offset	OpenCV	Calculate shift from target center
Command	Pi → Nano	Send F, T-xx, Txx, S
Gyro / Encoder	Nano → Pi	Feedback to adjust turn precisely

🧮 OpenCV Line Following Strategy
👁️ Vision Algorithm Plan
Capture frame from camera (top-down view recommended)

Convert to grayscale

Apply thresholding to highlight white lines on black background

Crop lower region of interest (ROI) (bottom of image = closer to car)

Find contours or white pixels in ROI

Detect line positions (center line and left line)

Calculate midpoint between left line and center line

Compare midpoint to image center → turn left/right/forward

🧪 Basic Vision Tuning Values
python
Copy
Edit
# Pseudocode constants
WHITE_THRESHOLD = 200  # for binary thresholding
ROI_Y = 100            # how many pixels from bottom to use as ROI
CENTER_OFFSET_TOLERANCE = 10  # pixels
🧠 Control Output Plan (sent to Nano)
Offset from center	Action
< –10 pixels	Send T-20 (turn left)
> +10 pixels	Send T20 (turn right)
Between ±10 pixels	Send F (go forward)

✅ Step-by-Step Execution Plan
✅ 1. Line Following
 Write Python OpenCV script

 Calibrate thresholds to detect white on black

 Extract left & center lines

 Compute midpoint between those two

 Compare midpoint to image center

 Send turn/forward commands to Nano

✅ 2. Command Handling
 Nano receives commands like F, T-20, T20

 Nano uses MPU6050 to track turning angle

 Nano moves car accordingly

🗂️ Suggested File Structure
bash
Copy
Edit
/mini-self-driving-car/
│
├── /nano/
│   └── nano_motor_control.ino
│
├── /pi/
│   └── line_following.py
│
├── /docs/
│   └── README.md
│
└── road.jpg  # sample road image for debugging


✅ line_following.py – OpenCV + Serial Integration (Phase 1)
python
Copy
Edit
import cv2
import numpy as np
import serial
import time

# === Serial Setup ===
try:
    nano = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    time.sleep(2)
    print("[INFO] Connected to Nano")
except:
    print("[ERROR] Could not connect to Nano")

# === Parameters ===
ROI_Y_START = 100  # Region from bottom
THRESHOLD = 180
TOLERANCE = 15
FRAME_WIDTH = 320

# === Pi Camera Setup ===
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

def send_command(cmd):
    print(f"[SEND] {cmd}")
    try:
        nano.write((cmd + '\n').encode())
    except Exception as e:
        print(f"[ERROR] Serial send failed: {e}")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, THRESHOLD, 255, cv2.THRESH_BINARY)

    roi = binary[-ROI_Y_START:, :]  # Bottom part of the frame
    roi_color = frame[-ROI_Y_START:, :]

    # Detect white pixels in left and center regions
    height, width = roi.shape
    center_line_x = int(width / 2)

    # Calculate white line positions
    left_line_x = None
    center_line_x_pos = None

    for x in range(width):
        column = roi[:, x]
        white_pixels = np.sum(column == 255)
        if white_pixels > 10:  # minimum line threshold
            if x < center_line_x and left_line_x is None:
                left_line_x = x
            elif x >= center_line_x and center_line_x_pos is None:
                center_line_x_pos = x

    if left_line_x is not None and center_line_x_pos is not None:
        midpoint = int((left_line_x + center_line_x_pos) / 2)
        offset = midpoint - center_line_x

        # Debug visuals
        cv2.line(roi_color, (left_line_x, 0), (left_line_x, ROI_Y_START), (255, 0, 0), 2)
        cv2.line(roi_color, (center_line_x_pos, 0), (center_line_x_pos, ROI_Y_START), (0, 255, 0), 2)
        cv2.line(roi_color, (midpoint, 0), (midpoint, ROI_Y_START), (0, 0, 255), 2)

        if abs(offset) < TOLERANCE:
            send_command("F")
        elif offset > 0:
            send_command("T20")  # Turn right
        else:
            send_command("T-20")  # Turn left

    else:
        send_command("S")  # Stop if line not detected

    cv2.imshow("ROI", roi_color)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
🧠 How It Works
ROI: It takes a slice from the bottom of the image to analyze.

Thresholding: Converts the image to black and white to detect white lines.

Logic:

Detects left and center white lines.

Calculates their midpoint.

Compares that midpoint to the actual image center.

Sends turn/forward/stop commands to Nano.

✅ What You Need on the Nano Side
Ensure your Nano reads single-line commands like:

cpp
Copy
Edit
// Arduino/Nano snippet
String input = "";

void loop() {
  if (Serial.available()) {
    input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "F") moveForward();
    else if (input == "S") stopMotors();
    else if (input.startsWith("T")) {
      int angle = input.substring(1).toInt();
      turnByAngle(angle); // Your gyro logic
    }
  }
}
