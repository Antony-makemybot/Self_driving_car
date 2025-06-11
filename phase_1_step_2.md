
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
