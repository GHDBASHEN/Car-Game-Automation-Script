# Car-Game-Automation-Script

A Python-based automation script for a car game that enables the car to autonomously navigate by understanding when to accelerate, steer left or right, and slow down. The script uses computer vision techniques to identify paths and detect obstacles in real-time.

## Features
- **Path Detection**: Detects lanes and road paths using edge detection and Hough Line Transform.
- **Obstacle Detection**: Integrates a pre-trained YOLO object detection model to identify obstacles like other vehicles and pedestrians.
- **Decision-Making Logic**: Implements logic for deciding when to steer, accelerate, or slow down based on detected paths and obstacles.

## Requirements
- Python 3.x
- OpenCV (`cv2`)
- NumPy
- PyAutoGUI
- Keyboard
- YOLOv5 (or any other pre-trained object detection model)
- Time module (built-in)

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/GHDBASHEN/Car-Game-Automation-Script.git
   cd Car-Game-Automation-Script
 2.Install this libraries too:
   ```bash
      pip install opencv-python pyautogui keyboard
