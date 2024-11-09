import cv2
import numpy as np
import pyautogui
import keyboard
import time
from yolov5 import detect  # Assuming you have a YOLO model for obstacle detection

def capture_screen(region=None):
    """Capture a portion of the screen or the entire screen."""
    screenshot = pyautogui.screenshot(region=region)
    frame = np.array(screenshot)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return frame

def process_frame_for_path(frame):
    """Process the frame to detect lanes/path."""
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (0, height),
        (width, height),
        (width, height // 2),
        (0, height // 2)
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_edges = cv2.bitwise_and(edges, mask)

    return masked_edges

def detect_obstacles(frame):
    """Use a pre-trained YOLO model to detect obstacles."""
    # You can use a pre-trained YOLOv5 model here to detect objects
    detections = detect.run(source=frame, conf_thres=0.5, save_conf=False)
    return detections

def make_decision(lines, obstacles, width):
    """Decide on the car's movement based on detected lines and obstacles."""
    left_lane = []
    right_lane = []

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1 + 1e-6)  # Avoid division by zero
                if slope < -0.5:  # Left lane
                    left_lane.append(line)
                elif slope > 0.5:  # Right lane
                    right_lane.append(line)

    # Check for obstacles in the path
    obstacle_in_path = any(d['name'] == 'car' or d['name'] == 'pedestrian' for d in obstacles)

    if obstacle_in_path:
        return 'slow'  # Slow down or prepare to reverse
    elif len(left_lane) > len(right_lane):
        return 'left'  # Steer left
    elif len(right_lane) > len(left_lane):
        return 'right'  # Steer right
    else:
        return 'straight'  # Accelerate forward

def press_key(key, duration=0.1):
    """Simulate a key press for a specified duration."""
    keyboard.press(key)
    time.sleep(duration)
    keyboard.release(key)

def main():
    print("Starting the game automation script...")
    time.sleep(3)  # Delay to switch to the game screen

    try:
        while True:
            screen = capture_screen(region=(0, 0, 800, 600))
            processed_frame = process_frame_for_path(screen)
            lines = cv2.HoughLinesP(processed_frame, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=100)
            obstacles = detect_obstacles(screen)

            decision = make_decision(lines, obstacles, screen.shape[1])
            print(f"Decision: {decision}")

            if decision == 'left':
                press_key('a', duration=0.2)
            elif decision == 'right':
                press_key('d', duration=0.2)
            elif decision == 'slow':
                press_key('s', duration=0.5)  # Slow down or reverse if necessary
            else:
                press_key('w', duration=0.1)  # Accelerate forward

            # Display the processed frame for debugging
            cv2.imshow('Processed Frame', processed_frame)

            # Exit condition (press 'q' to quit the script)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Automation script stopped by user.")
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
