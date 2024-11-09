import cv2
import numpy as np
import pyautogui
import keyboard
import time

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

def detect_lines(frame):
    """Detect lines using Hough Line Transform."""
    lines = cv2.HoughLinesP(frame, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=100)
    return lines

def make_decision(lines, width):
    """Decide on the car's movement based on detected lines."""
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

    if len(left_lane) > len(right_lane):
        return 'left'  # Steer left
    elif len(right_lane) > len(left_lane):
        return 'right'  # Steer right
    else:
        return 'straight'  # Go straight

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
            lines = detect_lines(processed_frame)
            
            decision = make_decision(lines, screen.shape[1])
            print(f"Decision: {decision}")

            if decision == 'left':
                press_key('a', duration=0.2)
            elif decision == 'right':
                press_key('d', duration=0.2)
            else:
                press_key('w', duration=0.1)  # Accelerate forward

            # Exit condition (press 'q' to quit the script)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Display the processed frame for debugging
            cv2.imshow('Processed Frame', processed_frame)

    except KeyboardInterrupt:
        print("Automation script stopped by user.")
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
