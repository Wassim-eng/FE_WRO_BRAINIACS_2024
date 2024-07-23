import cv2
import numpy as np
import time
import serial

KNOWN_HEIGHT = 100

LOWER_GREEN = np.array([80, 50, 50])
UPPER_GREEN = np.array([100, 255, 255])

LOWER_RED1 = np.array([0, 100, 75])
UPPER_RED1 = np.array([5, 255, 255])
LOWER_RED2 = np.array([170, 100, 75])
UPPER_RED2 = np.array([180, 255, 255])

# Focal length and optical center 
f_x = 600
f_y = 600
c_x = 320
c_y = 240

baud_rate = 115200
port = '/dev/ttyUSB0'
ser = serial.Serial(port, baud_rate,timeout = 15)  # Adjust port and baud rate as necessary
time.sleep(1)


def find_distance_to_object(known_height, focal_length, perceived_height):
    return (known_height * focal_length) / perceived_height

def find_horizontal_angle(c_x, perceived_x, f_x):
    return np.arctan((perceived_x - c_x) / f_x)

def data_to_arduino(horizontal_angle_degrees, color_send, distance_mm):
    angle_int = int(horizontal_angle_degrees)  # Convert to integer
    distance_int = int(distance_mm) #Type casting the distance to an integer for easier comparison in arduino
    data = f"{color_send}{angle_int},{distance_int}\n"
    print(data)
    ser.write(data.encode())

def main():
    # Attempt to use camera index 0
    cap = cv2.VideoCapture(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # Width
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # Height

    if not cap.isOpened():
        print("Camera index 0 not available, trying index 1")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Camera index 1 not available either, exiting")
            return

    closest_green_distance = float('inf')
    closest_green_rect = None
    closest_red_distance = float('inf')
    closest_red_rect = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask_green = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
        mask_green = cv2.medianBlur(mask_green, 5)  # Apply median blur to reduce noise

        mask_red1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
        mask_red2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_red = cv2.medianBlur(mask_red, 5)  # Apply median blur to reduce noise

        kernel = np.ones((12, 12), np.uint8)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Reset closest distances and rectangles
        closest_green_distance = float('inf')
        closest_green_rect = None
        closest_red_distance = float('inf')
        closest_red_rect = None

        for contour in contours_red:
            area = cv2.contourArea(contour)
            if area < 800:
                continue

            x, y, width, height = cv2.boundingRect(contour)
            perceived_height = height

            distance = find_distance_to_object(KNOWN_HEIGHT, f_y, perceived_height)

            # Update closest red object
            if distance < closest_red_distance:
                closest_red_distance = distance
                closest_red_rect = (x, y, width, height)

        for contour in contours_green:
            area = cv2.contourArea(contour)
            if area < 1100:
                continue

            x, y, width, height = cv2.boundingRect(contour)
            perceived_height = height

            distance = find_distance_to_object(KNOWN_HEIGHT, f_y, perceived_height)

            # Update closest green object
            if distance < closest_green_distance:
                closest_green_distance = distance
                closest_green_rect = (x, y, width, height)

        closest_color_distance = min(closest_red_distance, closest_green_distance)
        closest_color_rect = closest_red_rect if closest_color_distance == closest_red_distance else closest_green_rect

        # Draw bounding rectangles for closest objects and display the distance and angle
        if closest_color_rect is not None:
            x, y, width, height = closest_color_rect
            perceived_x = x + width / 2
            perceived_y = y + height / 2
            horizontal_angle = find_horizontal_angle(c_x, perceived_x, f_x)
            
            if closest_color_rect == closest_red_rect:
                color_send = 'r'
            else:
                color_send = 'g'
                
            if closest_color_rect == closest_red_rect:
                color = (0, 0, 255)  # Red
                text_color = (0, 0, 255)
            else:
                color = (0, 255, 0)  # Green
                text_color = (0, 255, 0)

            # Draw bounding rectangle
            cv2.rectangle(frame, (x, y), (x + width, y + height), color, 2)

            # Draw horizontal and vertical lines to the center of the object
            cv2.line(frame, (int(perceived_x), 0), (int(perceived_x), frame.shape[0]), color, 1)
            cv2.line(frame, (0, int(perceived_y)), (frame.shape[1], int(perceived_y)), color, 1)

            # Display text with object information
            cv2.putText(frame, f"{'Red' if color == (0, 0, 255) else 'Green'} - Distance: {closest_color_distance:.2f} mm", (x, y - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
            cv2.putText(frame, f"Horizontal Angle: {np.degrees(horizontal_angle):.2f} degrees", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
            
            # Send horizontal angle to Arduino
            data_to_arduino(np.degrees(horizontal_angle), color_send, closest_color_distance)

        # Display the frame
        cv2.imshow('Frame', frame)
        
        # Check for 'q' key press to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
