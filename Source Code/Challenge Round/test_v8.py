import numpy as np
import cv2
from jetcam.csi_camera import CSICamera
from threading import Thread
from flask import Flask, Response
import serial
import time
from collections import deque


class CameraHandler:
    def __init__(self, use_windows_camera=False, camera_index=0, width=272, height=204, capture_width=640, capture_height=480, capture_fps=90):
        self.use_windows_camera = use_windows_camera
        self.current_image = None

        if self.use_windows_camera:
            self.camera = cv2.VideoCapture(camera_index)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        else:
            self.camera1 = CSICamera(capture_device=1, width=width, height=height, capture_width=capture_width, capture_height=capture_height, capture_fps=capture_fps, flip_method=2)
            self.camera2 = CSICamera(capture_device=0, width=width, height=height, capture_width=capture_width, capture_height=capture_height, capture_fps=capture_fps, flip_method=2)

            self.camera1.running = True
            self.camera2.running = True

        self.start_camera()

    def start_camera(self):
        def capture_frames():
            while True:
                if self.use_windows_camera:
                    ret, frame = self.camera.read()
                    if ret:
                        self.current_image = frame
                else:
                    self.current_image = cv2.hconcat([self.camera1.value, self.camera2.value])
                time.sleep(0.01)
        Thread(target=capture_frames).start()

    def get_frame(self):
        if self.use_windows_camera:
            return cv2.flip(self.current_image, 1)
        return cv2.rotate(self.current_image, cv2.ROTATE_180)


class ImageProcessor:
    def __init__(self):
        # Adjusted HSV ranges for better detection
        # Red detection range
        self.red_min1 = (0, 70, 50)
        self.red_max1 = (5, 255, 255)
        self.red_min2 = (160, 70, 50)
        self.red_max2 = (180, 255, 255)
        # Green detection range - adjusted for better detection
        self.green_min = (35, 50, 50)
        self.green_max = (85, 255, 255)
        # Orange detection range (CMYK to HSV conversion)
        self.orange_min = (5, 100, 100)
        self.orange_max = (25, 255, 255)
        # Blue detection range (CMYK to HSV conversion)
        self.blue_min = (100, 100, 50)
        self.blue_max = (130, 255, 255)
        # Black detection range
        self.black_min = (0, 5, 0)
        self.black_max = (180, 255, 70)  # Adjusted V max to 70
        # Initialize deque for tracking bounding boxes
        self.red_track = deque(maxlen=5)
        self.green_track = deque(maxlen=5)

    def split_image(self, frame):
        hsv_full = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the detection area
        detection_top = 65
        detection_bottom = 175
        hsv_detect = hsv_full[detection_top:detection_bottom, :]

        # Red mask
        red_mask1 = cv2.inRange(hsv_detect, self.red_min1, self.red_max1)
        red_mask2 = cv2.inRange(hsv_detect, self.red_min2, self.red_max2)
        red_mask_detect = cv2.bitwise_or(red_mask1, red_mask2)

        # Green mask
        green_mask_detect = cv2.inRange(hsv_detect, self.green_min, self.green_max)

        # Orange mask
        orange_mask_detect = cv2.inRange(hsv_detect, self.orange_min, self.orange_max)

        # Blue mask
        blue_mask_detect = cv2.inRange(hsv_detect, self.blue_min, self.blue_max)

        # Exclude red and green areas from black detection
        red_green_mask_detect = cv2.bitwise_or(red_mask_detect, green_mask_detect)
        hsv_no_red_green_detect = cv2.bitwise_and(hsv_detect, hsv_detect, mask=cv2.bitwise_not(red_green_mask_detect))

        # Black mask using HSV thresholds on hsv_no_red_green
        black_mask_detect = cv2.inRange(hsv_no_red_green_detect, self.black_min, self.black_max)

        # Create full-frame masks
        mask_shape = (frame.shape[0], frame.shape[1])
        black_mask = np.zeros(mask_shape, dtype=np.uint8)
        green_mask = np.zeros(mask_shape, dtype=np.uint8)
        red_mask = np.zeros(mask_shape, dtype=np.uint8)
        orange_mask = np.zeros(mask_shape, dtype=np.uint8)
        blue_mask = np.zeros(mask_shape, dtype=np.uint8)

        # Place detection masks into full-frame masks
        black_mask[detection_top:detection_bottom, :] = black_mask_detect
        green_mask[detection_top:detection_bottom, :] = green_mask_detect
        red_mask[detection_top:detection_bottom, :] = red_mask_detect
        orange_mask[detection_top:detection_bottom, :] = orange_mask_detect
        blue_mask[detection_top:detection_bottom, :] = blue_mask_detect

        # Apply median blur and morphological operations
        kernel = np.ones((3, 3), np.uint8)
        black_mask = cv2.morphologyEx(cv2.medianBlur(black_mask, 5), cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(cv2.medianBlur(green_mask, 5), cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(cv2.medianBlur(red_mask, 5), cv2.MORPH_OPEN, kernel)
        orange_mask = cv2.morphologyEx(cv2.medianBlur(orange_mask, 5), cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(cv2.medianBlur(blue_mask, 5), cv2.MORPH_OPEN, kernel)

        return frame, black_mask, green_mask, red_mask, orange_mask, blue_mask

    def detect_blobs(self, mask, color):
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 100:  # Adjusted area threshold
                x, y, w, h = cv2.boundingRect(cnt)
                cX = x + w // 2
                cY = y + h // 2
                detection = {'contour': cnt, 'bbox': (x, y, w, h), 'area': area, 'centroid': (cX, cY)}
                detections.append(detection)
        return detections

    def get_largest_detection(self, detections):
        if not detections:
            return None
        # Return the detection with the largest area
        largest_detection = max(detections, key=lambda d: d['area'])
        return largest_detection


class WallAnalyzer:
    def __init__(self):
        # PID controller parameters
        self.Kp = 3000.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.previous_error = 0
        self.integral = 0
        self.last_time = None

    def detect_walls(self, black_mask):
        height, width = black_mask.shape

        # Wall detection rectangle from 100 to 150 in full frame
        vertical_start = 90
        vertical_end = 140

        # Left ROI
        left_roi = black_mask[vertical_start:vertical_end, :width // 2]

        # Right ROI
        right_roi = black_mask[vertical_start:vertical_end, width // 2:]

        # Calculate the percentage of black pixels in each ROI
        left_black_pixels = cv2.countNonZero(left_roi)
        right_black_pixels = cv2.countNonZero(right_roi)

        left_total_pixels = left_roi.shape[0] * left_roi.shape[1]
        right_total_pixels = right_roi.shape[0] * right_roi.shape[1]

        left_black_ratio = left_black_pixels / left_total_pixels if left_total_pixels > 0 else 0
        right_black_ratio = right_black_pixels / right_total_pixels if right_total_pixels > 0 else 0

        return left_black_ratio, right_black_ratio, left_roi, right_roi

    def calculate_wall_turn(self, left_black_ratio, right_black_ratio):
        error = right_black_ratio - left_black_ratio

        # PID calculations
        current_time = time.time()
        dt = current_time - self.last_time if self.last_time else 0.1
        self.last_time = current_time

        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        self.previous_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Limit the output
        output = max(min(output, 1000), -1000)

        return output


class TurnCalculator:
    def __init__(self):
        self.traffic_past = 0

    def calculate_traffic_turn(self, red_detections, green_detections):
        # Only consider the largest red and green detections
        largest_red = image_processor.get_largest_detection(red_detections)
        largest_green = image_processor.get_largest_detection(green_detections)

        target = [0, None]
        if largest_red and (not largest_green or largest_red['area'] > largest_green['area']):
            target = [1, largest_red]
        elif largest_green:
            target = [2, largest_green]

        traffic_turn = 0
        if target[0]:
            # Turn calculation based on detected color
            if target[0] == 1:  # Red detected
                traffic_turn = -500  # Sharp left turn
            elif target[0] == 2:  # Green detected
                traffic_turn = 500  # Sharp right turn

        # Smooth the turn value
        traffic_turn = max(min(traffic_turn, 500), -500)
        self.traffic_past = self.traffic_past * 0.7 + traffic_turn * 0.3
        return self.traffic_past

    def calculate_final_turn(self, wall_turn, traffic_turn):
        # Combine wall turn and traffic turn with weighting
        final_turn = wall_turn + traffic_turn
        return max(min(final_turn, 1000), -1000)


class CarController:
    def __init__(self, serial_port='/dev/ttyUSB0', baudrate=9600):
        self.serial = serial.Serial(port=serial_port, baudrate=baudrate, timeout=10)
        pass

    def send_turn_command(self, turn_value):
        mapped_turn = 100 - int(np.interp(turn_value, [-1000, 1000], [0, 100]))
        self.serial.write(bytes(chr(mapped_turn), 'utf-8'))
        pass  # No console printing as per your request


class DebugServer:
    def __init__(self, camera_handler, image_processor, wall_analyzer, turn_calculator, car_controller):
        self.camera_handler = camera_handler
        self.image_processor = image_processor
        self.turn_calculator = turn_calculator
        self.car_controller = car_controller
        self.wall_analyzer = wall_analyzer
        self.app = Flask(__name__)

    def run_debug(self):
        while True:
            frame = self.camera_handler.get_frame()
            frame_with_detections, black_mask, green_mask, red_mask, orange_mask, blue_mask = self.image_processor.split_image(frame.copy())

            # Get frame dimensions
            height, width, _ = frame.shape

            # Detect walls
            left_black_ratio, right_black_ratio, left_roi, right_roi = self.wall_analyzer.detect_walls(black_mask)

            # Calculate wall turn using PID controller
            wall_turn = self.wall_analyzer.calculate_wall_turn(left_black_ratio, right_black_ratio)

            # Detect traffic lights
            red_detections = self.image_processor.detect_blobs(red_mask, 'red')
            green_detections = self.image_processor.detect_blobs(green_mask, 'green')
            orange_detections = self.image_processor.detect_blobs(orange_mask, 'orange')
            blue_detections = self.image_processor.detect_blobs(blue_mask, 'blue')

            # Combine wall turn and traffic turn
            traffic_turn = self.turn_calculator.calculate_traffic_turn(red_detections, green_detections)
            final_turn = self.turn_calculator.calculate_final_turn(wall_turn, traffic_turn)

            # Send the turn command
            self.car_controller.send_turn_command(final_turn)

            # For debugging, we can draw on the frame
            # Convert masks to BGR images for visualization
            black_mask_bgr = cv2.cvtColor(black_mask, cv2.COLOR_GRAY2BGR)

            # Draw ROIs
            cv2.rectangle(frame, (0, 100), (width // 2, 150), (255, 0, 0), 2)  # Left ROI
            cv2.rectangle(frame, (width // 2, 100), (width, 150), (0, 255, 0), 2)  # Right ROI

            # Draw the contours of orange and blue detections
            for detection in orange_detections:
                cnt = detection['contour']
                cv2.drawContours(frame, [cnt], -1, (0, 165, 255), 2)  # Orange color in BGR

            for detection in blue_detections:
                cnt = detection['contour']
                cv2.drawContours(frame, [cnt], -1, (255, 0, 0), 2)  # Blue color in BGR

            # Draw bounding boxes around red and green detections
            largest_red = self.image_processor.get_largest_detection(red_detections)
            largest_green = self.image_processor.get_largest_detection(green_detections)

            if largest_red:
                x, y, w, h = largest_red['bbox']
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            else:
                self.image_processor.red_track.clear()

            if largest_green:
                x, y, w, h = largest_green['bbox']
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            else:
                self.image_processor.green_track.clear()

            # Display contours' area on the frame
            for detection in orange_detections:
                area = detection['area']
                x, y, w, h = detection['bbox']
                cv2.putText(frame, f'OArea: {area}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

            for detection in blue_detections:
                area = detection['area']
                x, y, w, h = detection['bbox']
                cv2.putText(frame, f'BArea: {area}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            # Draw turn values on the frame
            cv2.putText(frame, f'Wall Turn: {wall_turn:.2f}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f'Traffic Turn: {traffic_turn:.2f}', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f'Final Turn: {final_turn:.2f}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # Overlay the black mask on the frame for visualization
            combined_frame = cv2.addWeighted(frame, 0.7, black_mask_bgr, 0.3, 0)

            _, buffer = cv2.imencode('.jpg', combined_frame)

            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    def start_server(self):
        @self.app.route('/')
        def index():
            return Response(self.run_debug(), mimetype='multipart/x-mixed-replace; boundary=frame')

        self.app.run(host='0.0.0.0', port=5000)


class MainRunner:
    def __init__(self, camera_handler, image_processor, wall_analyzer, turn_calculator, car_controller):
        self.camera_handler = camera_handler
        self.image_processor = image_processor
        self.wall_analyzer = wall_analyzer
        self.turn_calculator = turn_calculator
        self.car_controller = car_controller

    def run(self):
        while True:
            frame = self.camera_handler.get_frame()
            frame_with_detections, black_mask, green_mask, red_mask, orange_mask, blue_mask = self.image_processor.split_image(frame.copy())

            # Get frame dimensions
            height, width, _ = frame.shape

            # Detect walls
            left_black_ratio, right_black_ratio, left_roi, right_roi = self.wall_analyzer.detect_walls(black_mask)

            # Calculate wall turn using PID controller
            wall_turn = self.wall_analyzer.calculate_wall_turn(left_black_ratio, right_black_ratio)

            # Detect traffic lights
            red_detections = self.image_processor.detect_blobs(red_mask, 'red')
            green_detections = self.image_processor.detect_blobs(green_mask, 'green')
            traffic_turn = self.turn_calculator.calculate_traffic_turn(red_detections, green_detections)

            # Combine wall turn and traffic turn
            final_turn = self.turn_calculator.calculate_final_turn(wall_turn, traffic_turn)

            print(final_turn)
            self.car_controller.send_turn_command(final_turn)



if __name__ == '__main__':
    camera_handler = CameraHandler()
    image_processor = ImageProcessor()
    wall_analyzer = WallAnalyzer()
    turn_calculator = TurnCalculator()
    car_controller = CarController()

    # Uncomment this to run in normal mode (without Flask debug mode)
    #runner = MainRunner(camera_handler, image_processor, wall_analyzer, turn_calculator, car_controller)
    #runner.run()

    # To use the debug mode and stream via Flask:
    server = DebugServer(camera_handler, image_processor, wall_analyzer, turn_calculator, car_controller)
    server.start_server()

