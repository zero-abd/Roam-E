import time
import numpy as np
import cv2
from jetcam.csi_camera import CSICamera
from threading import Thread
from flask import Flask, Response
import serial


class CameraHandler:
    def __init__(self, use_windows_camera=True, camera_index=0, width=272, height=204, capture_width=3280, capture_height=2464, capture_fps=21):
        self.use_windows_camera = use_windows_camera
        self.current_image = None

        if self.use_windows_camera:
            self.camera = cv2.VideoCapture(camera_index)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        else:
            self.camera = CSICamera(width=width, height=height, capture_width=capture_width, capture_height=capture_height, capture_fps=capture_fps)
        
        self.start_camera()

    def start_camera(self):
        def capture_frames():
            while True:
                if self.use_windows_camera:
                    ret, frame = self.camera.read()
                    if ret:
                        self.current_image = frame
                else:
                    self.current_image = self.camera.value
                time.sleep(0.01)
        Thread(target=capture_frames).start()

    def get_frame(self):
        if self.use_windows_camera:
            return cv2.flip(self.current_image, 1)
        return cv2.rotate(self.current_image, cv2.ROTATE_180)


class ImageProcessor:
    def __init__(self):
        self.red_min1 = (0, 35, 75)
        self.red_max1 = (5, 255, 255)
        self.red_min2 = (175, 35, 75)
        self.red_max2 = (180, 255, 255)
        self.green_min = (70, 25, 25)
        self.green_max = (90, 255, 255)
        self.blob_detector = self.create_blob_detector()

    def create_blob_detector(self):
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 50
        return cv2.SimpleBlobDetector_create(params)

    def split_image(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_mask = cv2.bitwise_or(cv2.inRange(hsv, self.red_min1, self.red_max1), cv2.inRange(hsv, self.red_min2, self.red_max2))
        green_mask = cv2.inRange(hsv, self.green_min, self.green_max)
        gray_image = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        wall_edges = cv2.Canny(cv2.GaussianBlur(gray_image, (3, 3), 0), 100, 125, 3)
        return wall_edges, green_mask, red_mask

    def detect_blobs(self, image):
        return self.blob_detector.detect(image)


class WallAnalyzer:
    @staticmethod
    def split_wall(wall_image):
        wall_heights_np = (wall_image != 0).argmax(axis=1)
        wall_heights = [wall_heights_np[i:i+34] for i in range(0, 272, 34)]
        wall_slopes = [np.mean(np.diff(heights)) for heights in wall_heights]
        wall_heights_avg = [np.mean(heights) for heights in wall_heights]
        return WallAnalyzer.classify_wall(wall_heights_avg, wall_slopes)

    @staticmethod
    def classify_wall(heights, slopes):
        wall_type = [0] * 8
        sp = False
        for i in range(6, 2, -1):
            if sp or heights[i] - heights[i+1] > 4:
                wall_type[i] = 1
                sp = True

        sp = False
        for i in range(2, 6):
            if sp or heights[i+1] - heights[i] > 4:
                wall_type[i+1] = 2
                sp = True

        wall_type[0], wall_type[1], wall_type[6], wall_type[7] = 1, 1, 2, 2
        return wall_type, heights

    def calculate_wall_turn(self, wall_data, clockwise):
        turn = 0
        for i, section in enumerate(wall_data[0]):
            tmp_turn = 10 if section == 0 else (-20 if section == 2 else 0)
            turn += tmp_turn
        return max(min(turn, 1000), -1000)


class TurnCalculator:
    def __init__(self):
        self.traffic_past = 0

    def calculate_traffic_turn(self, red_points, green_points):
        target, threshold = [0, None], 50
        for points, index in [(red_points, 1), (green_points, 2)]:
            for point in points:
                if point.pt[0] > threshold and (not target[1] or target[1].size < point.size):
                    target = [index, point]

        traffic_turn = 0
        if target[0]:
            result = ((target[1].size / 10) ** 4) * (target[1].pt[0] * 0.1 - 50)
            traffic_turn = max(result * (1 if target[0] == 1 else -1), 0)

        traffic_turn = max(min(traffic_turn, 500), -500)
        self.traffic_past *= 0.9
        return traffic_turn + self.traffic_past * 0.9

    def calculate_final_turn(self, wall_turn, traffic_turn):
        return wall_turn + traffic_turn


class CarController:
    def __init__(self, serial_port='/dev/ttyUSB0', baudrate=9600):
        # self.serial = serial.Serial(port=serial_port, baudrate=baudrate, timeout=10)
        pass

    def send_turn_command(self, turn_value):
        mapped_turn = int(np.interp(turn_value, [-1200, 1200], [0, 110]))
        self.serial.write(bytes(chr(mapped_turn), 'utf-8'))


class DebugServer:
    def __init__(self, camera_handler, image_processor, turn_calculator, car_controller):
        self.camera_handler = camera_handler
        self.image_processor = image_processor
        self.turn_calculator = turn_calculator
        self.car_controller = car_controller
        self.app = Flask(__name__)

    def run_debug(self):
        while True:
            frame = self.camera_handler.get_frame()
            wall_edges, green_mask, red_mask = self.image_processor.split_image(frame)

            red_detections = self.image_processor.detect_blobs(255 - red_mask[79:100])
            green_detections = self.image_processor.detect_blobs(255 - green_mask[79:100])
            traffic_turn = self.turn_calculator.calculate_traffic_turn(red_detections, green_detections)

            final_turn = self.turn_calculator.calculate_final_turn(0, traffic_turn)

            combined_frame = cv2.merge((wall_edges, green_mask, red_mask))
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
        self.clockwise = 1

    def run(self):
        while True:
            frame = self.camera_handler.get_frame()
            wall_edges, green_mask, red_mask = self.image_processor.split_image(frame)

            wall_vimage = np.swapaxes(np.concatenate((wall_edges[79:130, :272], np.full((2, 272), 1)), axis=0), 0, 1)
            walls, heights = self.wall_analyzer.split_wall(wall_vimage)
            wall_turn = self.wall_analyzer.calculate_wall_turn((walls, heights), self.clockwise)

            red_detections = self.image_processor.detect_blobs(255 - red_mask[79:100])
            green_detections = self.image_processor.detect_blobs(255 - green_mask[79:100])
            traffic_turn = self.turn_calculator.calculate_traffic_turn(red_detections, green_detections)

            final_turn = self.turn_calculator.calculate_final_turn(wall_turn, traffic_turn)
            self.car_controller.send_turn_command(final_turn)

            print(f"Wall turn: {wall_turn}, Traffic turn: {traffic_turn}, Final turn: {final_turn}")
            time.sleep(0.05)


if __name__ == '__main__':
    camera_handler = CameraHandler()
    image_processor = ImageProcessor()
    wall_analyzer = WallAnalyzer()
    turn_calculator = TurnCalculator()
    car_controller = CarController()

    runner = MainRunner(camera_handler, image_processor, wall_analyzer, turn_calculator, car_controller)

    # Uncomment this to run in normal mode (without Flask debug mode)
    # runner.run()

    # If you want to use the debug mode and stream via Flask, uncomment this:
    server = DebugServer(camera_handler, image_processor, turn_calculator, car_controller)
    server.start_server()
