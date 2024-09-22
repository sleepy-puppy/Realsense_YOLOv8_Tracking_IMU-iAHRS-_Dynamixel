import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QPushButton, QWidget, QHBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
import pyrealsense2 as rs
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
import threading
import timeit
import rclpy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String
import subprocess
import time


class RealSenseObjectTracker(QMainWindow):
    def __init__(self):
        super().__init__()

        # UI 및 변수 초기화
        self.init_ui()
        self.init_vars()
        
        # Realsense 및 YOLO 모델 초기화
        self.pipeline, self.align = self.initialize_realsense()
        self.models = {
            "person": YOLO("yolov8m-pose.pt"),
            "knife": YOLO("knife_2.pt")
        }

        # Deep SORT Tracker 초기화
        self.trackers = {
            "person": DeepSort(max_age=30, n_init=3),
            "knife": DeepSort(max_age=30, n_init=3)
        }

        # ROS2 초기화
        rclpy.init()
        self.init_ros2_publishers()

        # 타이머 및 쓰레드 초기화
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.main_loop)
        self.timer.start(1)
        
        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.capture_thread.start()

    def init_ui(self):
        self.setGeometry(100, 100, 1280, 720)
        self.setWindowTitle("Police")
        self.main_layout = QVBoxLayout()
        self.label = QLabel(self)
        self.main_layout.addWidget(self.label)

        # 트래킹 모드 버튼
        self.manual_tracking_button = self.create_button("카메라 수동 트래킹", self.switch_to_manual_tracking)
        self.automatic_tracking_button = self.create_button("카메라 자동 트래킹", self.switch_to_automatic_tracking)
        self.audio_comm_button = self.create_button("현장 통신", self.send_audio_comm)
        self.stop_audio_comm_button = self.create_button("현장 통신 중지", self.send_stop_audio_comm)
        self.start_tracking_button = self.create_button("로봇 트래킹 시작", self.start_tracking)
        self.stop_tracking_button = self.create_button("로봇 트래킹 중지", self.stop_tracking)

        self.autonomous_driving_button = self.create_button("로봇 자동 주행", self.start_autonomous_driving)

        self.button_layout = QHBoxLayout()
        self.main_layout.addLayout(self.button_layout)
        central_widget = QWidget(self)
        central_widget.setLayout(self.main_layout)
        self.setCentralWidget(central_widget)

    def create_button(self, text, func):
        button = QPushButton(text, self)
        button.clicked.connect(func)
        self.main_layout.addWidget(button)
        return button

    def init_vars(self):
        self.tracking = False
        self.target_id = None
        self.global_center_x = 0
        self.global_center_y = 0
        self.last_detection_time = {}
        self.buttons = {}
        self.label_history = {}
        self.dangerous_person_time = {}
        self.person_tracks = []
        self.DISAPPEAR_THRESHOLD = 3
        self.DANGER_THRESHOLD = 1
        self.frame = None
        self.stop_thread = False

    def initialize_realsense(self):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        pipeline.start(config)
        align = rs.align(rs.stream.color)
        return pipeline, align

    def init_ros2_publishers(self):
        self.node = rclpy.create_node('realsense_tracker_node')
        self.publisher = self.node.create_publisher(Int32MultiArray, 'to_dynamixel', 10)
        self.start_signal_publisher = self.node.create_publisher(String, 'start_signal', 10)
        self.tracking_mode_publisher = self.node.create_publisher(String, 'camera_tracking_mode', 10)
        self.reset_signal_publisher = self.node.create_publisher(String, 'reset_signal', 10)
        self.audio_comm_publisher = self.node.create_publisher(String, 'audio_comm', 10)
        self.emergency_publisher = self.node.create_publisher(String, 'emergency', 10)
        self.target_person_depth_publisher = self.node.create_publisher(Float32MultiArray, 'target_person_depth', 10)
        self.robot_tracking_mode_publisher = self.node.create_publisher(String, 'robot_tracking_mode', 10)

    def capture_frames(self):
        while not self.stop_thread:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if color_frame and depth_frame:
                self.frame = np.asanyarray(color_frame.get_data())
                self.depth_frame = depth_frame

    def main_loop(self):
        if self.frame is None:
            return

        start_t = timeit.default_timer()
        current_frame = self.frame.copy()

        # 사람 및 칼 탐지 및 추적
        person_detections, person_keypoints = self.detect_objects("person", current_frame)
        knife_detections, _ = self.detect_objects("knife", current_frame)

        self.person_tracks = self.track_objects("person", person_detections, current_frame)
        knife_tracks = self.track_objects("knife", knife_detections, current_frame)

        closest_person_id = None
        if knife_tracks:
            for knife_track in knife_tracks:
                knife_center = self.get_center(knife_track.to_ltrb())
                closest_person_id = self.find_closest_hand(knife_center, self.person_tracks, person_keypoints)

        if self.tracking and self.target_id is not None:
            for track in self.person_tracks:
                if track.track_id == self.target_id and track.is_confirmed() and track.time_since_update <= 1:
                    center_x, center_y = self.get_center(track.to_ltrb())
                    dx1 = 280 - center_x if center_x < 280 else 0
                    dx2 = center_x - 360 if center_x > 360 else 0
                    dy1 = 200 - center_y if center_y < 200 else 0
                    dy2 = center_y - 280 if center_y > 280 else 0

                    if (dx1 < dy1 and dx2 <= dy1) or (dx1 < dy2 and dx2 <= dy2):
                        self.publish_data(102, 0 if center_y > 280 else 1)  # y ccw/cw
                    elif (dy1 < dx1 and dy2 <= dx1) or (dy1 < dx2 and dy2 <= dx2):
                        self.publish_data(101, 0 if center_x > 360 else 1)  # x ccw/cw

        # 바운딩박스 그리기 및 UI 업데이트
        self.draw_bounding_box("person", self.person_tracks, current_frame, danger_ids=closest_person_id)
        self.draw_bounding_box("knife", knife_tracks, current_frame)
        self.update_buttons_and_labels(self.person_tracks, closest_person_id)
        self.update_ui_frame(current_frame)

        terminate_t = timeit.default_timer()
        FPS = int(1. / (terminate_t - start_t))

    def detect_objects(self, obj_type, frame):
        model = self.models[obj_type]
        results = model(frame, stream=False)
        detections, keypoints = [], []

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 바운딩 박스 좌표
                conf = box.conf.item()  # 신뢰도
                if conf > 0.5:
                    width, height = x2 - x1, y2 - y1
                    detections.append([[x1, y1, width, height], conf, int(box.cls.item())])

                if obj_type == "person" and hasattr(result, "keypoints"):
                    keypoints.append(result.keypoints)
        return detections, keypoints

    def track_objects(self, obj_type, detections, frame):
        """DeepSORT로 객체 추적"""
        tracks = self.trackers[obj_type].update_tracks(detections, frame=frame)
        return tracks

    def draw_bounding_box(self, obj_type, tracks, frame, danger_ids=None):
        """추적된 객체의 바운딩박스를 그리는 함수"""
        color_map = {"person": (0, 255, 0), "knife": (0, 0, 255)}  # 사람: 초록색, 칼: 빨간색

        for track in tracks:
            if not track.is_confirmed() or track.time_since_update > 1:
                continue

            track_id = track.track_id
            x1, y1, x2, y2 = map(int, track.to_ltrb())

            # 위험인물은 빨간색, 일반 사람은 초록색
            if obj_type == "person":
                color = (0, 0, 255) if danger_ids and track_id == danger_ids else color_map[obj_type]
            else:  # 칼은 항상 빨간색
                color = color_map[obj_type]

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f"{obj_type.capitalize()} ID: {track_id}"
            cv2.putText(frame, label, (x1 + 5, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    def find_closest_hand(self, knife_center, person_tracks, keypoints):
        min_distance, closest_person_id = float('inf'), None

        for i, track in enumerate(person_tracks):
            if not track.is_confirmed() or track.time_since_update > 1:
                continue

            if i < len(keypoints) and keypoints[i].shape[1] >= 9:
                left_hand = keypoints[i].xy[0][7][:2].cpu().numpy()
                right_hand = keypoints[i].xy[0][8][:2].cpu().numpy()

                distances = [np.linalg.norm(knife_center - left_hand), np.linalg.norm(knife_center - right_hand)]
                closest_hand_distance = min(distances)

                if closest_hand_distance < min_distance:
                    min_distance = closest_hand_distance
                    closest_person_id = track.track_id

        return closest_person_id

    def get_center(self, ltrb):
        x1, y1, x2, y2 = map(int, ltrb)
        return np.array([(x1 + x2) // 2, (y1 + y2) // 2])

    def update_buttons_and_labels(self, person_tracks, dangerous_person_id):
        current_time = timeit.default_timer()

        for track in person_tracks:
            if not track.is_confirmed() or track.time_since_update > 1:
                continue

            person_id = track.track_id

            if person_id == dangerous_person_id:
                self.dangerous_person_time[person_id] = current_time
            else:
                # 칼의 위치에 따라 새로운 위험 인물이 판별된 경우, 위험 인물 타이머를 초기화
                if dangerous_person_id:
                    self.dangerous_person_time[dangerous_person_id] = current_time

            if person_id not in self.buttons:
                self.create_button_id(person_id)
                self.button_layout.addWidget(self.buttons[person_id])
            self.label_history[person_id] = current_time

        for person_id in list(self.buttons.keys()):
            if current_time - self.label_history.get(person_id, 0) > self.DISAPPEAR_THRESHOLD:
                self.buttons[person_id].setParent(None)
                del self.buttons[person_id]
                del self.label_history[person_id]
                self.publish_reset_signal()


            if person_id in self.dangerous_person_time and current_time - self.dangerous_person_time[person_id] > self.DANGER_THRESHOLD:
                del self.dangerous_person_time[person_id]
                self.publish_reset_signal()

        if dangerous_person_id:
            self.publish_emergency_signal([dangerous_person_id])

    def update_ui_frame(self, frame):
        color_image = cv2.resize(frame, (640, 360))
        h, w, ch = color_image.shape
        bytes_per_line = ch * w
        q_image = QImage(color_image.data, w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(q_image)
        self.label.setPixmap(pixmap)
        self.label.setFixedSize(w, h)

    """카메라 수동 트래킹"""
    def switch_to_manual_tracking(self):
        msg = String()
        msg.data = 'manual'
        self.tracking_mode_publisher.publish(msg)
        print("Switched to manual tracking mode")

    """카메라 자동 트래킹"""
    def switch_to_automatic_tracking(self):
        msg = String()
        msg.data = 'automatic'
        self.tracking_mode_publisher.publish(msg)
        print("Switched to automatic tracking mode")

    def create_button_id(self, track_id):
        button = QPushButton(str(track_id), self)
        button.clicked.connect(lambda: self.on_button_click(track_id))
        self.buttons[track_id] = button

    def on_button_click(self, track_id):
        self.target_id = track_id
        self.send_start_signal()
        self.tracking = True

    def publish_data(self, dxl_id, direction):
        data = Int32MultiArray(data=(dxl_id, direction))
        self.publisher.publish(data)

    """현장 통신"""
    def send_audio_comm(self):
        msg = String()
        msg.data = 'Audio communication triggered'
        self.audio_comm_publisher.publish(msg)
        print("Audio communication message sent")

    """현장 통신 중지"""
    def send_stop_audio_comm(self):
        msg = String()
        msg.data = 'Audio communication stopped'
        self.audio_comm_publisher.publish(msg)
        print("Audio communication stop message sent")

    """로봇 트래킹 시작"""
    def start_tracking(self):
        msg = String()
        msg.data = 'robot tracking start'
        self.robot_tracking_mode_publisher.publish(msg)
        print("Tracking started")
        
        self.tracking = True
        self.tracking_thread = threading.Thread(target=self.publish_depth_continuously)
        self.tracking_thread.start()

    def publish_depth_continuously(self):
        while self.tracking and not self.stop_thread and self.target_id is not None:
            for track in self.person_tracks:
                if track.track_id == self.target_id and track.is_confirmed() and track.time_since_update <= 1:
                    # 바운딩박스 중심 좌표 계산
                    center_x, center_y = self.get_center(track.to_ltrb())

                    # 중앙점에 해당하는 깊이값(depth) 계산
                    depth_value = self.depth_frame.get_distance(center_x, center_y)

                    # 거리 값을 ROS2 메시지로 발행
                    depth_msg = Float32MultiArray()
                    depth_msg.data = [float(self.target_id), float(depth_value)]
                    self.target_person_depth_publisher.publish(depth_msg)
                    print(f"Published depth: {depth_value} for target ID: {self.target_id}")

            time.sleep(0.1)  # 0.1초 간격으로 계속 발행

    """로봇 트래킹 중지"""
    def stop_tracking(self):
        self.tracking = False
        self.tracking_thread.join()  # 추적 중지 시 쓰레드 종료
        msg = String()
        msg.data = 'robot tracking stop'
        self.robot_tracking_mode_publisher.publish(msg)
        print("Tracking stopped")

        self.target_id = None

    def publish_reset_signal(self):
        msg = String()
        msg.data = 'Reset'
        self.reset_signal_publisher.publish(msg)
        print("Reset signal sent")

    def send_start_signal(self):
        msg = String()
        msg.data = 'Start'
        self.start_signal_publisher.publish(msg)
        print("Start signal sent")

    def publish_emergency_signal(self, dangerous_person_ids):
        msg = String()
        msg.data = f"Emergency: Dangerous person detected with IDs {dangerous_person_ids}"
        self.emergency_publisher.publish(msg)
        print("Emergency signal sent")

    """로봇 자동 주행"""
    def start_autonomous_driving(self):
        subprocess.Popen(['ros2', 'launch', 'gps_navigation', 'buddybot_launch.py'])
        print("GPS_navigation pkg start")

    def closeEvent(self, event):
        self.stop_thread = True
        self.pipeline.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    main_win = RealSenseObjectTracker()
    main_win.show()
    try:
        sys.exit(app.exec_())
    except SystemExit:
        print('Closing Window...')


if __name__ == '__main__':
    main()
