#imu_realsense_tracking_pyqt (1)

import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QHBoxLayout, QWidget, QPushButton, QVBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt
import pyrealsense2 as rs
from ultralytics import YOLO
from tracker import Tracker  # Assuming you have the Tracker class implementation
import timeit
import rclpy
from std_msgs.msg import Int32MultiArray, String


class RealSenseObjectTracker(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.setGeometry(100, 100, 1280, 720)
        self.setWindowTitle("Police")

        self.main_layout = QVBoxLayout()
        self.label = QLabel(self)
        self.main_layout.addWidget(self.label)

        self.global_center_x = 0
        self.global_center_y = 0
        self.target_id = None
        self.pipeline, self.align = self.initialize_realsense()
        self.tracker = self.initialize_tracker()
        self.yolo_model = self.initialize_yolo()

        rclpy.init()
        self.node = rclpy.create_node('to_dynamixel_node')
        self.publisher = self.node.create_publisher(Int32MultiArray, 'to_dynamixel', 10)
        self.start_signal_publisher = self.node.create_publisher(String, 'start_signal', 10)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.main)
        self.timer.start(1)  # Reduced from 100 to 30 milliseconds

        self.last_detection_time = {}
        self.buttons = {}
        self.button_layout = QHBoxLayout()
        self.main_layout.addLayout(self.button_layout)
        central_widget = QWidget(self)
        central_widget.setLayout(self.main_layout)
        self.setCentralWidget(central_widget)
        self.DISAPPEAR_THRESHOLD = 3

    def initialize_realsense(self):
        pipeline = rs.pipeline()
        config = rs.config()
        # config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        pipeline.start(config)
        align = rs.align(rs.stream.color)
        return pipeline, align

    # def scale_depth_image(self, depth_image):
    #     return cv2.convertScaleAbs(depth_image, alpha=0.025)

    def initialize_tracker(self):
        return Tracker()

    def initialize_yolo(self):
        return YOLO("./yolov8n.pt")

    def main(self):
        start_t = timeit.default_timer()
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        # aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # if not aligned_depth_frame or not color_frame:
        #     return

        # depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # depth_image = cv2.resize(depth_image, (1280, 720))
        color_image = cv2.resize(color_image, (1280, 720))
        # depth_image_scaled = self.scale_depth_image(depth_image)

        results = self.yolo_model(color_image, stream=True)

        detections = []
        for result in results:
            for box in result.boxes:
                confidence = box.conf
                if confidence > 0.5:
                    detections.append(box.xyxy.tolist()[0])

        self.tracker.update(color_image, detections)

        current_ids = []

        for track in self.tracker.tracks:
            bbox = track.bbox
            x1, y1, x2, y2 = map(int, bbox)
            track_id = track.track_id
            current_ids.append(track_id)
            color = (0, 0, 255) if track_id == self.target_id else (0, 0, 0)
            cv2.rectangle(color_image, (x1, y1), (x2, y2), color, 2)
            cv2.putText(color_image, f"ID: {track_id}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            self.last_detection_time[track_id] = timeit.default_timer()

            if track_id not in self.buttons:
                self.create_button(track_id)

            if track_id == self.target_id:
                self.global_center_x = (x1 + x2) // 2
                self.global_center_y = (y1 + y2) // 2
                print(self.global_center_x, self.global_center_y)
                
                dx1 = dx2 = dy1 = dy2 = 0
                if self.global_center_x < 600:
                    dx1 = 600 - self.global_center_x
                elif self.global_center_x > 680:
                    dx2 = self.global_center_x - 680

                if self.global_center_y < 320:
                    dy1 = 320 - self.global_center_y
                elif self.global_center_y > 400:
                    dy2 = self.global_center_y - 400

                if (dx1 < dy1 and dx2 <= dy1) or (dx1 < dy2 and dx2 <= dy2):
                    if self.global_center_y > 400:
                        self.publish_data(102, 0)  # y ccw
                    elif self.global_center_y < 320:
                        self.publish_data(102, 1)  # y cc
                elif (dy1 < dx1 and dy2 <= dx1) or (dy1 < dx2 and dy2 <= dx2):
                    if self.global_center_x > 680:
                        self.publish_data(101, 0)  # x ccw
                    elif self.global_center_x < 600:
                        self.publish_data(101, 1)  # x cw

        for track_id in list(self.buttons.keys()):
            if track_id not in current_ids:
                if timeit.default_timer() - self.last_detection_time.get(track_id, 0) > self.DISAPPEAR_THRESHOLD:
                    button = self.buttons.pop(track_id)
                    button.deleteLater()

        self.button_layout.setAlignment(Qt.AlignLeft)
        self.button_layout.setSpacing(5)
        for button in sorted(self.buttons.values(), key=lambda b: int(b.text())):
            self.button_layout.addWidget(button)

        terminate_t = timeit.default_timer()
        FPS = int(1./(terminate_t - start_t))

        color_image = cv2.resize(color_image, (640, 360))

        h, w, ch = color_image.shape
        bytes_per_line = ch * w
        q_image = QImage(color_image.data, w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped()

        pixmap = QPixmap.fromImage(q_image)
        self.label.setPixmap(pixmap)
        self.label.setFixedSize(w, h)

    def create_button(self, track_id):
        button = QPushButton(str(track_id), self)
        button.clicked.connect(self.on_button_click)
        self.buttons[track_id] = button

    def send_start_signal(self):
        msg = String()
        msg.data = 'Start'
        self.start_signal_publisher.publish(msg)
        print("Start signal sent")


    def on_button_click(self):
        sender = self.sender()
        if sender:
            self.target_id = int(sender.text())
            self.send_start_signal()

    def closeEvent(self, event):
        self.pipeline.stop()
        event.accept()

    def publish_data(self, dxl_id, direction):  # Added method for publishing data
        data = Int32MultiArray(data=(dxl_id, direction))
        self.publisher.publish(data)


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
