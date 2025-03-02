import sys
import numpy as np
import cv2
import serial
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QPushButton, QLabel, QWidget, QMessageBox, QHBoxLayout
from PyQt5.QtGui import QPainter, QPen, QImage, QPixmap
from PyQt5.QtCore import Qt, QPoint, QTimer

# Try to open serial connection
try:
    ser = serial.Serial('COM5', baudrate=115200, timeout=1)
    print("Connected to STM32.")
except serial.SerialException as e:
    print("Failed to connect to STM32:", e)
    ser = None

CANVAS_SIZE = 280  
GRID_SIZE = 28
PIXEL_SIZE = CANVAS_SIZE // GRID_SIZE

class DrawingApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Digit Drawing with Camera Feed")
        self.setFixedSize((CANVAS_SIZE * 2) + 40, CANVAS_SIZE + 150)

        self.drawing = False
        self.last_point = QPoint()
        self.image = QImage(CANVAS_SIZE, CANVAS_SIZE, QImage.Format_Grayscale8)
        self.image.fill(Qt.white)

        self.grid_data = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)

        # Central widget and layout
        central_widget = QWidget()
        main_layout = QVBoxLayout()
        canvas_video_layout = QHBoxLayout()  # Layout for drawing and camera feed

        # Drawing canvas
        self.label = QLabel()
        self.label.setFixedSize(CANVAS_SIZE, CANVAS_SIZE)
        canvas_video_layout.addWidget(self.label)

        # Video feed
        self.video_label = QLabel()
        self.video_label.setFixedSize(CANVAS_SIZE, CANVAS_SIZE)
        canvas_video_layout.addWidget(self.video_label)

        main_layout.addLayout(canvas_video_layout)

        # Buttons
        clear_button = QPushButton("Clear")
        clear_button.clicked.connect(self.clear_canvas)
        main_layout.addWidget(clear_button)

        send_button = QPushButton("Send")
        send_button.clicked.connect(self.send_image)
        main_layout.addWidget(send_button)

        self.result_label = QLabel("Prediction: None")
        main_layout.addWidget(self.result_label)

        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        # Timer to check UART for incoming data every 100ms
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.read_uart_data)
        self.timer.start(100)

        # OpenCV camera feed
        self.cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Use camera index 0
        self.video_timer = QTimer(self)
        self.video_timer.timeout.connect(self.update_video)
        self.video_timer.start(30)  # Update video every 30ms

    def paintEvent(self, event):
        canvas_painter = QPainter(self)
        canvas_painter.drawImage(self.label.pos(), self.image)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drawing = True
            self.last_point = event.pos() - self.label.pos()

    def mouseMoveEvent(self, event):
        if self.drawing:
            painter = QPainter(self.image)
            pen = QPen(Qt.black, PIXEL_SIZE, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
            painter.setPen(pen)
            current_point = event.pos() - self.label.pos()
            painter.drawLine(self.last_point, current_point)
            self.last_point = current_point
            self.update()

            x = current_point.x() // PIXEL_SIZE
            y = current_point.y() // PIXEL_SIZE
            if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
                self.grid_data[y, x] = 255

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drawing = False

    def clear_canvas(self):
        self.image.fill(Qt.white)
        self.grid_data.fill(0)
        self.update()

    def send_image(self):
        if not ser or not ser.is_open:
            QMessageBox.critical(self, "Error", "Serial port not connected.")
            return

        normalized_data = (self.grid_data.astype(np.float32) / 0.032967425882816315) + -9
        int8_data = normalized_data.astype(np.int8)

        if int8_data.size != 784:
            QMessageBox.critical(self, "Error", f"Invalid image size: {int8_data.size}")
            return

        try:
            ser.write(int8_data.tobytes())
            print("Image data sent to STM32.")
        except serial.SerialException as e:
            QMessageBox.critical(self, "Error", f"Failed to send data: {e}")
            return

    def read_uart_data(self):
        if ser and ser.in_waiting > 0:
            try:
                output = ser.read(1)
                predicted_class = int.from_bytes(output, byteorder='big')
                self.display_result(predicted_class)
            except serial.SerialException as e:
                print(f"Error reading UART data: {e}")

    def display_result(self, predicted_class):
        self.result_label.setText(f"Task running: {predicted_class}")

    def update_video(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = cv2.resize(frame, (CANVAS_SIZE, CANVAS_SIZE))  # Resize to fit QLabel
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(qimg))

    def closeEvent(self, event):
        if self.cap.isOpened():
            self.cap.release()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = DrawingApp()
    main_window.show()
    sys.exit(app.exec_())

if ser and ser.is_open:
    ser.close()
