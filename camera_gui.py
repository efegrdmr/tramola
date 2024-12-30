import sys
import rospy
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

        self.bridge = CvBridge()
        rospy.init_node('camera_gui', anonymous=True)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.update_image)

    def init_ui(self):
        self.setWindowTitle('Camera Feed')
        self.layout = QVBoxLayout()
        self.image_label = QLabel()
        self.layout.addWidget(self.image_label)
        self.setLayout(self.layout)

    def update_image(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        self.image_label.setPixmap(QPixmap.fromImage(q_image))

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = CameraGUI()
    gui.show()
    sys.exit(app.exec_())

