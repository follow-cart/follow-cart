import sys

from PySide2.QtCore import QSize
from PySide2.QtWidgets import QApplication, QMainWindow, QDial, QGridLayout, QLabel, QWidget
from PySide2 import QtCore
import threading
import time

class DialGui:
    def __init__(self):
        self.app = None
        self.impl_ = None
        thread = threading.Thread(target=self.run)
        thread.start()

    def run(self):
        self.app = QApplication(sys.argv)
        self.impl_ = DialGui.Impl()
        self.impl_.show()
        sys.exit(self.app.exec_())

    def linear_velocity(self):
        return self.impl_.linear_velocity()

    def angular_velocity(self):
        return self.impl_.angular_velocity()

    class Impl(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("Dial GUI")
            self.layout = QGridLayout()

            self.dial1 = QDial()
            self.dial1.setMinimum(-7)
            self.dial1.setMaximum(7)
            self.dial1.setNotchesVisible(True)
            self.dial1.valueChanged.connect(self.dial1_cb)

            self.label10 = QLabel()
            self.label10.setText('Linear')
            self.label10.setAlignment(QtCore.Qt.AlignCenter)

            self.label11 = QLabel()
            self.label11.setText(str(self.dial1.value()))
            self.label11.setAlignment(QtCore.Qt.AlignCenter)

            self.layout.addWidget(self.label10, 0, 0)
            self.layout.addWidget(self.dial1, 1, 0)
            self.layout.addWidget(self.label11, 2, 0)

            self.dial2 = QDial()
            self.dial2.setMinimum(-45)
            self.dial2.setMaximum(45)
            self.dial2.setNotchesVisible(True)
            self.dial2.valueChanged.connect(self.dial2_cb)

            self.label20 = QLabel()
            self.label20.setText('Angular')
            self.label20.setAlignment(QtCore.Qt.AlignCenter)

            self.label21 = QLabel()
            self.label21.setText(str(self.dial2.value()))
            self.label21.setAlignment(QtCore.Qt.AlignCenter)

            self.layout.addWidget(self.label20, 0, 1)
            self.layout.addWidget(self.dial2, 1, 1)
            self.layout.addWidget(self.label21, 2, 1)

            widget = QWidget()
            widget.setLayout(self.layout)
            self.setCentralWidget(widget)

        def dial1_cb(self):
            self.label11.setText(str(self.dial1.value()))

        def dial2_cb(self):
            self.label21.setText(str(self.dial2.value()))

        def linear_velocity(self) -> float:
            return self.dial1.value()

        def angular_velocity(self) -> float:
            return (-1) * self.dial2.value()






