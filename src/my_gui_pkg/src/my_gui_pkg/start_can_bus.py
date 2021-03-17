# =============== Information ====================
# Project:      Texas A&M Autodrive Challenge
# Language:     Python 3.6
# ROS Package: 
# Repository:
# File Name:    start_can_bus.py
# Version:
# Description:  Qt GUI to start-up can_bus
# Date:         March 11, 2021
# Author:       Gregory Petri
# Contact:      gcpetri@tamu.edu

# =============== Imports =========================
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import * 
from PyQt5.QtGui import *
import sys

class Ui_StartWindow(QMainWindow):

	def __init__(self):
		super(Ui_StartWindow, self).__init__()
		self.setGeometry(700,200,600,500)
		self.setWindowTitle("can_bus")
		self.initUI()

	def initUI(self):

		# label top
		self.lbl_desc = QtWidgets.QLabel(self)
		self.lbl_desc.setText("Texas A&M University 12th Unmanned Team")
		self.lbl_desc.setFont(QFont('Consolas', 10))
		self.lbl_desc.setStyleSheet("color: rgb(100,100,100);")
		self.lbl_desc.move(50,50)
		self.update(self.lbl_desc)

		# label can bus
		self.lbl_canbus = QtWidgets.QLabel(self)
		self.lbl_canbus.setText("Start can_bus")
		self.lbl_canbus.setFont(QFont('Consolas', 20))
		self.lbl_canbus.setStyleSheet("color: rgb(84,187,203); font-weight: bold;")
		self.update(self.lbl_canbus)
		self.lbl_canbus.move(50,70)

		# mode
		self.lbl_choose = QtWidgets.QLabel(self)
		self.lbl_choose.setText("Select Start-up Mode")
		self.lbl_choose.setFont(QFont('Consolas', 12))
		self.update(self.lbl_choose)
		self.lbl_choose.move(50,200)
		# mode redio buttons
		self.rb_complete = QtWidgets.QRadioButton(self)
		self.rb_complete.setText("Complete")
		self.update(self.rb_complete)
		self.rb_complete.move(50, 250)
		self.rb_steering = QtWidgets.QRadioButton(self)
		self.rb_steering.setText("Steering")
		self.update(self.rb_steering)
		self.rb_steering.move(200, 250)
		self.rb_pedals = QtWidgets.QRadioButton(self)
		self.rb_pedals.setText("Pedals")
		self.update(self.rb_pedals)
		self.rb_pedals.move(350, 250)

		# launch button
		self.btn_launch = QtWidgets.QPushButton(self)
		self.btn_launch.setText("Launch")
		self.btn_launch.setGeometry(50, 320, 120, 50)
		self.btn_launch.clicked.connect(self.launch)

		# launch label
		self.lbl_launch = QtWidgets.QLabel(self)
		self.lbl_launch.setFont(QFont('Consolas', 8))
		self.lbl_launch.setStyleSheet("color: rgb(100,100,100);")
		self.update(self.lbl_launch)
		self.lbl_launch.move(50,400)

		# progress bar
		self.progress_load = QtWidgets.QProgressBar(self)
		self.progress_load.setGeometry(50,420,520,20)


	def launch(self):
		if self.rb_complete.isChecked():
			self.lbl_launch.setText("loading complete mode...")
			self.lbl_launch.setStyleSheet("color: rgb(100,100,100);")
			self.update(self.lbl_launch)
			self.btn_launch.setEnabled(False)
			self.rb_complete.setEnabled(False)
			self.rb_steering.setEnabled(False)
			self.rb_pedals.setEnabled(False)
		elif self.rb_steering.isChecked():
			self.lbl_launch.setText("loading steering mode...")
			self.lbl_launch.setStyleSheet("color: rgb(100,100,100);")
			self.update(self.lbl_launch)
			self.btn_launch.setEnabled(False)
			self.rb_complete.setEnabled(False)
			self.rb_steering.setEnabled(False)
			self.rb_pedals.setEnabled(False)
		elif self.rb_pedals.isChecked():
			self.lbl_launch.setText("loading pedals mode...")
			self.lbl_launch.setStyleSheet("color: rgb(100,100,100);")
			self.update(self.lbl_launch)
			self.btn_launch.setEnabled(False)
			self.rb_complete.setEnabled(False)
			self.rb_steering.setEnabled(False)
			self.rb_pedals.setEnabled(False)
		else:
			self.lbl_launch.setText("please select start-up mode.")
			self.lbl_launch.setStyleSheet("color: rgb(255,102,102);")
			self.update(self.lbl_launch)

	def update(self, arg):
		arg.adjustSize()


def window():
	app = QApplication(sys.argv)
	win = Ui_StartWindow()

	win.show()
	sys.exit(app.exec_())

###### run: pyuic5 -x <ui file name> -o <python file name>
if __name__ == "__main__":
	window()

