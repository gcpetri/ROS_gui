#!/usr/bin/env python

# Python libraries
import os
import sys
import time

# ROS libraries
import rospy
import rospkg
import roslaunch
from std_msgs.msg import String, Int32, Int64, Float32

# PyQt5 libraries
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding import QtCore
from python_qt_binding.QtCore import Qt, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QWidget

# Stdout redirect helper class
class WidgetStream(QtCore.QObject):
    textWritten = QtCore.pyqtSignal(str)
    def write(self, text):
        self.textWritten.emit(str(text))

# dashboard rqt class
class Canbus_Dashboard_Plugin(Plugin):

    def __init__(self, context):

        # globals
        self.FILEPATH = os.path.join(rospkg.RosPack().get_path('can_bus_gui'), 'logs')
        self.LOAD_TIME_FACTOR = 1
        self.IS_ON = False

        # create plugin
        super(Canbus_Dashboard_Plugin, self).__init__(context)
        self.setObjectName('Canbus_Dashboard_Plugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())

        # Create Canbus Dashboard QWidget
        self._widget = QWidget()
	self._widget.setGeometry(350,200,300,500)
	uifile = os.path.join(rospkg.RosPack().get_path('can_bus_gui'), 'resource', 'can_dashboard.ui')
	loadUi(uifile, self._widget)
	self._widget.setObjectName('can_dashboard')

        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

	# Add widget to the user interface
        context.add_widget(self._widget)

        # get the starting time
        self.START_TIME = time.time()

        # set the custom logging output file name
        if not os.path.exists(self.FILEPATH):
            os.mkdir(self.FILEPATH)
        self.LOGFILENAME = "0_custom_log_CAN_BUS.txt"
        self.CONSOLEFILENAME = "0_console_CAN_BUS.txt"
        num = 0
        while os.path.exists(self.FILEPATH+self.LOGFILENAME) or os.path.exists(self.FILEPATH+self.CONSOLEFILENAME):
             num += 1
             self.LOGFILENAME = str(num)+self.LOGFILENAME[1:]
        self.LOGFILENAME = str(num)+self.LOGFILENAME[1:]
        self.CONSOLEFILENAME = str(num)+self.CONSOLEFILENAME[1:]
        self._widget.lbl_log_path.setText("log file: "+self.LOGFILENAME)

        # set up pictures
        self._widget.btn_save_console.setIcon(QIcon(os.path.join(rospkg.RosPack().get_path('can_bus_gui'), 'resource/images', 'save_icon.png')))

        # set up buttons
        self._widget.btn_logit.clicked.connect(self.appendToLog)
        self._widget.btn_clear_log.clicked.connect(self.clearLog)
        self._widget.btn_end.clicked.connect(self.toggle_start_widget)
        self._widget.btn_save_console.clicked.connect(self.save_console)

        # subscriber paths
        # CAN Controller
        self.can_control_type = '/can_bus/can_control_type'
        self.vehicle_control_mode = '/can_bus/vehicle_control_mode'
        self.can_init = '/can_bus/can_init'
        # CAN Diagnostics
        self.StrWhAng = '/can_bus/can_diagnostics/StrWhAng'
        self.StrWhAngV = '/can_bus/can_diagnostics/StrWhAngV'
        self.StrWhGrd = '/can_bus/can_diagnostics/StrWhAngGrd'
        self.StrWhGrdV = '/can_bus/can_diagnostics/StrWhAngGrdV'
        self.StrWhAngSenCalStat = '/can_bus/can_diagnostics/StrWhAngSenCalStat'
        # steering
        self.steeringAutoTopic = '/can_steering_auto'
        self.steeringManualTopic = '/can_bus/steering_manual'
        self.steeringRateTopic = '/can_bus/steering_rate'
        # break
        self.breakManuelTopic = '/can_bus/brake_manual'
        self.breakAutoTopic = '/can_brake_auto'
        # throttle
        self.throttleManualTopic = '/can_bus/throttle_manual'
        self.throttleAutoTopic = 'can_throttle_auto'
        # transmission
        self.transmissionRequestTopic = '/can_bus/transmission_request'
        self.transmissionStateTopic = '/can_bus/transmission_state'
        # lowspeed
        self.lowBeamTopic = '/can_bus/low_beams'
        self.rightBlinkerTopic = '/can_bus/right_blinker'
        self.leftBlinkerTopic = '/can_bus/left_blinker'
        self.hazardTopic = '/can_bus/hazards'
        self.lockDoorsTopic = '/can_bus/lock_doors'

	# initialize subscribers
	self.subscriber()

        # create start widget
        self.createStartWidget()
        self._start_widget.hide()

    ##############################################
    ################# destructor #################
    ##############################################

    def __del__(self):
        sys.stdout = sys.__stdout__

    ##############################################
    ################# complete ###################
    ##############################################

    # write stdout to text browser
    def writeStdout(self, text):
        self._widget.txt_console.append(text)

    # clear the text browser
    @Slot()
    def clearLog(self):
        self._widget.txt_console.clear()

    def update_complete(self, category, value):
        self._widget.lbl_updates_category.setText(str(category))
        self._widget.lbl_updates_value.setText(str(value))
        self._widget.lbl_updates_timestamp.setText(str(time.time() - self.START_TIME))

    ##############################################
    ############## CAN controller ################
    ##############################################

    # callback function to display the control type
    def control_type(self, data):
        self._widget.lbl_control_type.setText(data.data)

    # callback function to display the control mode
    def control_mode(self, data):
        if (data.data == 0):
             self._widget.lbl_control_mode.setText("Mode:: OFF")
        elif (data.data == 1):
             self._widget.lbl_control_mode.setText("Mode:: Manual")
        elif (data.data == 2):
             self._widget.lbl_control_mode.setText("Mode:: Auto")

    # callback function to display the controller status
    def control_status(self, data):
        if (data.data == 1):
            self._widget.lbl_can_status.setText("Unhealthy")
        else:
            self._widget.lbl_can_status.setText("Good")

    ################################################
    ############### CAN diagnostics ################
    ################################################

    def wheel_angle(self, data):
        self._widget.lbl_StrWhAng.setText(str(data.data))

    def wheel_angular_velocity(self, data):
        self._widget.lbl_StrWhAngV.setText(str(data.data))

    def wheel_gradient(self, data):
        self._widget.lbl_StrWhGrd.setText(str(data.data))

    def wheel_gradient_velocity(self, data):
        self._widget.lbl_StrWhGrdV.setText(str(data.data))

    def wheel_angular_sen_cal_stat(self, data):
        self._widget.lbl_StrWhAngSenCalStat.setText(str(data.data))

    ################################################
    ################## steering ####################
    ################################################

    # callback function to display the steering angle
    def steering_value(self, data):
        if (data.data <= 515 and data.data >= -515):
            try:
                self._widget.dial_steering_angle.setValue(data.data)
	        self._widget.lcd_steering_angle.setValue(data.data)
                self.update_complete("steering angle", data.data)
            except ValueError:
                print("cannot convert steering_auto value to number")
        else:
            pass

    # callback function to display the steering rate
    def steering_rate(self, data):
        if (data.data > 0 and data.data < 500):
            self._widget.lbl_steering_rate_value.setText(str(data.data))
            self.update_complete("steering rate", data.data)
        else:
            pass

    ################################################
    ################### break ######################
    ################################################

    def break_value(self, data):
        if (data.data >= -7000 and data.data <= 0):
            self._widget.lcd_break_value.setValue(data.data)
            self.update_complete("break", data.data)
        else:
            pass

    ################################################
    ################### throttle ###################
    ################################################

    def throttle_value(self, data):
        self._widget.lcd_throttle_value.setValue(data.data)
        self.update_complete("throttle", data.data)

    ################################################
    ################## transmission ################
    ################################################

    def transmission_request(self, data):
        self._widget.lbl_request_trans_state.setText(str(data.data))
        self.update_complete("transmission request", data.data)

    def transmission_state(self, data):
        self._widget.lbl_trans_state.setText(str(data.data))
        self.update_complete("transmission state", data.data)

    ################################################
    ################### lowspeed ###################
    ################################################

    def leftBlinker(self, data):
        self._widget.lbl_left_blinker_value.setText(str(data.data))
        self.update_complete("left blinker", data.data)

    def rightBlinker(self, data):
        self._widget.lbl_right_blinker_value.setText(str(data.data))
        self.update_complete("right blinker", data.data)

    def lowBeam(self, data):
        self._widget.lbl_lowbeam_value.setText(str(data.data))
        self.update_complete("lowbeam", data.data)

    def hazards(self, data):
        self._widget.lbl_hazards_value.setText(str(data.data))
        self.update_complete("hazards", data.data)

    def lockDoors(self, data):
        self._widget.lbl_lock_doors_data.setText(str(data.data))
        if (data.data == 0): self._widget.lbl_lock_request_data.setText("1")
        elif (data.data == 1): self._widget.lbl_lock_request_data.setText("2")
        else: self._widget.lbl_lock_request_data.setText("0")
        self.update_complete("lock doors", data.data)

    ###################################################
    ############ Log Custom Runtime Events ############
    ###################################################

    def getTime(self):
        return str(time.time() - self.START_TIME)

    @Slot()
    def appendToLog(self):
        logFile = open(self.FILEPATH+'/'+self.LOGFILENAME, "a+")
        logFile.write(self._widget.le_log.text())
        logFile.write("\t"+self.getTime()+"\n")
        self._widget.le_log.clear()
        logFile.close()

    # initialize all the subscribers
    def subscriber(self):
        # CAN controller
        rospy.Subscriber(self.can_control_type, String, callback=self.control_type)
        rospy.Subscriber(self.vehicle_control_mode, Int64, callback=self.control_mode)
        rospy.Subscriber(self.can_init, Int64, callback=self.control_status)
        # CAN diagnostics
        rospy.Subscriber(self.StrWhAng, Float32, callback=self.wheel_angle)
        rospy.Subscriber(self.StrWhAngV, Float32, callback=self.wheel_angular_velocity)
        rospy.Subscriber(self.StrWhGrd, Float32, callback=self.wheel_gradient)
        rospy.Subscriber(self.StrWhGrdV, Int32, callback=self.wheel_gradient_velocity)
        rospy.Subscriber(self.StrWhAngSenCalStat, Int32, callback=self.wheel_angular_sen_cal_stat)
        # steering
        rospy.Subscriber(self.steeringAutoTopic, Int64, callback=self.steering_value)
        rospy.Subscriber(self.steeringManualTopic, Int64, callback=self.steering_value)
        rospy.Subscriber(self.steeringRateTopic, Int64, callback=self.steering_rate)
        # break
        rospy.Subscriber(self.breakManuelTopic, Int64, callback=self.break_value)
        rospy.Subscriber(self.breakAutoTopic, Int64, callback=self.break_value)
        # transmission
        rospy.Subscriber(self.transmissionRequestTopic, String, callback=self.transmission_request)
        rospy.Subscriber(self.transmissionStateTopic, String, callback=self.transmission_state)
        # lowspeed
        rospy.Subscriber(self.leftBlinkerTopic, Int64, callback=self.leftBlinker)
        rospy.Subscriber(self.rightBlinkerTopic, Int64, callback=self.rightBlinker)
        rospy.Subscriber(self.lowBeamTopic, Int64, callback=self.lowBeam)
        rospy.Subscriber(self.hazardTopic, Int64, callback=self.hazards)
        rospy.Subscriber(self.lockDoorsTopic, Int64, callback=self.lockDoors)

    ###########################################
    ############# Save Console ################
    ###########################################

    @Slot()
    def save_console(self):
        with open (self.FILEPATH+'/'+self.CONSOLEFILENAME, 'w+') as consoleFile:
             consoleFile.write(str(self._widget.txt_console.toPlainText()))
        self._widget.lbl_save_text.setText('saved file to '+self.CONSOLEFILENAME)

    # end the session
    @Slot()
    def closeWidget(self):
        sys.stdout = sys.__stdout__
        self._widget.lbl_can_status.setText("CLOSED")

    ############################################
    ################ CAN start #################
    ############################################

    def createStartWidget(self):
        # Create Start Canbus QWidget
        self._start_widget = QWidget()
        self._start_widget.setGeometry(250,200,200,400)
        uifile = os.path.join(rospkg.RosPack().get_path('can_bus_gui'), 'resource', 'can_start.ui')
        loadUi(uifile, self._start_widget)
        self._start_widget.setObjectName('Start Canbus')

        # setting default visibility
        self._start_widget.lbl_load_status.setText("")
        self._start_widget.progressBar_load.hide()

        # adding functionality
        self._start_widget.btn_launch.clicked.connect(self.on_launch)

        # ros launch variables
        self.cli_args = []
        self.cli_args.append("can_bus")
        self.cli_args.append("can_bus.launch")
        self.is_launching = False

    def start_roslaunch(self, arg):
        # setup roslaunch
        self.cli_args.append("mode:="+arg)
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.roslaunch_args = self.cli_args[2:]
        self.roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(self.cli_args)
        self.parent = roslaunch.parent.ROSLaunchParent(self.uuid, self.roslaunch_file)

        # change qt widget visibility
        self.is_launching = True
        self._start_widget.progressBar_load.show()

        # redirect standard output
        sys.stdout = WidgetStream(textWritten=self.writeStdout)

        # start can bus
        self.parent.start()

        # update progress bar
        num_loops = self.LOAD_TIME_FACTOR * 10
        for i in range(num_loops):
             self._start_widget.progressBar_load.setValue(i % num_loops)
             time.sleep(0.1)
        self._start_widget.progressBar_load.setValue(100)
        self._start_widget.progressBar_load.hide()
        self._start_widget.lbl_load_status.setText('canbus launched')
        self.is_launching = False
        self.IS_ON = True
        self._start_widget.hide()
        self._widget.btn_end.setText("END")

    @Slot()
    def on_launch(self):
        if self._start_widget.rb_full.isChecked():
             self._start_widget.lbl_load_status.setText("loading full mode...")
             self._start_widget.btn_launch.setEnabled(False)
             self._start_widget.rb_pedals.setEnabled(False)
             self._start_widget.rb_full.setEnabled(False)
             self._start_widget.rb_steering.setEnabled(False)
             self.start_roslaunch("can_complete")
        elif self._start_widget.rb_pedals.isChecked():
             self._start_widget.lbl_load_status.setText("loading pedals mode...")
             self._start_widget.btn_launch.setEnabled(False)
             self._start_widget.rb_full.setEnabled(False)
             self._start_widget.rb_pedals.setEnabled(False)
             self._start_widget.rb_steering.setEnabled(False)
             self.start_roslaunch("can_pedals")
        elif self._start_widget.rb_steering.isChecked():
             self._start_widget.lbl_load_status.setText("loading steering mode...")
             self._start_widget.btn_launch.setEnabled(False)
             self._start_widget.rb_full.setEnabled(False)
             self._start_widget.rb_steering.setEnabled(False)
             self._start_widget.rb_pedals.setEnabled(False)
             self.start_roslaunch("can_steering")
        else:
             self._start_widget.lbl_load_status.setText("select a mode.")
             self._start_widget.lbl_load_status.adjustSize()

    # show/hide the start canbus widget
    @Slot()
    def toggle_start_widget(self):
        if (self.IS_ON == False):
             self._start_widget.show()
             self._start_widget.raise_()
        else:
             self.shutdown_plugin()
             self._widget.btn_end.setText("START")

    ###########################################
    ############## Shutdown Canbus ############
    ###########################################

    def shutdown_plugin(self):
        if self.is_launching == False:
             self._widget.lbl_can_status = "CLOSING"
             if self.IS_ON:
                 self.parent.shutdown()
                 sys.stdout = sys.__stdout__
                 print("--- Goodbye! ---")
        else:
             self._widget.lbl_can_status = "CLOSE FAILED"


