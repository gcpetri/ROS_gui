#!/usr/bin/env python
import os
from cStringIO import StringIO
import rospy
import rospkg
import time
import sys

from std_msgs.msg import String, Int64

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Slot
from python_qt_binding.QtWidgets import QWidget

class ControllerStatusPlugin(Plugin):

    def __init__(self, context):

        # create plugin
        super(ControllerStatusPlugin, self).__init__(context)
        self.setObjectName('ControllerStatusPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())

        # Create QWidget
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

        # create ros subscribers
        rospy.Subscriber('/can_bus/can_control_type', String, callback=self.control_type)
        rospy.Subscriber('/can_bus/vehicle_control_mode', Int64, callback=self.control_mode)


     # callback function to display the control type
    def control_type(self, data):
        self._widget.lbl_control_type.setText(data.data)

    # callback function to display the control mode
    def control_mode(self, data):
        self._widget.lbl_control_mode.setText(str(data.data))

   # def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        # pass

    #def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        # pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
