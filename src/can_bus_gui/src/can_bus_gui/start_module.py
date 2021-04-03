import os
import io
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Slot
from python_qt_binding.QtWidgets import QWidget

import roslaunch
import shlex
import subprocess
import sys

class StartPlugin(Plugin):

    def __init__(self, context):

        # create plugin
        super(StartPlugin, self).__init__(context)
        self.setObjectName('StartPlugin')

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
        #self._widget.setStyleSheet("background: rgb(250,250,250);")
	uifile = os.path.join(rospkg.RosPack().get_path('can_bus_gui'), 'resource', 'canbusmain.ui')
	loadUi(uifile, self._widget)
	self._widget.setObjectName('canbusmain')

        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # setting default visibility
        self._widget.lbl_load_status.setText("")
        self._widget.progressBar_load.hide()
        self._widget.btn_cancel.hide()

        # adding functionality
	self._widget.btn_launch.clicked.connect(self.on_launch)
        self._widget.btn_cancel.clicked.connect(self.on_cancel)

        # ros launch variables
        self.cli_args = []
        self.can_bus = "can_bus"
        self.can_launch = "can_bus.launch"
        self.cli_args.append(self.can_bus)
        self.cli_args.append(self.can_launch)
        self.is_launching = False

	# Add widget to the user interface
        context.add_widget(self._widget)

    def start_roslaunch(self, arg):
        # setup roslaunch
        self.cli_args.append("mode:="+arg)
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.roslaunch_args = self.cli_args[2:]
        self.roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(self.cli_args)
        self.parent = roslaunch.parent.ROSLaunchParent(self.uuid, self.roslaunch_file)
        # change qt widget visibility
        self.is_launching = True
        self._widget.progressBar_load.show()
        self._widget.btn_cancel.show()
        # run roslaunch
        self.parent.start()
        # self.temp_out = io.BytesIO()
        # try:
        #     sys.stdout = self.temp_out
        #     self.parent.start()
        #     self.temp_string = str(self.temp_out.getvalue())
        #     while self.temp_string != "CAN Diagnostics - Starting Diagnostics...":
        #          self.lbl_load_status.setText(self.temp_string)
        #          self.temp_string = str(self.temp_out.getvalue())
        # except:
        #     print("Could no redirect")
        # finally:
        #     sys.stdout = sys.__stdout__
        #     print("back to normal standard output")

    @Slot()
    def on_launch(self):
	if self._widget.rb_full.isChecked():
	     self._widget.lbl_load_status.setText("loading full mode...")
	     self._widget.btn_launch.setEnabled(False)
	     self._widget.rb_pedals.setEnabled(False)
	     self._widget.rb_full.setEnabled(False)
	     self._widget.rb_steering.setEnabled(False)
             self.start_roslaunch("can_complete")
	elif self._widget.rb_pedals.isChecked():
	     self._widget.lbl_load_status.setText("loading pedals mode...")
	     self._widget.btn_launch.setEnabled(False)
	     self._widget.rb_full.setEnabled(False)
	     self._widget.rb_pedals.setEnabled(False)
	     self._widget.rb_steering.setEnabled(False)
             self.start_roslaunch("can_pedals")
	elif self._widget.rb_steering.isChecked():
	     self._widget.lbl_load_status.setText("loading steering mode...")
	     self._widget.btn_launch.setEnabled(False)
	     self._widget.rb_full.setEnabled(False)
	     self._widget.rb_steering.setEnabled(False)
             self._widget.rb_pedals.setEnabled(False)
             self.start_roslaunch("can_steering")
	else:
	     self._widget.lbl_load_status.setText("select a mode.")
             self._widget.lbl_load_status.adjustSize()

    @Slot()
    def on_cancel(self):
        self.shutdown_plugin()

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        if self.is_launching == True:
             self.parent.shutdown()

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
