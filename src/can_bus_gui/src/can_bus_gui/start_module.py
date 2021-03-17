import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Slot
from python_qt_binding.QtWidgets import QWidget
#from python_qt_binding.QtGui import QLabel, QTreeWidget, QTreeWidgetItem, QVBoxLayout, QCheckBox, QWidget, QToolBar, QLineEdit, QPushButton

class StartPlugin(Plugin):

    def __init__(self, context):
        super(StartPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('StartPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
	#self._widget.setGeometry(700,200,600,500)
        #self._widget.setStyleSheet("background: rgb(250,250,250);")
	uifile = os.path.join(rospkg.RosPack().get_path('can_bus_gui'), 'resource', 'canbusmain.ui')
	loadUi(uifile, self._widget)
	self._widget.setObjectName('canbusmain')

        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

	#adding functionality
	self._widget.btn_launch.clicked.connect(self.on_launch)

	# Add widget to the user interface
        context.add_widget(self._widget)

    @Slot()
    def on_launch(self):
	if self._widget.rb_full.isChecked():
	     self._widget.lbl_load_status.setText("loading full mode...")
	     self._widget.btn_launch.setEnabled(False)
	     self._widget.rb_pedals.setEnabled(False)
	     self._widget.rb_full.setEnabled(False)
	     self._widget.rb_steering.setEnabled(False)
	elif self._widget.rb_pedals.isChecked():
	     self._widget.lbl_load_status.setText("loading pedals mode...")
	     self._widget.btn_launch.setEnabled(False)
	     self._widget.rb_full.setEnabled(False)
	     self._widget.rb_pedals.setEnabled(False)
	     self._widget.rb_steering.setEnabled(False)
	elif self._widget.rb_steering.isChecked():
	     self._widget.lbl_load_status.setText("loading steering mode...")
	     self._widget.btn_launch.setEnabled(False)
	     self._widget.rb_full.setEnabled(False)
	     self._widget.rb_steering.setEnabled(False)
             self._widget.rb_pedals.setEnabled(False)
	else:
	     self._widget.lbl_load_status.setText("select a mode.")
	print("launch")

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

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
