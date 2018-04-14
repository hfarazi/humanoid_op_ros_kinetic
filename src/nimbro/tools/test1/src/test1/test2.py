import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QPushButton, QVBoxLayout, QWidget, QSpinBox

class test3(Plugin):

    def __init__(self, context):
        super(test3, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('test3')

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        #ui_file = os.path.join(rospkg.RosPack().get_path('test1'), 'resource', 'test3.ui')
        # Extend the widget with all attributes and children from UI file
        #loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('test3Ui')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        #context.add_widget(self._widget)
	
	self._container = QWidget()
        self._layout    = QVBoxLayout()
        self._container.setLayout(self._layout)
	
	self._some_button = QPushButton('Click here')
	self._layout.addWidget(self._some_button)
	self._some_button.clicked.connect(self._button_pressed)

	self._some_spinbox = QSpinBox()
	self._layout.addWidget(self._some_spinbox)


	context.add_widget(self._container)
	#context.add_widget(self._widget)
	#context.add_widget(self._container)

    def _button_pressed(self):
	# Some button action here.
	pass

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
