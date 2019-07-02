#!/usr/bin/python3
# -*- coding: utf-8 -*-


class Value:
    def __init__(self, name, unit=''):
        self._val = 'null'
        self.name = name
        self.unit = unit

    def __str__(self):
        return str(self._val)

    def set(self, val):
        self._val = val

    def get(self):
        return self._val


class ValueTable:
    def __init__(self):
        self.table = {
            'target_gear': Value('gear'),
            'target_speed_mps': Value('speed', 'm/s'),
            'target_steer_rad': Value('steer', 'rad'),
            'target_brake': Value('brake', ' / 200'),
            'input_mode': Value('mode'),
            'input_Estop': Value('Estop'),
            'input_gear': Value('gear'),
            'input_speed_mps': Value('speed', 'm/s'),
            'input_speed_kph': Value('speed', 'k/h'),
            'input_steer_rad': Value('steer', 'rad'),
            'input_steer_degree': Value('steer', 'deg'),
            'input_brake': Value('brake', '/ 200'),
            'feedback_mode': Value('mode'),
            'feedback_Estop': Value('Estop'),
            'feedback_gear': Value('gear'),
            'feedback_speed_mps': Value('speed', 'm/s'),
            'feedback_speed_kph': Value('speed', 'k/h'),
            'feedback_steer_rad': Value('steer', 'rad'),
            'feedback_steer_degree': Value('steer', 'deg'),
            'feedback_brake': Value('brake', '/ 200'),
            'feedback_encoder': Value('encoder', 'tick'),
        }

    def get(self, key):
        return self.table[key]

import rospy
from erp42_ros.msg import ERP42_feedback
from erp42_ros.msg import ERP42_input
from erp42_ros.msg import target


class RosFeedbackSub:
    def __init__(self,value_table):
        rospy.Subscriber("feedback", ERP42_feedback, self.callback)
        self.mode_value = value_table.get('feedback_mode')
        self.Estop_value = value_table.get('feedback_Estop')
        self.gear_value = value_table.get('feedback_gear')
        self.speed_mps_value = value_table.get('feedback_speed_mps')
        self.speed_kph_value = value_table.get('feedback_speed_kph')
        self.steer_rad_value = value_table.get('feedback_steer_rad')
        self.steer_degree_value = value_table.get('feedback_steer_degree')
        self.brake_value = value_table.get('feedback_brake')
        self.encoder_value = value_table.get('feedback_encoder')

    def callback(self, msg):
        self.mode_value.set(msg.mode)
        self.Estop_value.set(msg.Estop)
        self.gear_value.set(msg.gear)
        self.speed_mps_value.set(msg.speed_mps)
        self.speed_kph_value.set(msg.speed_kph)
        self.steer_rad_value.set(msg.steer_rad)
        self.steer_degree_value.set(msg.steer_degree)
        self.brake_value.set(msg.brake)
        self.encoder_value.set(msg.encoder)

class RosInputSub:
    def __init__(self,value_table):
        rospy.Subscriber("ctrl_input", ERP42_input, self.callback)
        self.mode_value = value_table.get('input_mode')
        self.Estop_value = value_table.get('input_Estop')
        self.gear_value = value_table.get('input_gear')
        self.speed_mps_value = value_table.get('input_speed_mps')
        self.speed_kph_value = value_table.get('input_speed_kph')
        self.steer_rad_value = value_table.get('input_steer_rad')
        self.steer_degree_value = value_table.get('input_steer_degree')
        self.brake_value = value_table.get('input_brake')

    def callback(self, msg):
        self.mode_value.set(msg.mode)
        self.Estop_value.set(msg.Estop)
        self.gear_value.set(msg.gear)
        self.speed_mps_value.set(msg.speed_mps)
        self.speed_kph_value.set(msg.speed_kph)
        self.steer_rad_value.set(msg.steer_rad)
        self.steer_degree_value.set(msg.steer_degree)
        self.brake_value.set(msg.brake)


class RosTargetSub:

    def __init__(self,value_table):
        rospy.Subscriber("target_input", target, self.callback)
        self.gear_value = value_table.get('target_gear')
        self.speed_mps_value = value_table.get('target_speed_mps')
        self.steer_rad_value = value_table.get('target_steer_rad')
        self.brake_value = value_table.get('target_brake')
        
    def callback(self, msg):    
        self.gear_value.set(msg.gear)
        self.speed_mps_value.set(msg.speed_mps)
        self.steer_rad_value.set(msg.steer_rad)
        self.brake_value.set(msg.brake)



from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit
from PyQt5.QtCore import QTimer

class ValueWidget(QWidget):
    def __init__(self, value):
        super().__init__()
        self.name = value.name
        self.value = value
        self.unit = value.unit
        self.title = value.name
        self.left = 0
        self.top = 0
        self.width = 200
        self.height = 50
        self.initUI()

    def initUI(self):
        self.setWindowTitle(self.title)
        #self.setGeometry(self.left, self.top, self.width, self.height)
        self.color = [255, 255, 255]

        self.textbox = QLineEdit(self)
        self.textbox.move(3, 3)
        self.textbox.resize(280, 40)
        self.textbox.setReadOnly(True)
        self.textbox.setText(str(self.value))

        self.label_name = QLabel(self)
        self.label_name.setText(self.name)

        self.label_unit = QLabel(self)
        self.label_unit.setText(self.unit)

        self.layout = QHBoxLayout(self)
        self.layout.addWidget(self.label_name)
        self.layout.addWidget(self.textbox)
        self.layout.addWidget(self.label_unit)

        self.setLayout(self.layout)
        self.show()

    def set_bg_color(self, r, g, b):
        style = 'QLineEdit{{background-color: rgb({}, {}, {})}}'.format(
            str(r), str(g), str(b))
        self.textbox.setStyleSheet(style)
        
    def update(self):
        self.textbox.setText(str(self.value))
        

class GearWidget(ValueWidget):
    def __init__(self,value):
        super().__init__(value)

    def update(self):
        if self.value.get() is 0:
            self.set_bg_color(0,255,0)
            self.textbox.setText('Drive')
        elif self.value.get() is 1:
            self.set_bg_color(255,255,0)
            self.textbox.setText('Neutral')
        elif self.value.get() is 2:
            self.set_bg_color(255,0,0)
            self.textbox.setText('Reverse')
        
        
class ModeWidget(ValueWidget):
    def __init__(self,value):
        super().__init__(value)
        self.set_bg_color(255,0,0)
        self.textbox.setText('NULL')

    def update(self):
        if self.value.get() is 0:
            self.set_bg_color(255,255,0)
            self.textbox.setText('MANUAL')
        elif self.value.get() is 1:
            self.set_bg_color(0,255,0)
            self.textbox.setText('AUTO')
        
class MonitorWidget(QWidget):
    def __init__(self,value_table):
        super().__init__()
        self.title = 'Monitor'
        self.left = 0
        self.top = 0
        self.width = 200
        self.height = 500
        self.widgets=[]
        self.initUI(value_table)


    def initUI(self, value_table):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.color = [255, 255, 255]

        # make target widgets
        self.target_gear_widget = GearWidget( value_table.get('target_gear'))
        self.target_speed_mps_widget = ValueWidget(value_table.get('target_speed_mps'))
        self.target_steer_rad_widget = ValueWidget(value_table.get('target_steer_rad'))
        self.target_brake_widget = ValueWidget(value_table.get('target_brake'))
        self.target_layout = QVBoxLayout()
        self.target_layout.addWidget(self.target_gear_widget)
        self.target_layout.addWidget(self.target_speed_mps_widget)
        self.target_layout.addWidget(self.target_steer_rad_widget)
        self.target_layout.addWidget(self.target_brake_widget)
        self.widgets.append(self.target_gear_widget)
        self.widgets.append(self.target_speed_mps_widget)
        self.widgets.append(self.target_steer_rad_widget)
        self.widgets.append(self.target_brake_widget)
        

        # make input widgets
        self.input_mode_widget = ModeWidget(value_table.get('input_mode'))
        self.input_Estop_widget = ValueWidget(value_table.get('input_Estop'))
        self.input_gear_widget = GearWidget(value_table.get('input_gear'))
        self.input_speed_mps_widget = ValueWidget(value_table.get('input_speed_mps'))
        self.input_speed_kph_widget = ValueWidget(value_table.get('input_speed_kph'))
        self.input_steer_rad_widget = ValueWidget(value_table.get('input_steer_rad'))
        self.input_steer_degree_widget = ValueWidget(value_table.get('input_steer_degree'))
        self.input_brake_widget = ValueWidget(value_table.get('input_brake'))
        self.input_layout = QVBoxLayout()
        self.input_layout.addWidget(self.input_mode_widget)
        self.input_layout.addWidget(self.input_Estop_widget)
        self.input_layout.addWidget(self.input_gear_widget)
        self.input_layout.addWidget(self.input_speed_mps_widget)
        self.input_layout.addWidget(self.input_speed_kph_widget)
        self.input_layout.addWidget(self.input_steer_rad_widget)
        self.input_layout.addWidget(self.input_steer_degree_widget)
        self.input_layout.addWidget(self.input_brake_widget)
        self.widgets.append(self.input_mode_widget)
        self.widgets.append(self.input_Estop_widget)
        self.widgets.append(self.input_gear_widget)
        self.widgets.append(self.input_speed_mps_widget)
        self.widgets.append(self.input_speed_kph_widget)
        self.widgets.append(self.input_steer_rad_widget)
        self.widgets.append(self.input_steer_degree_widget)
        self.widgets.append(self.input_brake_widget)
        
        # make feedback widgets
        self.feedback_mode_widget = ModeWidget(value_table.get('feedback_mode'))
        self.feedback_Estop_widget = ValueWidget(value_table.get('feedback_Estop'))
        self.feedback_gear_widget = GearWidget(value_table.get('feedback_gear'))
        self.feedback_speed_mps_widget = ValueWidget(value_table.get('feedback_speed_mps'))
        self.feedback_speed_kph_widget = ValueWidget(value_table.get('feedback_speed_kph'))
        self.feedback_steer_rad_widget = ValueWidget(value_table.get('feedback_steer_rad'))
        self.feedback_steer_degree_widget = ValueWidget(value_table.get('feedback_steer_degree'))
        self.feedback_brake_widget = ValueWidget(value_table.get('feedback_brake'))
        self.feedback_encoder_widget = ValueWidget(value_table.get('feedback_encoder'))
        self.feedback_layout = QVBoxLayout()
        self.feedback_layout.addWidget(self.feedback_mode_widget)
        self.feedback_layout.addWidget(self.feedback_Estop_widget)
        self.feedback_layout.addWidget(self.feedback_gear_widget)
        self.feedback_layout.addWidget(self.feedback_speed_mps_widget)
        self.feedback_layout.addWidget(self.feedback_speed_kph_widget)
        self.feedback_layout.addWidget(self.feedback_steer_rad_widget)
        self.feedback_layout.addWidget(self.feedback_steer_degree_widget)
        self.feedback_layout.addWidget(self.feedback_brake_widget)
        self.feedback_layout.addWidget(self.feedback_encoder_widget)
        self.widgets.append(self.feedback_mode_widget)
        self.widgets.append(self.feedback_Estop_widget)
        self.widgets.append(self.feedback_gear_widget)
        self.widgets.append(self.feedback_speed_mps_widget)
        self.widgets.append(self.feedback_speed_kph_widget)
        self.widgets.append(self.feedback_steer_rad_widget)
        self.widgets.append(self.feedback_steer_degree_widget)
        self.widgets.append(self.feedback_brake_widget)
        self.widgets.append(self.feedback_encoder_widget)

        # Layouts! Assemble!
        self.layout = QHBoxLayout()
        self.layout.addLayout(self.target_layout)
        self.layout.addLayout(self.input_layout)
        self.layout.addLayout(self.feedback_layout)

        # set top level layout
        self.setLayout(self.layout)

        # make timer for update
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(20)

        # show
        self.show()

    def update(self):
        for w in self.widgets:
            w.update()

import sys
from PyQt5.QtWidgets import QApplication

if __name__ == '__main__':
    app = QApplication(sys.argv)
    # ex = MainWidget()
    rospy.init_node('Monitor', anonymous=True)
    vt = ValueTable()

    sub_feedback = RosFeedbackSub(vt)
    sub_target = RosTargetSub(vt)
    sub_input = RosInputSub(vt)
    ex = MonitorWidget(vt)
    
    sys.exit(app.exec_())
