#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import rospy
import math
from erp42_ros.msg import target
from sensor_msgs.msg import Joy


class ScalarMapping:
    def __init__(self, input_range, output_range):
        super().__init__()
        self.input_range = input_range
        self.output_range = output_range
        self.input_len = self.input_range[1]-self.input_range[0]
        self.output_len = self.output_range[1]-self.output_range[0]

    def get_output(self, input_val):
        ratio = (input_val - self.input_range[0])/self.input_len
        return self.output_range[0] + ratio * self.output_len


def do_nothing():
    pass


class BtnParse:
    def __init__(self,
                 on_push=do_nothing,
                 on_release=do_nothing,
                 initial_state=0):
        self.prev_state = initial_state
        self.on_push = on_push
        self.on_release = on_release

    def parse(self, value):
        if value is self.prev_state:
            return
        elif value is 0:
            self.prev_state = 0
            self.on_release()
        else:
            self.prev_state = 1
            self.on_push()


# Joy message input Range
# axes[0] : steer { Right end: -1, Left end: 1}
# axes[1] : clutch { Non: -1, Full: 1}
# axes[2] : accelerator { Non: -1, Full: 1}
# axes[3] : brake { Non:-1, Full:1 }
# axes[4] : arrow horizontal : {R:-1, N:0, L:1}
# axes[5] : arrow vertical : {D:-1, N:0, U:1}
# buttons[0] :
# buttons[1] :
# buttons[2] :
# buttons[3] :
# buttons[4] : gear up
# buttons[5] : gear down
# buttons[6] :
# buttons[7] :
# buttons[8] :
# buttons[9] :

class GearBtn:
    def __init__(self):
        self.gear = 1

    def gear_up(self):
        self.gear = self.gear + 1
        if self.gear > 2:
            self.gear = 2

    def gear_down(self):
        self.gear = self.gear - 1
        if self.gear < 0:
            self.gear = 0


class Interpreter:
    def __init__(self):
        rospy.Subscriber("joy", Joy, self.callback)
        self.pub = rospy.Publisher('target', target, queue_size=10)
        self.steer_map = ScalarMapping(
            [1.0, -1.0],
            [-math.pi/180.0*2000.0/71.0, math.pi/180.0*2000.0/71.0])
        self.brake_map = ScalarMapping([-1.0, 1.0], [1.0, 200.0])
        self.acc_map = ScalarMapping([-1.0, 1.0], [0.0, 20.0*1000.0/3600.0])
        self.gearbtn = GearBtn()
        self.btn4 = BtnParse(self.gearbtn.gear_down)
        self.btn5 = BtnParse(self.gearbtn.gear_up)

    def callback(self, msg):
        msg_target = target()
        msg_target.steer_rad = self.steer_map.get_output(msg.axes[0])
        msg_target.speed_mps = self.acc_map.get_output(msg.axes[2])
        msg_target.brake = int(self.brake_map.get_output(msg.axes[3]))
        self.btn4.parse(msg.buttons[4])
        self.btn5.parse(msg.buttons[5])
        msg_target.gear = self.gearbtn.gear
        self.pub.publish(msg_target)


if __name__ == '__main__':
    try:
        rospy.init_node('Monitor', anonymous=True)
        interpreter = Interpreter()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
