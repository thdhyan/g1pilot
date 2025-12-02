#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import evdev
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

STANDARD_AXIS_ORDER = [
    "left_x",
    "left_y",
    "right_x",
    "right_y",
    "l2",
    "r2",
    "arrow_vertical",
    "arrow_horizontal",
]

STANDARD_BUTTON_ORDER = [
    "cross",
    "circle",
    "square",
    "triangle",
    "l1",
    "r1",
    "l2",
    "r2",
    "l3",
    "r3",
    "share",
    "options",
    "ps",
    "touchpad",
]

AXIS_CODE_GROUPS = {
    "left_x":  [evdev.ecodes.ABS_X],
    "left_y":  [evdev.ecodes.ABS_Y],
    "right_x": [evdev.ecodes.ABS_Z],
    "right_y": [evdev.ecodes.ABS_RZ],
    "arrow_vertical":   [evdev.ecodes.ABS_HAT0Y],
    "arrow_horizontal": [evdev.ecodes.ABS_HAT0X],
    "l2":      [evdev.ecodes.ABS_BRAKE],
    "r2":      [evdev.ecodes.ABS_GAS],
}

BUTTON_CODE_GROUPS = {
    "cross":     [evdev.ecodes.BTN_SOUTH,  evdev.ecodes.BTN_A],
    "circle":    [evdev.ecodes.BTN_EAST,   evdev.ecodes.BTN_B],
    "square":    [evdev.ecodes.BTN_WEST,   evdev.ecodes.BTN_X],
    "triangle":  [evdev.ecodes.BTN_NORTH,  evdev.ecodes.BTN_Y],
    "l1":        [evdev.ecodes.BTN_TL],
    "r1":        [evdev.ecodes.BTN_TR],
    "l2":        [evdev.ecodes.BTN_TL2],
    "r2":        [evdev.ecodes.BTN_TR2],
    "l3":        [evdev.ecodes.BTN_THUMBL],
    "r3":        [evdev.ecodes.BTN_THUMBR],
    "share":     [evdev.ecodes.BTN_SELECT],
    "options":   [evdev.ecodes.BTN_START],
    "ps":        [evdev.ecodes.BTN_MODE],
    "touchpad":  [evdev.ecodes.BTN_TOUCH],
}


class ManualJoystick(Node):
    def __init__(self):
        super().__init__('manual_joystick')

        self.publisher = self.create_publisher(Joy, '/g1pilot/joy_manual', 10)
        self.auto_pub = self.create_publisher(Bool, '/g1pilot/auto_enable', 10)

        self.declare_parameter("publish_rate", 50.0)
        self.rate = self.get_parameter("publish_rate").get_parameter_value().double_value

        self.declare_parameter("joystick_name", "Xbox Wireless Controller")
        self.joystick_name = self.get_parameter("joystick_name").get_parameter_value().string_value

        self.device = self.find_joystick()
        if self.device is None:
            self.get_logger().error('No joystick found matching name: "%s"' % self.joystick_name)
            return

        self.get_logger().info(f'Using joystick: {self.device.name} ({self.device.path})')

        self.axis_code_to_name = {}
        self.button_code_to_name = {}

        self.current_axes = {name: 0.0 for name in STANDARD_AXIS_ORDER}
        self.current_buttons = {name: 0 for name in STANDARD_BUTTON_ORDER}

        self.auto_enabled = False
        self.triangle_prev = 0
        self.lock = threading.Lock()

        self.build_mapping()

        self.thread = threading.Thread(target=self.read_joystick, daemon=True)
        self.thread.start()

        self.create_timer(1.0 / self.rate, self.publish_joy)

    def find_joystick(self):
        for path in evdev.list_devices():
            dev = evdev.InputDevice(path)
            if dev.name == self.joystick_name:
                return dev
        return None

    def build_mapping(self):
        caps = self.device.capabilities(absinfo=True)

        abs_entries = caps.get(evdev.ecodes.EV_ABS, [])
        abs_codes = [code for (code, _) in abs_entries]

        key_entries = caps.get(evdev.ecodes.EV_KEY, [])
        key_codes = key_entries

        for name, candidates in AXIS_CODE_GROUPS.items():
            for code in candidates:
                if code in abs_codes:
                    self.axis_code_to_name[code] = name
                    break
        for name, candidates in BUTTON_CODE_GROUPS.items():
            for code in candidates:
                if code in key_codes:
                    self.button_code_to_name[code] = name
                    break

        self.get_logger().info("Axis mapping (code -> name): %s" % str(self.axis_code_to_name))
        self.get_logger().info("Button mapping (code -> name): %s" % str(self.button_code_to_name))

    def normalize_axis(self, code, raw_value):
        absinfo = self.device.absinfo(code)
        if absinfo.max == absinfo.min:
            return 0.0
        val = (2.0 * (raw_value - absinfo.min) / (absinfo.max - absinfo.min)) - 1.0
        if val > 1.0:
            val = 1.0
        elif val < -1.0:
            val = -1.0
        return float(val)

    def read_joystick(self):
        try:
            for event in self.device.read_loop():

                if event.type == evdev.ecodes.EV_ABS:
                    if event.code in self.axis_code_to_name:
                        name = self.axis_code_to_name[event.code]
                        value = self.normalize_axis(event.code, event.value)
                        with self.lock:
                            self.current_axes[name] = value

                elif event.type == evdev.ecodes.EV_KEY:
                    if event.code in self.button_code_to_name:
                        name = self.button_code_to_name[event.code]
                        pressed = 1 if event.value > 0 else 0

                        with self.lock:
                            self.current_buttons[name] = pressed

                        if name == "triangle":
                            if self.triangle_prev == 0 and pressed == 1:
                                self.auto_enabled = not self.auto_enabled
                                self.auto_pub.publish(Bool(data=self.auto_enabled))
                                self.get_logger().info(f"Auto mode set to: {self.auto_enabled}")
                            self.triangle_prev = pressed

        except OSError as e:
            self.get_logger().error(f"Error reading joystick: {e}")

    def publish_joy(self):
        with self.lock:
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.axes = [self.current_axes[name] for name in STANDARD_AXIS_ORDER]

            msg.buttons = [self.current_buttons[name] for name in STANDARD_BUTTON_ORDER]

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ManualJoystick()

    if node.device is None:
        node.get_logger().error("Shutting down ManualJoystick because no joystick was detected.")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
