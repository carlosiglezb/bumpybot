#!/usr/bin/env python2

import rospy
import numpy as np
from collections import deque
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse
from dynamic_reconfigure.server import Server
from bumpybot_torque_contact.cfg import TorqueDataFilterConfig

class TorqueDataFilter:
    def __init__(self):
        rospy.init_node('torque_data_filter', anonymous=True)

        self.offset_buffers = {}
        self.offsets = {}
        self.offset_window_duration = rospy.get_param('~offset_window_duration', 10.0)

        self.torque_scalars = {
            'wheel0_joint': rospy.get_param('~wheel0_torque_scalar', 2.0),
            'wheel1_joint': rospy.get_param('~wheel1_torque_scalar', 2.0),
            'wheel2_joint': rospy.get_param('~wheel2_torque_scalar', 2.0)
        }

        self.filtered_pub = rospy.Publisher('/filtered_torque_data', JointState, queue_size=1)

        self.enable_downsampling = rospy.get_param('~enable_downsampling', True)
        self.downsample_factor = rospy.get_param('~downsample_factor', 100)
        self.enable_low_pass = rospy.get_param('~enable_low_pass', True)
        self.low_pass_window_size = rospy.get_param('~low_pass_window_size', 5)

        self.downsample_buffers = {}
        self.low_pass_buffers = {}

        self.server = Server(TorqueDataFilterConfig, self.dynamic_reconfig_callback)
        rospy.Subscriber('/torque_sensor_v', JointState, self.callback)
        self.zero_service = rospy.Service('reset_offsets', Trigger, self.reset_offsets)

    def dynamic_reconfig_callback(self, config, level):
        self.enable_downsampling = config.enable_downsampling
        self.downsample_factor = config.downsample_factor
        self.enable_low_pass = config.enable_low_pass
        self.low_pass_window_size = config.low_pass_window_size
        self.offset_window_duration = config.zeroing_duration

        rospy.loginfo("Reconfigure: downsample=%s, factor=%d, low_pass=%s, lpw=%d, offset_window=%.2f",
                      self.enable_downsampling, self.downsample_factor,
                      self.enable_low_pass, self.low_pass_window_size, self.offset_window_duration)
        return config

    def reset_offsets(self, req):
        self.offset_buffers.clear()
        self.offsets.clear()
        rospy.loginfo("Offsets have been reset.")
        return TriggerResponse(success=True, message="Offsets reset successfully.")

    def callback(self, torque_msg):
        current_time = rospy.Time.now().to_sec()

        filtered_msg = JointState()
        filtered_msg.header = torque_msg.header
        filtered_msg.name = torque_msg.name
        filtered_msg_positions = []

        for i, name in enumerate(torque_msg.name):
            raw_val = torque_msg.position[i]

            if name not in self.offset_buffers:
                self.offset_buffers[name] = deque(maxlen=1000)
            self.offset_buffers[name].append((current_time, raw_val))

            # Remove outdated offset samples
            while self.offset_buffers[name] and (current_time - self.offset_buffers[name][0][0]) > self.offset_window_duration:
                self.offset_buffers[name].popleft()

            # Update offsets continuously
            self.offsets[name] = np.mean([val for (_, val) in self.offset_buffers[name]])

            offset = self.offsets.get(name, 0.0)
            corrected_val = raw_val - offset

            scalar = self.torque_scalars.get(name, 1.0)
            corrected_val *= scalar

            if self.enable_low_pass:
                corrected_val = self.low_pass_filter(name, corrected_val)

            if self.enable_downsampling:
                corrected_val = self.downsample_filter(name, corrected_val)
                if corrected_val is None:
                    continue

            filtered_msg_positions.append(corrected_val)

        if filtered_msg_positions:
            filtered_msg.position = filtered_msg_positions
            self.filtered_pub.publish(filtered_msg)

    def downsample_filter(self, joint, x_new):
        buf = self.downsample_buffers.setdefault(joint, [])
        buf.append(x_new)

        if len(buf) >= self.downsample_factor:
            avg_val = np.mean(buf)
            self.downsample_buffers[joint] = []
            return avg_val
        return None

    def low_pass_filter(self, joint, x_new):
        buf = self.low_pass_buffers.setdefault(joint, deque(maxlen=self.low_pass_window_size))
        buf.append(x_new)
        return np.mean(buf)

if __name__ == '__main__':
    try:
        TorqueDataFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
