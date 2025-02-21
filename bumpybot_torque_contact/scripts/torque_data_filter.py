#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import JointState
import numpy as np
from collections import deque
from message_filters import Subscriber, ApproximateTimeSynchronizer
from dynamic_reconfigure.server import Server
from bumpybot_torque_contact.cfg import TorqueDataFilterConfig



class TorqueDataFilter:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('torque_data_filter', anonymous=True)

        # Offsets and sample data for offset calibration
        self.offsets = {}
        self.sample_data = {}
        self.sampling = True
        self.start_time = rospy.get_time()

        # Default filter parameters (will be updated by dynamic reconfigure)
        self.enable_hampel = rospy.get_param('~enable_hampel', True)
        self.hampel_window_size = rospy.get_param('~hampel_window_size', 211)  # ideally odd
        self.hampel_k = rospy.get_param('~hampel_k', 2.0)
        self.enable_low_pass = rospy.get_param('~enable_low_pass', True)
        self.low_pass_window_size = rospy.get_param('~low_pass_window_size', 5)
        self.non_movement_time = rospy.get_param('~non_movement_time', 5.0)
        self.zeroing_duration = rospy.get_param('~zeroing_duration', 2.0)
        self.velocity_threshold = rospy.get_param('~velocity_threshold', 1e-3)
        self.torque_scalars0 = rospy.get_param('~wheel0_torque_scalar', 2.0)
        self.torque_scalars1 = rospy.get_param('~wheel1_torque_scalar', 2.0)
        self.torque_scalars2 = rospy.get_param('~wheel2_torque_scalar', 2.0)
        # Internal buffers for filters
        self.hampel_buffers = {}      # for each joint: deque for Hampel filter
        self.low_pass_buffers = {}    # for each joint: deque for low pass filtering
        self.torque_buffer = {}       # for averaging over zeroing_duration

        # Timing variables for movement/averaging
        self.last_movement_time = rospy.get_time()
        self.averaging_end_time = 0.0

        # Publisher
        self.filtered_pub = rospy.Publisher('/filtered_torque_data', JointState, queue_size=1)

        # Use message_filters to combine topics (torque and joint state for velocities)
        self.torque_sub = Subscriber('/torque_sensor_v', JointState)
        self.joint_state_sub = Subscriber('/joint_states', JointState)
        self.sync = ApproximateTimeSynchronizer([self.torque_sub, self.joint_state_sub],
                                                queue_size=10, slop=0.1)
        self.sync.registerCallback(self.callback)

        # Setup dynamic reconfigure server
        self.server = Server(TorqueDataFilterConfig, self.dynamic_reconfig_callback)

    def dynamic_reconfig_callback(self, config, level):
        # Update all parameters from dynamic reconfigure
        self.enable_hampel = config.enable_hampel
        self.hampel_window_size = config.hampel_window_size
        self.hampel_k = config.hampel_k
        self.enable_low_pass = config.enable_low_pass
        self.low_pass_window_size = config.low_pass_window_size
        self.non_movement_time = config.non_movement_time
        self.zeroing_duration = config.zeroing_duration
        self.velocity_threshold = config.velocity_threshold

        rospy.loginfo("Reconfigure Request: enable_hampel=%s, hampel_window_size=%d, hampel_k=%.2f, "
                      "enable_low_pass=%s, low_pass_window_size=%d, non_movement_time=%.2f, "
                      "zeroing_duration=%.2f, velocity_threshold=%.5f",
                      self.enable_hampel, self.hampel_window_size, self.hampel_k,
                      self.enable_low_pass, self.low_pass_window_size,
                      self.non_movement_time, self.zeroing_duration, self.velocity_threshold)
        return config

    def hampel_filter(self, joint, x_new):
        """Real-time Hampel filter for outlier detection."""
        if joint not in self.hampel_buffers:
            self.hampel_buffers[joint] = deque([], maxlen=self.hampel_window_size)
        elif self.hampel_buffers[joint].maxlen != self.hampel_window_size:
            # Recreate the deque with new maxlen
            self.hampel_buffers[joint] = deque(self.hampel_buffers[joint], maxlen=self.hampel_window_size)
        self.hampel_buffers[joint].append(x_new)
        window_data = np.array(self.hampel_buffers[joint])
        median_ = np.median(window_data)
        abs_dev = np.abs(window_data - median_)
        mad_ = np.median(abs_dev) if len(abs_dev) > 0 else 0.0
        epsilon = 1e-9
        if mad_ < epsilon:
            return x_new
        threshold = self.hampel_k * mad_
        if np.abs(x_new - median_) > threshold:
            return median_
        else:
            return x_new

    def low_pass_filter(self, joint, x_new):
        """Simple moving average low pass filter."""
        if joint not in self.low_pass_buffers:
            self.low_pass_buffers[joint] = deque([], maxlen=self.low_pass_window_size)
        elif self.low_pass_buffers[joint].maxlen != self.low_pass_window_size:
            self.low_pass_buffers[joint] = deque(self.low_pass_buffers[joint], maxlen=self.low_pass_window_size)
        self.low_pass_buffers[joint].append(x_new)
        return np.mean(self.low_pass_buffers[joint])

    def update_torque_buffer(self, joint, current_time, value):
        """Maintain a time window (of length zeroing_duration) for each joint."""
        if joint not in self.torque_buffer:
            self.torque_buffer[joint] = deque()
        buf = self.torque_buffer[joint]
        buf.append((current_time, value))
        while buf and (current_time - buf[0][0] > self.zeroing_duration):
            buf.popleft()

    def compute_average(self, joint):
        """Compute the average torque value from the buffer."""
        buf = self.torque_buffer.get(joint, [])
        if not buf:
            return 0.0
        values = [val for (_, val) in buf]
        return np.mean(values)

    def callback(self, torque_msg, joint_state_msg):
        current_time = rospy.get_time()

        # Check movement using velocities from joint_state_msg (expected 3x1 for wheels)
        movement_detected = any(abs(v) > self.velocity_threshold for v in joint_state_msg.velocity)
        rospy.loginfo_throttle_identical(2, "Movement detected: %s" % movement_detected)
        if movement_detected:

            self.last_movement_time = current_time
            self.averaging_end_time = 0.0
        else:
            if (current_time - self.last_movement_time) > self.non_movement_time:
                if current_time > self.averaging_end_time:
                    self.averaging_end_time = current_time + self.zeroing_duration

        # During initial sampling, use torque_msg for offset calibration.
        if self.sampling:
            if current_time - self.start_time <= self.zeroing_duration:
                for i, name in enumerate(torque_msg.name):
                    self.sample_data.setdefault(name, []).append(torque_msg.position[i])
            else:
                self.sampling = False
                self.calculate_offsets()
            return

        # Prepare filtered message
        filtered_msg = JointState()
        filtered_msg.header = torque_msg.header
        filtered_msg.name = torque_msg.name
        filtered_msg.position = []
        filtered_msg.velocity = torque_msg.velocity
        filtered_msg.effort = torque_msg.effort

        # Process each joint
        for i, name in enumerate(torque_msg.name):
            raw_val = torque_msg.position[i]
            # Optionally apply Hampel filter
            if self.enable_hampel:
                filtered_val = self.hampel_filter(name, raw_val)
            else:
                filtered_val = raw_val

            # Subtract offset if available
            if name in self.offsets:
                filtered_val -= self.offsets[name]
            else:
                rospy.logwarn("Joint %s not in offset config. Passing raw data." % name)

            #After offsets removed, multiply values by their associated scalar
            if name == 'wheel0_joint':
                filtered_val *= self.torque_scalars0
            elif name == 'wheel1_joint':
                filtered_val *= self.torque_scalars1
            elif name == 'wheel2_joint':
                filtered_val *= self.torque_scalars2

            # Optionally apply low pass filter
            if self.enable_low_pass:
                filtered_val = self.low_pass_filter(name, filtered_val)

            # Update per-joint buffer for averaging window
            self.update_torque_buffer(name, current_time, filtered_val)

            # If in averaging period, use the average over the last zeroing_duration seconds.
            if current_time < self.averaging_end_time:
                avg_val = self.compute_average(name)
                filtered_msg.position.append(avg_val)
            else:
                filtered_msg.position.append(filtered_val)

        self.filtered_pub.publish(filtered_msg)

    def calculate_offsets(self):
        """Calculate offsets from collected samples."""
        for joint, samples in self.sample_data.items():
            self.offsets[joint] = np.mean(samples)
        rospy.loginfo("Calculated offsets: %s" % str(self.offsets))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        filter_node = TorqueDataFilter()
        filter_node.run()
    except rospy.ROSInterruptException:
        pass
