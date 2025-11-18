#!/usr/bin/env python2
# -*- coding: utf-8 -*-
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
        # wait for /clock
        while rospy.Time.now().to_sec() == 0:
            rospy.loginfo("Torque Filter: Waiting for /clock to start...")
            rospy.sleep(0.1)

        # one-shot zeroing start time
        self.t_start = None
        # how long to collect offsets (s) - default to 3 seconds for one-time zeroing
        self.offset_window_duration = rospy.get_param('~zeroing_duration', 3.0)
        # ring buffer of (time, [τ0,τ1,τ2])
        self.raw_buffer = deque()
        # current offsets
        self.offsets ={}
        self.zeroing_complete = False  # Flag to track if zeroing is complete
        # 3×3 gain matrix
        # mat = rospy.get_param('~torque_gain_matrix',
        #                       [[88.8657469028280, -2.31826985824766, -10.1472058064587],
        #                        [3.07144763837246, 99.9572429648342, 4.06206652000198],
        #                        [14.5197097128484, -5.66624109777176, 128.453167231654]])
        # mat = rospy.get_param('~torque_gain_matrix',
        #                 [[1.0, 0.0, 0.0],
        #                 [0.0, 1.0, 0.0],
        #                 [0.0, 0.0, 1.0]])
        # mat = rospy.get_param('~torque_gain_matrix',
        #     [[ 55.0374493309455,   23.5451620313316,  -14.7412041025670],
        #     [-73.7759614980451,  131.8614323146500,  -64.8347276524114],
        #     [  4.2792182296721,   47.7494184018265,   92.2438248736885]])
        # mat = rospy.get_param('~torque_gain_matrix',
        #                 [[76.9257480818023, 0.0, 0.0],
        #                 [0.0, 345.17, 0.0],
        #                 [0.0, 0.0, 101.829267332720]])
        mat = rospy.get_param('~torque_gain_matrix',
                [[171.514129326594,   13.5805868655599,  -12.2170517398387],
                [ -12.8517746107314, 191.552867936577,   32.5069778008329],
                [  16.57538230691,   -32.2870096548296, 196.809817669663]])

        self.torque_gain = np.array(mat)

        # downsampling
        self.enable_downsampling  = rospy.get_param('~enable_downsampling', False)
        self.downsample_factor    = rospy.get_param('~downsample_factor', 5)
        # original low-pass (boxcar)
        self.enable_low_pass      = rospy.get_param('~enable_low_pass', True)
        self.low_pass_window_size = rospy.get_param('~low_pass_window_size', 120)

        # buffers & state - simplified for one-time zeroing
        self.offset_buffers     = {}   # name → list[float] (simplified from deque)
        self.low_pass_buffers   = {}   # name → deque[float]
        self.downsample_buffers = {}   # name → list[float]

        # ROS interfaces
        self.filtered_pub = rospy.Publisher('/filtered_torque_data',
                                            JointState, queue_size=100)
        self.server = Server(TorqueDataFilterConfig,
                             self.dynamic_reconfig_callback)
        rospy.Subscriber('/torque_sensor_v',
                         JointState, self.callback)
        self.zero_service = rospy.Service('reset_offsets',
                                          Trigger, self.reset_offsets)

    def dynamic_reconfig_callback(self, config, level):
        self.enable_downsampling    = config.enable_downsampling
        self.downsample_factor      = config.downsample_factor
        self.enable_low_pass        = config.enable_low_pass
        self.low_pass_window_size   = config.low_pass_window_size
        self.offset_window_duration = config.zeroing_duration
        self.T_MIN                 = config.T_MIN

        rospy.loginfo("Reconfigure: downsample=%s factor=%d, low_pass=%s lpw=%d, offset_window=%.2f",
                      self.enable_downsampling, self.downsample_factor,
                      self.enable_low_pass, self.low_pass_window_size,
                      self.offset_window_duration)
        return config

    def reset_offsets(self, req):
        self.offset_buffers = {}
        self.offsets = {}
        self.t_start = None
        self.zeroing_complete = False  # Reset the completion flag
        rospy.loginfo("Offsets and zeroing state have been reset.")
        return TriggerResponse(success=True,
                               message="Offsets reset successfully.")

    def callback(self, torque_msg):
        t = rospy.Time.now().to_sec()
        # record first callback time
        if self.t_start is None:
            self.t_start = t
            rospy.loginfo("Starting %.1f-second zeroing period...", self.offset_window_duration)

        if self.enable_downsampling:
            if not hasattr(self, 'downsample_counter'):
                self.downsample_counter = 0
            self.downsample_counter += 1
            if self.downsample_counter % self.downsample_factor != 0:
                return  # Skip this message entirely

        orig_names = torque_msg.name[:] #Dont want to permute the names otherwise the output wont be correctly ordered
        # remap [0,1,2] → [2,0,1]
        perm = [2, 0, 1]
        names = [torque_msg.name[i] for i in perm]
        raw = np.array([torque_msg.position[i] for i in perm])

        # 4) update sliding raw_buffer for future instant‐zero service
        self.raw_buffer.append((t, raw.copy()))
        cutoff = t - self.offset_window_duration
        while self.raw_buffer and self.raw_buffer[0][0] < cutoff:
            self.raw_buffer.popleft()

        # 5) ONE‐TIME offset collection (initial zeroing)
        if not self.zeroing_complete:
            if (t - self.t_start) <= self.offset_window_duration:
                # collect offsets
                for i, name in enumerate(names):
                    self.offset_buffers.setdefault(name, []).append(raw[i])
                # publish all‐zero during window
                out = JointState()
                out.header = torque_msg.header
                out.name = orig_names
                out.position = [0.0] * len(orig_names)
                self.filtered_pub.publish(out)
                return
            else:
                # compute final offsets
                rospy.loginfo("Zeroing period complete. Calculating final offsets...")
                for name in names:
                    buf = self.offset_buffers.get(name, [])
                    self.offsets[name] = float(np.mean(buf)) if buf else 0.0
                    rospy.loginfo("Final offset for %s: %.6f", name, self.offsets[name])
                self.offset_buffers.clear()
                self.zeroing_complete = True
                rospy.loginfo("Zeroing complete. Offset buffers cleared.")

        # 6) apply offsets
        offsets = np.array([self.offsets.get(n, 0.0) for n in names])
        corrected = raw - offsets

        # 7) 3×3 gain
        scaled = corrected.dot(self.torque_gain.T)

        # 8) low‐pass boxcar
        if self.enable_low_pass:
            for i, name in enumerate(names):
                buf = self.low_pass_buffers.setdefault(name, deque(maxlen=self.low_pass_window_size))
                buf.append(scaled[i])
                scaled[i] = float(np.mean(buf))

        # 9) minimum threshold
        for i in range(len(scaled)):
            if abs(scaled[i]) < self.T_MIN:
                scaled[i] = 0.0

        # 10) publish filtered result
        out = JointState()
        out.header = torque_msg.header
        out.name = orig_names
        out.position = scaled.tolist()
        self.filtered_pub.publish(out)

if __name__ == '__main__':
    try:
        TorqueDataFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass