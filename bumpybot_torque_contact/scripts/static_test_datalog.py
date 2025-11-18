#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import csv
import os
import time

class TorqueDataAverager:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('torque_data_averager', anonymous=True)

        # Offset sampling (first 3 seconds)
        self.offsets = {}
        self.sample_data = {}   # raw samples for offset calculation
        self.sampling = True
        self.start_time = rospy.get_time()

        # 15-second accumulation
        self.accum_window = 15.0
        self.accum_data = {}       # joint -> [sum_raw, sum_offset, count]
        self.last_accum_print_time = rospy.get_time()

        # Will hold the sorted list of joints after offset sampling
        self.joint_list = []
        self.header_written = False

        # Prepare CSV file (overwrite on each run)
        self.csv_filename = "torque_data.csv"
        self.csv_file = open(self.csv_filename, "w")
        self.csv_writer = csv.writer(self.csv_file)

        # Subscriber
        rospy.Subscriber('/torque_sensor_v', JointState, self.callback)

    def callback(self, msg):
        current_time = rospy.get_time()

        # 1) During the first 3 seconds, gather data for offset
        if self.sampling:
            if current_time - self.start_time <= 3.0:
                for i, name in enumerate(msg.name):
                    self.sample_data.setdefault(name, []).append(msg.position[i])
            else:
                # Done collecting offset data
                self.sampling = False
                self.calculate_offsets()

                # Once offsets are known, define the final joint list
                self.joint_list = sorted(self.sample_data.keys())

                # Write the CSV header with dynamic columns:
                #   [timestamp, raw_joint0, ..., raw_jointN, offset_joint0, ..., offset_jointN]
                header = ["timestamp"]
                for joint in self.joint_list:
                    header.append("raw_{}".format(joint))
                for joint in self.joint_list:
                    header.append("offset_{}".format(joint))
                self.csv_writer.writerow(header)
                self.csv_file.flush()

            return

        # 2) If offset sampling is complete, accumulate data
        for i, name in enumerate(msg.name):
            # If this joint wasn't discovered in the first 3s, skip or handle as desired
            if name not in self.joint_list:
                # Optionally skip or add it on the fly.
                # For simplicity, let's skip unknown joints to keep CSV columns consistent.
                continue

            raw_val = msg.position[i]

            # Build accum data structure if needed
            if name not in self.accum_data:
                self.accum_data[name] = [0.0, 0.0, 0]

            self.accum_data[name][0] += raw_val   # sum of raw
            offset_val = raw_val - self.offsets.get(name, 0.0)
            self.accum_data[name][1] += offset_val  # sum of offset-removed
            self.accum_data[name][2] += 1           # count

        # 3) Check if 15 seconds have passed
        if current_time - self.last_accum_print_time >= self.accum_window:
            self.log_and_save_averages()
            self.accum_data = {}
            self.last_accum_print_time = current_time

    def calculate_offsets(self):
        """Compute mean offset per joint from collected samples."""
        for joint, samples in self.sample_data.items():
            self.offsets[joint] = np.mean(samples)
        rospy.loginfo("Calculated offsets: %s" % str(self.offsets))

    def log_and_save_averages(self):
        """
        Compute average raw and offset-removed data for each joint,
        print to console, and write one row to CSV with all joints
        separated into columns.
        """
        now_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        rospy.loginfo("=== 15-second average at %s ===" % now_str)

        # Build row: [timestamp, raw_joint0, ..., raw_jointN, offset_joint0, ..., offset_jointN]
        row = [now_str]

        # We first collect the raw_avg columns, then offset_avg columns
        raw_avgs = []
        offset_avgs = []

        for joint in self.joint_list:
            if joint not in self.accum_data or self.accum_data[joint][2] == 0:
                # No data for this joint in the last 15s
                raw_avg = 0.0
                offset_avg = 0.0
            else:
                sum_raw, sum_offset, count_samples = self.accum_data[joint]
                raw_avg = sum_raw / float(count_samples)
                offset_avg = sum_offset / float(count_samples)

            raw_avgs.append(raw_avg)
            offset_avgs.append(offset_avg)

            rospy.loginfo("Joint: %s | raw_avg=%.5f | offset_avg=%.5f"
                          % (joint, raw_avg, offset_avg))

        # Now combine them into the final row
        row.extend(raw_avgs)
        row.extend(offset_avgs)

        # Write to CSV
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def run(self):
        rospy.spin()
        self.csv_file.close()

if __name__ == '__main__':
    try:
        node = TorqueDataAverager()
        node.run()
    except rospy.ROSInterruptException:
        pass
