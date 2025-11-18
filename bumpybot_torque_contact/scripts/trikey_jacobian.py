#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import os
import numpy as np
import rospy
import tf2_ros
import rospkg
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64MultiArray
from dynamic_reconfigure.server import Server
from bumpybot_torque_contact.cfg import JacobianConfig
import math
from collections import deque
from BBpolygons import load_BB_outline
from message_filters import Subscriber, ApproximateTimeSynchronizer

class ContactJacobian():
    def __init__(self,R, rw, rr, m, Br_alpha, Br_beta, F_MIN, outline_path=None):
        # Dynamic reconfigure
        self.R = R
        # Basic params
        self.rw, self.rr = rw, rr
        self.m, self.Br_alpha, self.Br_beta = m, Br_alpha, Br_beta
        self.scale = 1.0
        self.visualize_threshold = 0.025
        L = math.sqrt(3) * self.R
        self.M_mat = np.diag([m, m, L**2/2])
        self.Ib = 0.5 * m * L**2
        self.PAR_TOL = 1e-8
        self.F_MIN = F_MIN
        # Load outline & COM
        if outline_path is None:
            pkg = rospkg.RosPack().get_path('bumpybot_torque_contact')
            outline_path = os.path.join(pkg, 'cfg', 'BB_outline.csv')
        self.geom_vertices = load_BB_outline(outline_path)
        arr = np.array(self.geom_vertices)
        self.com = arr.mean(axis=0)
        self.last_cp = tuple(self.com)

        # State vars
        self.theta = None
        self.wz = None
        self.last_wz_time = None    # ← add this
        self.vx = None
        self.vy = None

        self.acceleration = None
        self.angular_accel_z = None
        self.torque_sensed = None

        # Health trackers
        self.last_imu_msg_time = None
        self.last_odom_msg_time = None
        self.last_torque_msg_time = None
        self.last_wheel_msg_time = None

        # Wait for clock
        while rospy.Time.now().to_sec() == 0:
            rospy.sleep(0.1)

        # Publishers
        self.pub_sphere = rospy.Publisher('contact_point', Marker, queue_size=1)
        self.pub_arrow  = rospy.Publisher('force_arrow', Marker, queue_size=1)
        self.force_pub  = rospy.Publisher('external_force_values', Float64MultiArray, queue_size=1)
        self._init_markers()

        # Subscribers: sync sensors + health checks
        imu_sub    = Subscriber('/imu/data', Imu)
        odom_sub   = Subscriber('/odometry/filtered', Odometry)
        torque_sub = Subscriber('/filtered_torque_data', JointState)
        wheel_sub  = Subscriber('/joint_states', JointState)

        ats = ApproximateTimeSynchronizer(
            [imu_sub, odom_sub, torque_sub, wheel_sub],
            queue_size=3, slop=0.025)
        ats.registerCallback(self.synced_callback)

        rospy.Subscriber('/imu/data', Imu,    self._health_imu_cb)
        rospy.Subscriber('/odometry/filtered', Odometry, self._health_odom_cb)
        rospy.Subscriber('/filtered_torque_data', JointState, self._health_torque_cb)
        rospy.Subscriber('/joint_states', JointState, self._health_wheel_cb)
        rospy.Timer(rospy.Duration(0.01), self._check_topic_health)
        #  OUTLIER FILTER STATE
        self.enable_outlier_rejection = True
        self.outlier_threshold_ratio  = 2.0
        self.outlier_window_size      = 5
        self.enable_outlier_reset     = False

        # keep a rolling buffer of the last N force magnitudes for median
        self._force_mag_buffer = deque(maxlen=self.outlier_window_size)

        # store last “valid” [cx, cy, Fx, Fy] in case of reset
        self._last_valid_output = (self.com[0], self.com[1], 0.0, 0.0)
        self.server = Server(JacobianConfig, self.dynamic_reconfig_callback)



    def dynamic_reconfig_callback(self, config, level):
        self.m     = config.mass
        self.R =      config.R
        self.Br_alpha    = config.roller_damping_alpha
        self.Br_beta     = config.roller_damping_beta
        self.scale = config.scale if hasattr(config, 'scale') else 1.0
        # recompute inertia terms if mass or R change
        L = math.sqrt(3) * self.R
        self.M_mat = np.diag([self.m, self.m, L**2/2])
        self.Ib    = 0.5 * self.m * L**2
        self.enable_outlier_rejection = config.enable_outlier_rejection
        self.outlier_threshold_ratio  = config.outlier_threshold_ratio
        self.outlier_window_size      = config.outlier_window_size
        self.enable_outlier_reset     = config.enable_outlier_reset
        self.F_MIN                   = config.F_MIN


        # if window size changed, rebuild buffer
        if len(self._force_mag_buffer) != self.outlier_window_size:
            self._force_mag_buffer = deque(maxlen=self.outlier_window_size)
        return config
    def _apply_outlier_filter(self, cx, cy, Fx, Fy):
        """
        Returns (filtered_cx, filtered_cy, filtered_Fx, filtered_Fy, is_outlier_flag)
        - If enable_outlier_rejection=False, simply returns inputs, False.
        - Otherwise, compare current |F| to median(|F_last|). If |F| > ratio*median,
          mark as outlier. Depending on enable_outlier_reset, either revert to last valid
          or just pass raw but flag it.
        """
        current_mag = math.hypot(Fx, Fy)

        if not self.enable_outlier_rejection:
            # always accept
            self._force_mag_buffer.append(current_mag)
            self._last_valid_output = (cx, cy, Fx, Fy)
            return cx, cy, Fx, Fy, False

        # 1) update buffer if non‐zero
        if current_mag > 0:
            self._force_mag_buffer.append(current_mag)

        # 2) compute median of buffer (if buffer not empty)
        if len(self._force_mag_buffer) < 1:
            median_mag = 0.0
        else:
            sorted_buf = sorted(self._force_mag_buffer)
            mid = len(sorted_buf) // 2
            if len(sorted_buf) % 2 == 1:
                median_mag = sorted_buf[mid]
            else:
                median_mag = 0.5 * (sorted_buf[mid-1] + sorted_buf[mid])

        # 3) decide if outlier
        threshold = self.outlier_threshold_ratio * (median_mag + 1e-9)
        is_outlier = (current_mag > threshold)

        if not is_outlier:
            # not an outlier → update last valid & return raw
            self._last_valid_output = (cx, cy, Fx, Fy)
            return cx, cy, Fx, Fy, False

        # is an outlier: either reset to last valid or pass raw
        if self.enable_outlier_reset:
            # revert to last valid contact/force
            lx, ly, lFx, lFy = self._last_valid_output
            return lx, ly, lFx, lFy, True
        else:
            # still return current, but flag outlier
            return cx, cy, Fx, Fy, True

    def _init_markers(self):
        self.arrow_marker = Marker(type=Marker.ARROW)
        self.arrow_marker.header.frame_id = 'base_link'
        self.arrow_marker.scale.x = self.arrow_marker.scale.y = self.arrow_marker.scale.z = 0.1
        self.arrow_marker.color.a = 1.0

        self.sphere_marker = Marker(type=Marker.SPHERE)
        self.sphere_marker.header.frame_id = 'base_link'
        self.sphere_marker.scale.x = self.sphere_marker.scale.y = self.sphere_marker.scale.z = 0.025
        self.sphere_marker.color.r = 1.0; self.sphere_marker.color.a = 1.0

    def _health_imu_cb(self, msg):    self.last_imu_msg_time    = rospy.Time.now()
    def _health_odom_cb(self, msg):   self.last_odom_msg_time   = rospy.Time.now()
    def _health_torque_cb(self, msg): self.last_torque_msg_time = rospy.Time.now()
    def _health_wheel_cb(self, msg):  self.last_wheel_msg_time  = rospy.Time.now()

    def synced_callback(self, imu_msg, odom_msg, torque_msg, wheel_msg):
        now = rospy.Time.now()
        self.last_imu_msg_time = self.last_odom_msg_time = \
        self.last_torque_msg_time = self.last_wheel_msg_time = now

        # IMU: linear accel
        ax, ay = imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y
        self.acceleration = np.array([[ax], [ay], [0]])
        # Print debug for IMU acceleration
        # ODOM: orientation (yaw) and angular velocity (wz)
        orientation_q = odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.theta = yaw

        self.vx = odom_msg.twist.twist.linear.x = 0 #THESE DONT WORK YET?
        self.vy = odom_msg.twist.twist.linear.y = 0

        wz = odom_msg.twist.twist.angular.z
        tstamp = odom_msg.header.stamp.to_sec()

        if self.last_wz_time is None:
            # first call: just initialize
            self.angular_accel_z = 0.0
        else:
            dt = tstamp - self.last_wz_time
            if dt > 1e-6:
                self.angular_accel_z = (wz - self.wz) / dt
            else:
                # too small or non-positive dt
                self.angular_accel_z = 0.0

        self.wz = wz
        self.last_wz_time = tstamp

        # Torque
        self.torque_sensed = np.array(torque_msg.position).reshape(3,1)

        # All data ready?
        if self.theta is None or self.acceleration is None or \
           self.angular_accel_z is None or self.torque_sensed is None:
            return

        # Build Jacobians & compute forces
        self.build_jacobians(self.theta)
        cx,cy,Fx,Fy,ok = self.external_forces()
        # Apply filters
        fcx, fcy, fFx, fFy, is_outlier = self._apply_outlier_filter(cx, cy, Fx, Fy)

        # Combine the two boolean flags (“ok” from intersection, and “not an outlier”)
        final_ok = ok and (not is_outlier)
        out = [fcx, fcy, fFx, fFy, 1.0 if final_ok else 0.0]
        self.visualize(out)
        msg = Float64MultiArray(); msg.data = out
        self.force_pub.publish(msg)

    def build_jacobians(self, th):
        self.Jcw = (1/self.rw)*np.array([
            [-math.sin(th), math.cos(th), self.R],
            [-math.sin(th+2*math.pi/3), math.cos(th+2*math.pi/3), self.R],
            [-math.sin(th+4*math.pi/3), math.cos(th+4*math.pi/3), self.R]
        ])
        self.Jcwinv = np.linalg.inv(self.Jcw)
        self.Jcr = (1/self.rr)*np.array([
            [math.cos(th), math.sin(th), 0],
            [math.cos(th+2*math.pi/3), math.sin(th+2*math.pi/3), 0],
            [math.cos(th+4*math.pi/3), math.sin(th+4*math.pi/3), 0]
        ])

    def external_forces(self):
        # roller damping like MATLAB: Br_k = 0.2*tanh(0.4*qr_dot)
        Xd = np.array([[self.vx],[self.vy],[self.wz]])
        qr_dot = self.Jcr.dot(Xd)

        Br_vec = self.Br_alpha * np.tanh(self.Br_beta* qr_dot)

        # inertia wrench
        X_dd = np.vstack([self.acceleration[0:2], [[self.angular_accel_z]]])
        T_noF = self.Jcwinv.T.dot(self.M_mat.dot(X_dd) + self.Jcr.T.dot(Br_vec))

        # residual & wrench
        deltaT = T_noF - self.torque_sensed
        W = self.scale * (self.Jcw.T.dot(deltaT))
        Fx, Fy = W[0,0], W[1,0]
        if math.hypot(Fx, Fy) < self.F_MIN:
            return [self.last_cp[0],
                self.last_cp[1],
                0.0,  # zero out small Fx
                0.0,  # zero out small Fy
                0.0]  # hit flag
        # moment balance
        RH = self.scale*(self.Ib*self.angular_accel_z - (self.R/self.rw)*np.sum(self.torque_sensed))
        a, b, c = Fy, -Fx, -RH

        pt, ok = self.force_line_intersection(self.geom_vertices, (a,b,c), (Fx, Fy))
        return [pt[0], pt[1], Fx, Fy, 1.0 if ok else 0.0]

    def force_line_intersection(self, verts, abc, force_xy):
            a, b, c = abc
            Fx, Fy = force_xy
            

            tol = self.PAR_TOL * (abs(a) + abs(b))
            pts = []
            
            # Find all edge intersections -
            for i in range(len(verts)):
                j = (i + 1) % len(verts)  # equivalent to mod(i,M)+1 in MATLAB
                v1 = np.array(verts[i])
                v2 = np.array(verts[j])
                d = v2 - v1
                denom = a * d[0] + b * d[1]
                
                if abs(denom) < tol:
                    continue
                    
                t = -(a * v1[0] + b * v1[1] + c) / denom
                if 0 <= t <= 1:
                    intersection = v1 + t * d
                    pts.append(tuple(intersection))
            

            if not pts:
                return (self.last_cp, False)
            

            pts_array = np.array(pts)
            dirs = pts_array - self.com
            projs = dirs.dot(np.array([Fx, Fy]))
            
            if np.any(projs > 0):
                # pick the intersection in the half-plane the force points to
                idx = np.argmax(projs)
            else:
                # if force is weirdly zero or all projections ≤0, fall back
                dists = np.linalg.norm(dirs, axis=1)
                idx = np.argmin(dists)
            
            cp = pts[idx]
            self.last_cp = cp
            return (cp, True)

    def visualize(self, data):
        cx, cy, Fx, Fy, hit = data
        norm = math.hypot(Fx, Fy)
        if not hit:
            rospy.logwarn_throttle(1, "No intersection to visualize")
            return
        self.sphere_marker.header.stamp = rospy.Time.now()
        self.sphere_marker.pose.position.x = cx
        self.sphere_marker.pose.position.y = cy
        self.pub_sphere.publish(self.sphere_marker)
        if norm < self.visualize_threshold:
            self.arrow_marker.points = []
            self.pub_arrow.publish(self.arrow_marker)
            return
        start = Point(x=cx,y=cy,z=0)
        end   = Point(x=cx+0.5*Fx/norm,y=cy+0.5*Fy/norm,z=0)
        self.arrow_marker.points = [start,end]
        self.pub_arrow.publish(self.arrow_marker)

    def _check_topic_health(self, event):
        now = rospy.Time.now()
        for last, topic in [
            (self.last_imu_msg_time, '/imu/data'),
            (self.last_odom_msg_time, '/odometry/filtered'),
            (self.last_torque_msg_time, '/filtered_torque_data'),
            (self.last_wheel_msg_time, '/joint_states')]:
            if last is None or (now-last).to_sec()>2.0:
                rospy.logwarn_throttle(5.0, "Topic %s has not published recently", topic)
if __name__=='__main__':
    rospy.init_node('Contact_Jacobian')
    params = {k: rospy.get_param('~'+k, v) for k,v in zip(
        ['R','wheel_radius','roller_radius','mass','roller_damping_alpha', 'roller_damping_beta', 'F_MIN','scale'],
        [0.248195487469,0.1,0.00918135,80,0.2,0.4,0.5,1.0]
    )}
    ContactJacobian(
        params['R'],
        params['wheel_radius'], params['roller_radius'],
        params['mass'], params['roller_damping_alpha'],params['roller_damping_beta'],params['F_MIN']
    )
    rospy.spin()
