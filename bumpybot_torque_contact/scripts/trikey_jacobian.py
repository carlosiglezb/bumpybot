#!/usr/bin/env python2
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64MultiArray
from dynamic_reconfigure.server import Server
from bumpybot_torque_contact.cfg import JacobianConfig
import math

class ContactJacobian():
    def __init__(self, rw, rr, M, Br, Iw, Ir, Ib, TractionTorque):
        # type: (float, float, float, float, float, float, float, float) -> None

        # ---- Dynamic Reconfigure Server ----
        self.server = Server(JacobianConfig, self.dynamic_reconfig_callback)

        # ---- Robot parameters ----
        self.rw = rw
        self.rr = rr
        self.M  = M
        self.Br = np.array([[Br], [Br], [Br]])  # keep as 3x1
        self.Iw = Iw
        self.Ir = Ir
        self.Ib = Ib
        self.TractionTorque = TractionTorque
        self.visualize_threshold = 0.025

        # ---- Internal state ----
        self.theta = None  # type:float      # yaw
        self.velocity = None  # type: np.ndarray    # vx, vy, wz
        self.torque_sensed = None  # type: np.ndarray   # from /filtered_torque_data
        self.Jcw = None  # type: np.ndarray
        self.Jcwdot = None  # type: np.ndarray
        self.Jcwinv = None  # type: np.ndarray
        self.Jcwdot_inv = None  # type: np.ndarray
        self.Jcr = None  # type: np.ndarray
        self.Jcrdot = None  # type:np.ndarray
        self.acceleration = None  # type:list

        # Time-tracking for update loop
        self.t_now = None  # type:float
        self.t_last = None  # type:float
        self.delta_t = 0.0  # type: float

        self.t_last_torque = None

        # Track wheel velocities to compute \dot{\omega} from the difference
        self.angular_vel_wheels_now = None  # type:np.ndarray  # current wheel speeds
        self.angular_vel_wheels_prev = None  # type:np.ndarray  # previous wheel speeds
        self.wheel_angular_acceleration = None  # type:np.ndarray


        while rospy.Time.now().to_sec() == 0:

            rospy.loginfo("Contact Detection: Waiting for /clock to start...")
            rospy.sleep(0.1)


        # ---- Marker Publishers ----
        self.pub_sphere = rospy.Publisher("contact_point", Marker, queue_size=10)
        self.pub_arrow = rospy.Publisher("force_arrow", Marker, queue_size=10)
        self.force_value_pub = rospy.Publisher("external_force_values", Float64MultiArray, queue_size=10)

        #debug publisher
        self.pub_lina = rospy.Publisher("self_acceleration", Float64MultiArray, queue_size=10)
        self.marker = self._init_markers()

        # ---- TF for wheel positions ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.robot_vertices = self.lookup_wheel_positions()




        if self.robot_vertices is None:
            rospy.logerr("Could not find wheel positions from TF, waiting 0.2 seconds and trying again.")
            rospy.sleep(0.2)
            self.robot_vertices = self.lookup_wheel_positions()
            if self.robot_vertices is None:
                rospy.logerr("Still could not find wheel positions from TF, aborting.")
                rospy.signal_shutdown("TF lookup failed")
                exit(1)
        else:
            rospy.loginfo("Found wheel positions from TF: {}".format(self.robot_vertices))

        # ---- ROS Subscribers ----
        # 1) Odom for pose + velocity
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        # rospy.Subscriber("/imu/data", Imu, self.odom_callback)
        # 2) Sensed torque
        rospy.Subscriber("/filtered_torque_data", JointState, self.torque_callback)
        # 3) Wheel joint states (to get actual wheel velocities)
        rospy.Subscriber("/joint_states", JointState, self.wheelcallback)    
        
        rospy.loginfo("Contact Jacobian node initialized.")
        self.timer = rospy.Timer(rospy.Duration(0.01), self.update_callback)

        # ---------------------------------------------------------------------
    def dynamic_reconfig_callback(self, config, level):
        self.M = config.mass
        self.Br = [[config.roller_damping_Br]] * 3
        self.Iw = config.wheel_inertia_iw
        self.Ir = config.roller_inertia_ir
        self.Ib = config.body_inertia_ib
        self.TractionTorque = config.TractionTorque
        rospy.loginfo("Reconfigure Request: mass=%.2f, Br=%.2f, Iw=%.2f, Ir=%.2f, Ib=%.2f, TractionTorque=%.2f",
                    self.M, config.roller_damping_Br, self.Iw, self.Ir, self.Ib, self.TractionTorque)
        return config


    # ------------------- Init Marker ------------------- #
    def _init_markers(self):
# Arrow marker for the force vector
        self.arrow_marker = Marker()
        self.arrow_marker.header.frame_id = "base_link"
        self.arrow_marker.type = Marker.ARROW
        self.arrow_marker.scale.x =0.1# Shaft diameter
        self.arrow_marker.scale.y = 0.1  # Arrowhead diameter
        self.arrow_marker.scale.z = 0.1 
        self.arrow_marker.color.a = 1.0   # Alpha transparency
        self.threshold = self.visualize_threshold  # Threshold for arrow visualization
        # Sphere marker for the contact point
        self.sphere_marker = Marker()
        self.sphere_marker.header.frame_id = "base_link"
        self.sphere_marker.type = Marker.SPHERE
        self.sphere_marker.scale.x = 0.025  # Sphere diameter
        self.sphere_marker.scale.y = 0.025
        self.sphere_marker.scale.z = 0.025
        self.sphere_marker.color.r = 1.0   # Red color
        self.sphere_marker.color.g = 0.0
        self.sphere_marker.color.b = 0.0
        self.sphere_marker.color.a = 1.0   # Fully visible
        # For the arrow marker
        self.arrow_marker.pose.orientation.x = 0.0
        self.arrow_marker.pose.orientation.y = 0.0
        self.arrow_marker.pose.orientation.z = 0.0
        self.arrow_marker.pose.orientation.w = 1.0

        # For the sphere marker
        self.sphere_marker.pose.orientation.x = 0.0
        self.sphere_marker.pose.orientation.y = 0.0
        self.sphere_marker.pose.orientation.z = 0.0
        self.sphere_marker.pose.orientation.w = 1.0


    # (Optional) If you want to see the wheel frames as spheres
    def publish_wheel_markers(self, vertices):
        marker_arr = MarkerArray()
        now = rospy.Time.now()
        for i, (x, y) in enumerate(vertices):
            mk = Marker()
            mk.header.frame_id = "base_link"
            mk.header.stamp = now
            mk.ns = "wheel_positions"
            mk.id = i
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose.position.x = x
            mk.pose.position.y = y
            mk.pose.position.z = 0.0
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = 0.1
            mk.color.a = 1.0
            mk.color.r = 1.0
            mk.color.g = 0.0
            mk.color.b = 0.0
            marker_arr.markers.append(mk)
        # self.wheel_marker_pub.publish(marker_arr)

    # ------------------- Lookup Wheel Positions ------------------- #
    def lookup_wheel_positions(self):
        wheel_frames = ["wheel0", "wheel1", "wheel2"]
        base_frame   = "base_link"
        vertices = []
        for wheel_frame in wheel_frames:
            try:
                trans = self.tf_buffer.lookup_transform(
                    base_frame, wheel_frame, rospy.Time(0), rospy.Duration(0.1)
                )
            except:
                rospy.logerr("TF transform to {} not found.".format(wheel_frame))
                return None
            else:
                x = trans.transform.translation.x
                y = trans.transform.translation.y
                vertices.append((x, y))
                # store R as distance from center
                self.R = math.sqrt(x**2 + y**2)
        self.publish_wheel_markers(vertices)
        return vertices

    # ------------------- Hardware Callbacks ------------------- #
    def odom_callback(self, msg):
        ## new callback (Imu, not odom) we dont actually need linear velocity
        # 1) Yaw
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.theta = yaw

        # 2) Angular velocity (omega)
        # wx = msg.angular_velocity.x
        # wy = msg.angular_velocity.y
        # wz = msg.angular_velocity.z
        # self.velocity = np.array([[wx], [wy], [wz]])



        # # 3) Linear Accel from IMU
        # ax = msg.linear_acceleration.x
        # ay = msg.linear_acceleration.y
        # az = msg.linear_acceleration.z
        # self.acceleration=np.array([[ax], [ay], [az]])

        # # Build Jacobians if yaw is valid
        if self.theta is not None:
            self.build_jacobians(self.theta)
        ## old odom callback
        # # 1) Yaw
        # q = msg.pose.pose.orientation
        # quat = [q.x, q.y, q.z, q.w]
        # _, _, yaw = euler_from_quaternion(quat)
        # self.theta = yaw

        # 2) Robot velocity [vx, vy, wz]
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        self.velocity = np.array([[vx], [vy], [wz]])

        # # Build Jacobians if yaw is valid
        # if self.theta is not None:
        #     self.build_jacobians(self.theta)

    def torque_callback(self, joint_msg):
        """
        /filtered_torque_data is sensor_msgs/JointState
        'position' stores the 3 torque values (one per wheel).
        """
        self.torque_sensed = np.array(joint_msg.position).reshape(3,1)
        self.t_last_torque = rospy.Time.now()
        

    def wheelcallback(self, joint_msg):
        """
        Subscribe to /joint_states (wheel velocities in joint_msg.velocity).
        track consecutive calls to approximate dot_omega
        """
        # Must ensure that 'joint_msg.velocity' has 3 wheels in the same order.
        # If your real hardware has a different JointState layout, adapt accordingly.
        if len(joint_msg.velocity) < 3:
            return

        # current wheel speeds
        if self.angular_vel_wheels_now is None:
            self.angular_vel_wheels_now = np.array(joint_msg.velocity).reshape(3,1)
            self.angular_vel_wheels_prev = self.angular_vel_wheels_now.copy()
            return
        else:
            # shift the old into prev
            self.angular_vel_wheels_prev = self.angular_vel_wheels_now.copy()
            # new
            self.angular_vel_wheels_now  = np.array(joint_msg.velocity).reshape(3,1)

        self.wheel_angular_acceleration = self.angular_vel_wheels_now - self.angular_vel_wheels_prev

    # ------------------- Build Jacobians ------------------- #
    def build_jacobians(self, theta):
        self.Jcw = (1.0/self.rw)*np.array([
            [-math.sin(theta),                      math.cos(theta),                      self.R],
            [-math.sin(theta+2.0/3.0*math.pi),      math.cos(theta+2.0/3.0*math.pi),      self.R],
            [-math.sin(theta+4.0/3.0*math.pi),      math.cos(theta+4.0/3.0*math.pi),      self.R]
        ])

        self.Jcwdot = (1.0/self.rw)*np.array([
            [-math.cos(theta),                     -math.sin(theta),                      0.0],
            [-math.cos(theta+2.0/3.0*math.pi),     -math.sin(theta+2.0/3.0*math.pi),      0.0],
            [-math.cos(theta+4.0/3.0*math.pi),     -math.sin(theta+4.0/3.0*math.pi),      0.0]
        ])

        self.Jcwinv = np.linalg.inv(self.Jcw)
        self.Jcwdot_inv = np.linalg.pinv(self.Jcwdot)

        self.Jcr = (1.0/self.rr)*np.array([
            [math.cos(theta),                      math.sin(theta),                      0.0],
            [math.cos(theta+2.0/3.0*math.pi),      math.sin(theta+2.0/3.0*math.pi),      0.0],
            [math.cos(theta+4.0/3.0*math.pi),      math.sin(theta+4.0/3.0*math.pi),      0.0]
        ])

        self.Jcrdot = (1.0/self.rr)*np.array([
            [-math.sin(theta),                     math.cos(theta),                      0.0],
            [-math.sin(theta+2.0/3.0*math.pi),     math.cos(theta+2.0/3.0*math.pi),      0.0],
            [-math.sin(theta+4.0/3.0*math.pi),     math.cos(theta+4.0/3.0*math.pi),      0.0]
        ])

    # ------------------- Periodic Update ------------------- #
    def update_callback(self, event):
        # 1) Manage time
        if self.t_last is None:
            self.t_last = rospy.Time.now().to_sec()
            return
        self.t_now = rospy.Time.now().to_sec()
        self.delta_t = self.t_now - self.t_last
        self.t_last = self.t_now

        if  self.t_last_torque is None or rospy.Time.now() - self.t_last_torque > rospy.Duration(1):
            self.torque_sensed = None




        # 2) Check we have enough data
        if (self.theta is None):
            rospy.logwarn_throttle(0.1, "Waiting for  Theta")
            return
        if  (self.torque_sensed is None):
            rospy.logwarn_throttle(0.1, "Waiting for Torque")
            return
        if (self.angular_vel_wheels_now is None) or (self.wheel_angular_acceleration is None):
            rospy.logwarn_throttle(0.1, "Waiting for /joint_states data to compute wheel acceleration.")
            return

        # 3) Compute external force
        output_nominal = self.external_forces()

        # 4) Visualize
        self.visualize(output_nominal)

        # 5) Publish numeric data
        msg = Float64MultiArray()
        msg.data = output_nominal
        self.force_value_pub.publish(msg)

    # ------------------- Core Math: External Forces ------------------- #
    def external_forces(self):
        """
          a = Jcw^{-1} * (wheel_angular_accel)  +  Jcwdot_inv * (wheel_angular_vel)
        Then compute no-external-force torque, compare w/ sensed, and get Fext.
        """
        # a) body acceleration from the wheels
        self.acceleration = np.matmul(self.Jcwinv, self.wheel_angular_acceleration)  + np.matmul(self.Jcwdot_inv, self.angular_vel_wheels_now)

        #body acceleration from imu
        # self.acceleration gotten in odom_callback

        # rospy.logwarn([self.acceleration, self.acceleration])
        msg = Float64MultiArray()
        msg.data=self.acceleration
        self.pub_lina.publish(msg)
        # b) torque_no_fext: eqn(38)-like
        #    T_noFext = Jcw^T * [ M*a + Jcr^T * Br ]
        T_noFext = np.matmul(
            self.Jcwinv.T,
            self.M*self.acceleration + np.matmul(self.Jcr.T, self.Br)
        )

        # c) difference vs sensed
        #    T_sensed is 3x1
        if (self.torque_sensed is None):
            diff_torque = 0
        else:
            diff_torque = T_noFext - self.torque_sensed  # shape(3,1)
        print("diff torque: ", diff_torque)
        # d) external force in body frame
        #    F_ext_body = Jcw^T * (T_noFext - T_sensed)
        F_body = np.matmul(self.Jcw.T, diff_torque)
        Fextx = F_body[0,0]
        Fexty = F_body[1,0]
        # optionally Fz=F_body[2], but we presumably ignore

        # e) transform to local or do intersection in local
        tf_Fext = self.vector_transform([Fextx, Fexty])
        contact_pt = self.force_line_intersection(self.robot_vertices, tf_Fext)

        rospy.loginfo("Contact=(%.3f, %.3f), Fext=(%.3f, %.3f)",
                      contact_pt[0], contact_pt[1], Fextx, Fexty)
        return [contact_pt[0], contact_pt[1], Fextx, Fexty]

    def vector_transform(self, Fext):
        """
        Rotate global force -> local frame, if needed.
        """
        x_global, y_global = Fext
        cosT = math.cos(self.theta)
        sinT = math.sin(self.theta)
        # same transform you had: F_local = R(-theta)*F_global
        x_n =  cosT*x_global + sinT*y_global
        y_n = -sinT*x_global + cosT*y_global
        return [x_n, y_n]

    def force_line_intersection(self, robot_vertices, Fext):
        """
        Same triangular intersection logic, but corrected for y=... - edge_start[1].
        """
        top_left, bottom_tip, top_right = robot_vertices
        edges = [
            (top_left, bottom_tip),
            (top_left, top_right),
            (bottom_tip, top_right),
        ]

        intersections = []
        edge_flag = [False, False, False]
        contact_point = [0, 0]
        edge_count = -1

        for edge_start, edge_end in edges:
            edge_count += 1
            denom = (Fext[1]*(edge_end[0] - edge_start[0])
                     - Fext[0]*(edge_end[1] - edge_start[1]))
            if abs(denom) < 1e-12:
                continue
            s = ((Fext[0]*edge_start[1]) - (Fext[1]*edge_start[0])) / denom
            if 0 <= s <= 1:
                x = edge_start[0] + s*(edge_end[0] - edge_start[0])
                # **Important** fix: (edge_end[1] - edge_start[1]) not [0]
                y = edge_start[1] + s*(edge_end[1] - edge_start[1])
                edge_flag[edge_count] = True
                intersections.append((x, y))

        if (Fext[0] == 0 and Fext[1] == 0):
            rospy.loginfo("No intersection, zero force.")
            return [0, 0]

        elif Fext[0] != 0:
            if edge_flag[0] and edge_flag[1]:
                if Fext[1] > 0:
                    contact_point = intersections[0]
                else:
                    contact_point = intersections[1]
            elif edge_flag[0] and edge_flag[2]:
                if Fext[0] > 0:
                    contact_point = intersections[0]
                else:
                    contact_point = intersections[1]
            elif edge_flag[1] and edge_flag[2]:
                if Fext[1] > 0:
                    contact_point = intersections[0]
                else:
                    contact_point = intersections[1]
            else:
                # If exactly one intersection or none
                if len(intersections) == 1:
                    contact_point = intersections[0]
        else:
            # purely vertical Fext
            if len(intersections) == 1:
                contact_point = intersections[0]
            elif len(intersections) >= 2:
                if Fext[1] > 0:
                    contact_point = intersections[0]
                else:
                    contact_point = intersections[-1]

        return contact_point

    # ------------------- Visualization ------------------- #
    def visualize(self, output_nominal):
        contact_x, contact_y, Fextx, Fexty = output_nominal
        norm = math.hypot(Fextx, Fexty)

        # Publish the sphere marker at the contact point
        self.sphere_marker.header.stamp = rospy.Time.now()
        self.sphere_marker.pose.position.x = contact_x
        self.sphere_marker.pose.position.y = contact_y
        self.sphere_marker.pose.position.z = 0.0
        if norm >= self.threshold:
            self.pub_sphere.publish(self.sphere_marker)
        else:
                self.sphere_marker.pose.position.x = 0
                self.sphere_marker.pose.position.y = 0
                self.sphere_marker.pose.position.z = 0
                self.pub_sphere.publish(self.sphere_marker)
                self.arrow_marker.points = []
                self.pub_arrow.publish(self.arrow_marker)
                rospy.loginfo("Force magnitude below threshold, not visualizing arrow.")
                return

        arrow_length = 0.5
        start_pt = Point(
            x=contact_x,
            y=contact_y,
            z=0.0
        )
        end_pt = Point(
            x=contact_x + arrow_length*Fextx/norm,
            y=contact_y + arrow_length*Fexty/norm, 
            z=0.0
        )
        rospy.loginfo("arrow end_pt=(%.3f, %.3f)", end_pt.x, end_pt.y)
                    

        # Set arrow color (Red -> Blue gradient)
        self.arrow_marker.color.r = 0
        self.arrow_marker.color.g = 1
        self.arrow_marker.color.b = 0

        self.arrow_marker.header.stamp = rospy.Time.now()
        self.arrow_marker.points = [start_pt, end_pt]
        self.pub_arrow.publish(self.arrow_marker)

def main():
    rospy.init_node("Contact_Jacobian", anonymous=True)

    # Get params from ROS param server
    rw = rospy.get_param("~wheel_radius", 0.1)
    rr = rospy.get_param("~roller_radius", 0.00918135)
    BotMass  = rospy.get_param("~mass", 30)
    Br = rospy.get_param("~roller_damping_Br", 0.2)
    Iw = rospy.get_param("~wheel_inertia_iw", 1)
    Ir = rospy.get_param("~roller_inertia_ir", 1)
    Ib = rospy.get_param("~body_inertia_ib", 1)
    TractionTorque = rospy.get_param("~TractionTorque", 1)

    external_torque = ContactJacobian(rw, rr, BotMass, Br, Iw, Ir, Ib, TractionTorque)
    rospy.spin()

if __name__ == "__main__":
    main()
