#!/usr/bin/env python2
import numpy as np
import rospy 
import tf2_ros
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64MultiArray
from dynamic_reconfigure.server import Server
from bumpybot_torque_contact.cfg import JacobianConfig

import math

class ContactJacobian():
    def __init__(self, 
                 rw,   # wheel radius
                 rr,   # roller radius
                 M, # mass
                 Br,   # roller damping
                 Iw,   # wheel inertia
                 Ir,   # roller inertia
                 Ib,   # body inertia
                 TractionTorque):


        # Setup dynamic reconfigure server
        self.server = Server(JacobianConfig, self.dynamic_reconfig_callback)

        # Robot parameters
        self.rw = rw
        self.rr = rr
        self.M  = M
        self.Br = [[Br],[Br],[Br]]
        self.Iw = Iw
        self.Ir = Ir
        self.Ib = Ib
        self.TractionTorque = TractionTorque
        rospy.sleep(3) # Delay for filtered torque data to be published

        # Internal state
        self.theta     = None  # yaw
        self.velocity  = None  # [vx, vy, wz]
        self.torque_sensed = None  # from /filtered_torque_data
        self.Jcw       = None
        self.Jcwdot    = None
        self.Jcwinv   = None
        self.Jcr       = None
        self.Jcrdot    = None
        self.acceleration = None
        self.t = rospy.Time.now()  # or updated in odom callback

        # Subscribers
        rospy.Subscriber("/odom_icp_filtered", Odometry, self.odom_callback)
        rospy.Subscriber("/filtered_torque_data", JointState, self.torque_callback)

        # Timer to do periodic computations
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update_callback)

        # Marker publisher (for external force visualization)
        self.pub = rospy.Publisher("external_force", Marker,queue_size=10)
        self.force_value_pub = rospy.Publisher("external_force_values", Float64MultiArray,queue_size=10)

        self.marker = self._init_marker()
        # Publisher for wheel positions
        self.wheel_marker_pub = rospy.Publisher("wheel_positions", MarkerArray, queue_size=10)

        # ---------------------------------------------------------------------
        # If you want to get the wheel positions from TF,
        # set up a tf2 buffer/listener here:
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.robot_vertices = self.lookup_wheel_positions()
        if self.robot_vertices is None:
            rospy.logerr("Could not find wheel positions from TF, waiting 3 seconds and trying again.")
            # wait 3 seconds for TF to catch up and try once more
            rospy.sleep(3.0)
            self.robot_vertices = self.lookup_wheel_positions()
            if self.robot_vertices is None:
                rospy.logerr("Still could not find wheel positions from TF, aborting.")
                rospy.signal_shutdown("TF lookup failed")
            else:
                rospy.loginfo("Found wheel positions from TF.")
        else:
            rospy.loginfo("Found wheel positions from TF.")

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
    def _init_marker(self):
        """Initialize a visualization marker for the external force arrow."""
        marker = Marker()
        marker.header.frame_id = "base_link"  # or your robot's reference frame
        marker.header.stamp    = rospy.Time.now()
        marker.ns = "external_force"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        return marker
    def publish_wheel_markers(self, vertices):
            """
            Publishes a MarkerArray of small spheres showing each wheel position
            in the 'base_link' frame.
            """
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

                # default orientation
                mk.pose.orientation.w = 1.0

                mk.scale.x = 0.1
                mk.scale.y = 0.1
                mk.scale.z = 0.1

                mk.color.a = 1.0
                mk.color.r = 1.0  # red
                mk.color.g = 0.0
                mk.color.b = 0.0

                marker_arr.markers.append(mk)

            self.wheel_marker_pub.publish(marker_arr)
    def lookup_wheel_positions(self): # Gets the wheel positions from TF, and the center to wheel distance (R)
        """
        Uses tf2 to lookup the (x,y) positions of each wheel center 
        relative to the robot base.  
        """
        wheel_frames = ["wheel0", "wheel1", "wheel2"]
        base_frame   = "base_link"
        vertices = []
        for wheel_frame in wheel_frames:
            try:
                trans = self.tf_buffer.lookup_transform(
                    base_frame, wheel_frame, rospy.Time(0), rospy.Duration(1.0)
                )
            except:
                rospy.logerr("Could not lookup transform to {}, is /tf being published correctly?".format(wheel_frame))
            else:
                x = trans.transform.translation.x
                y = trans.transform.translation.y
                vertices.append((x,y))
                # get the distance from the center of the robot to the wheel (they are symmetric)
                self.R = np.sqrt(x**2 + y**2)
                self.publish_wheel_markers(vertices)
        return vertices
        

    # ------------------- ROS Callbacks ------------------- #
    def odom_callback(self, odom_msg):
        # Extract yaw from odometry quaternion
        q = odom_msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.theta = yaw

        # Build velocity vector
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y
        wz = odom_msg.twist.twist.angular.z
        self.velocity = np.array([[vx], [vy], [wz]])

        # Build Jcw, etc. if we have a valid yaw
        if self.theta is not None:
            self.build_jacobians(self.theta)
        else:
            rospy.logwarn("No valid yaw yet, waiting for odom data...")

        # Update the current time (if needed in your math)
        self.t = odom_msg.header.stamp

    def torque_callback(self, joint_msg):
        """
        Assumes /filtered_torque_data is sensor_msgs/JointState 
        and that 'position' is your torque reading for each wheel.
        """
        self.torque_sensed = joint_msg.position 
        # Expecting an array of 3 (for 3 wheels)

    # ------------------- Jacobian Builders ------------------- #
    def build_jacobians(self, theta):
        # Jcw in eqn 16 (one common formula for 3-wheel omni)
        self.Jcw = (1.0 / self.rw) * np.array([
            [-np.sin(theta),                   np.cos(theta),                   self.R],
            [-np.sin(theta + (2.0/3.0)*np.pi), np.cos(theta + (2.0/3.0)*np.pi), self.R],
            [-np.sin(theta + (4.0/3.0)*np.pi), np.cos(theta + (4.0/3.0)*np.pi), self.R]
        ])
    
        self.Jcwdot = (1.0 / self.rw) * np.array([
            [-np.cos(theta),                   -np.sin(theta),                   0.0],
            [-np.cos(theta + (2.0/3.0)*np.pi), -np.sin(theta + (2.0/3.0)*np.pi), 0.0],
            [-np.cos(theta + (4.0/3.0)*np.pi), -np.sin(theta + (4.0/3.0)*np.pi), 0.0]
        ])
    
        self.Jcwinv = np.linalg.inv(self.Jcw)
        self.Jcwdot_inv = np.linalg.pinv(self.Jcwdot)
    
        # Similarly for Jcr, Jcrdot if you need them:
        self.Jcr = (1.0 / self.rr) * np.array([
            [ np.cos(theta),                   np.sin(theta),                   0],
            [ np.cos(theta + (2.0/3.0)*np.pi), np.sin(theta + (2.0/3.0)*np.pi), 0],
            [ np.cos(theta + (4.0/3.0)*np.pi), np.sin(theta + (4.0/3.0)*np.pi), 0]
        ])
        self.Jcrdot = (1.0 / self.rr) * np.array([
            [-np.sin(theta),                   np.cos(theta),                   0],
            [-np.sin(theta + (2.0/3.0)*np.pi), np.cos(theta + (2.0/3.0)*np.pi), 0],
            [-np.sin(theta + (4.0/3.0)*np.pi), np.cos(theta + (4.0/3.0)*np.pi), 0]
        ])


    # ------------------- Periodic Update ------------------- #
    def update_callback(self, event):
        # Always re-publish the wheel markers
        # if self.robot_vertices:
            # self.publish_wheel_markers(self.robot_vertices)
        # Need a valid orientation, velocity, and torque to proceed:
        if self.theta is None or self.velocity is None or self.torque_sensed is None:
            rospy.logwarn_throttle(2.0, "Waiting for odom and/or torque data...")
            return

        output_nominal = self.external_forces()
        self.visualize(output_nominal)
                # Publish numeric data
        msg = Float64MultiArray()
        msg.data = output_nominal
        self.force_value_pub.publish(msg)


    # ------------------- Original Math Logic ------------------- #
    def NominalTorque(self):    
        pass

    def torque_no_fext(self):
        """
        Return the 'no external force' torque, eqn. 38, etc.
        Adjust as needed to match your actual references.
        """
        # Ensure acceleration is initialized
        if self.acceleration is None:
            self.acceleration = np.zeros((3, 1))
        
        # Ensure Br is a column vector
        Br = np.array(self.Br).reshape(3, 1)
        
        # Ensure Jcr is a 2D array
        Jcr = np.array(self.Jcr)
        
        return np.matmul(
            np.transpose(self.Jcwinv),
            (self.M * self.acceleration) + np.matmul(np.transpose(Jcr), Br)
        )

    def external_forces(self):
        """
        Evaluate external force from difference between measured torque
        and nominal torque, then find intersection, etc.
        """
        # 1) Compute nominal torque
        self.NominalTorque()

        # 2) torque_no_fext
        T_ext_not = self.torque_no_fext()

        # 3) Sensed torque is self.torque_sensed, e.g. shape (3,)
        #    Make sure shapes match. Example: reshape to (3,1)
        T_sensed = np.array(self.torque_sensed).reshape(3,1)

        # 4) Force difference
        RH_Matrix = np.matmul(np.transpose(self.Jcw), (T_ext_not - T_sensed))
        self.Fextx = RH_Matrix[0][0]
        self.Fexty = RH_Matrix[1][0]
        Fext = [self.Fextx, self.Fexty]
        # 5) Transform or find intersection
        tf_Fext = self.vector_transform(Fext)
        intersection = self.force_line_intersection(self.robot_vertices, tf_Fext)

        rospy.loginfo("Contact point=({}, {}), Fext=({}, {})".format(intersection[0], intersection[1], self.Fextx, self.Fexty))
        return [intersection[0], intersection[1], self.Fextx, self.Fexty]

    def vector_transform(self, Fext):
        """Transform global force to local frame, if needed."""
        x_n = np.cos(self.theta)*Fext[0] + np.sin(self.theta)*Fext[1]
        y_n = -np.sin(self.theta)*Fext[0] + np.cos(self.theta)*Fext[1]
        return [x_n, y_n]

    def force_line_intersection(self, robot_vertices, Fext):
        """ 
        Solves parametric parameter s separately for two lines that it intersects
        Finds the first intersection of the force vector
        Need to transform the force vector to a local frame where the centroid of the robot body is (0,0)
        *only rotational motion is considered

        Parameters: 
        self: instance of class
        robot_vertices: the vertices of the robot

        Return:
        contact_point: The point where the external force first intersects the robot
        """
        top_left, bottom_tip, top_right = robot_vertices[0], robot_vertices[1], robot_vertices[2]

        # Edges of the triangle
        edges = [(top_left, bottom_tip), (top_left, top_right), (bottom_tip, top_right)]

        intersections = [] #parametric parameter along the edge. s in order will be point one edge 1, 2, then 3
        edge_flag = [False, False, False]
        edge_count = -1
        contact_point = [0, 0]
        # Loop through each edge of the triangle
        #Centroid on the local frame is (0,0)
        for edge_start, edge_end in edges: #checks for edge 1 to edge 2 to edge 3
            edge_count += 1
            # s = (Fext[0]*(edge_start[1] - 0) - Fext[1]*(edge_start[0] - 0))/(Fext[1]*(edge_end[0] - edge_start[0]) - Fext[0]*(edge_end[1] - edge_end[1]))
            # Calculate the denominator correctly
            denom = (Fext[1] * (edge_end[0] - edge_start[0])
                    - Fext[0] * (edge_end[1] - edge_start[1]))
            
            # Avoid division by zero if denom == 0
            if abs(denom) < 1e-12:
                continue

            # Parametric "s" for the edge
            s = ((Fext[0] * edge_start[1]) - (Fext[1] * edge_start[0])) / denom
            if s >= 0 and s <= 1:
            
                x = edge_start[0] + s * (edge_end[0] - edge_start[0])
                y = edge_start[1] + s * (edge_end[1] - edge_start[0])
                edge_flag[edge_count] = True
                intersections.append((x, y))
        if Fext[0] == 0 and Fext[1] == 0:
            rospy.loginfo("No intersection")
        #The stuff for first contact point    

        elif Fext[0] != 0:
            if edge_flag[0] == True and edge_flag[1] == True:
                if Fext[1] > 0:
                    contact_point = intersections[0]
                else:
                    contact_point = intersections[1]
            if edge_flag[0] == True and edge_flag[2] ==True:
                if Fext[0] > 0:
                    contact_point = intersections[0]
                else:
                    contact_point[1] = intersections[1]

            if edge_flag[1] == True and edge_flag[2] ==True:
                if Fext[1] > 0:
                    contact_point = intersections[0]
                else:
                    contact_point = intersections[1]
            else: #this is when m = 0, a horizontal line
                if Fext[0] > 0:
                    contact_point = intersections[0]

                else:
                    contact_point = intersections[1]
        else: # slope in x is 0, vertical line
            if Fext[1] > 0: 
                if edge_flag[0] == True and edge_flag[1] == True: # edge1 and edge2
                    contact_point = intersections[1]
                else:
                    contact_point = intersections[0]
            else: # edge2 awnd edge 3
                 contact_point = intersections[0]
        rospy.loginfo("Contact point=({}, {})".format(contact_point[0], contact_point[1]))
        return contact_point


    # ------------------- Visualization ------------------- #
    def visualize(self, output_nominal):
        contact_x, contact_y, Fextx, Fexty = output_nominal

        # Build arrow from contact point outward (or vice versa)
        # Make sure your marker frame_id aligns with how you interpret contact_x,y
        norm = np.hypot(Fextx, Fexty)
        if norm < 1e-9:
            norm = 1e-9
        ux, uy = Fextx/norm, Fexty/norm

        # Arrow length
        arrow_length = 0.5
        start_pt = Point(
            x=contact_x + arrow_length * ux,
            y=contact_y + arrow_length * uy,
            z=0.0
        )
        end_pt = Point(x=contact_x, y=contact_y, z=0.0)
        
        # Update color as a function of magnitude
        if not hasattr(self, "force_max"):
            self.force_max = norm
        alpha = 0.1
        self.force_max = max(self.force_max*(1-alpha) + norm*alpha, norm, 1e-3)
        ratio = min(norm / self.force_max, 1.0)
        self.marker.color.r = ratio
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0 - ratio

        self.marker.header.stamp = rospy.Time.now()
        self.marker.points = [start_pt, end_pt]
        self.pub.publish(self.marker)

def main():
    # Init ROS node
    rospy.init_node("Contact_Jacobian", anonymous=True)

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
