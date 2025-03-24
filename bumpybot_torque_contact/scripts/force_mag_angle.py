#!/usr/bin/env python2
import rospy
import math
from std_msgs.msg import Float64MultiArray

def callback(msg):
    # Expecting 4 values: [x, y, fx, fy]
    if len(msg.data) != 4:
        rospy.logwarn("Expected 4 elements in the array, got %d", len(msg.data))
        return

    # Extract force values from the array
    fx = msg.data[2]
    fy = msg.data[3]

    # Compute magnitude and angle (radians)
    magnitude = math.sqrt(fx**2 + fy**2)
    angle = math.atan2(fy, fx)

    # Create a new message to publish the results.
    output = Float64MultiArray()
    output.data = [magnitude, angle]

    pub.publish(output)
    rospy.loginfo("Published force magnitude: %.3f, angle: %.3f rad", magnitude, angle)

def force_processor():
    rospy.init_node('force_processor_node', anonymous=True)
    while rospy.Time.now().to_sec() == 0:
            rospy.loginfo("Force_Mag_Angle: Waiting for /clock to start...")
            rospy.sleep(0.1)
    # Subscriber to the external force values
    rospy.Subscriber('/external_force_values', Float64MultiArray, callback)

    # Publisher for the computed magnitude and angle
    global pub
    pub = rospy.Publisher('/external_force_mag_angle', Float64MultiArray)
    rospy.spin()

if __name__ == '__main__':
    try:
        force_processor()
    except rospy.ROSInterruptException:
        pass
