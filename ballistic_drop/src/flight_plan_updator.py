# Import necessary libraries
import rospy
from mavros_msgs.msg import GlobalPositionTarget, PositionTarget
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3

# Initialize node
rospy.init_node('flight_plan_update')

# Set up publishers for target location and approach vector
target_pub = rospy.Publisher('/mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)
vector_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

# Set up callback for receiving GPS location updates
def gps_callback(data):
    # Get latitude, longitude, and altitude from GPS data
    lat = data.latitude
    lon = data.longitude
    alt = data.altitude

    # Create GlobalPositionTarget message with target location
    target_msg = GlobalPositionTarget()
    target_msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
    target_msg.type_mask = GlobalPositionTarget.IGNORE_VX + GlobalPositionTarget.IGNORE_VY + GlobalPositionTarget.IGNORE_VZ + GlobalPositionTarget.IGNORE_AFX + GlobalPositionTarget.IGNORE_AFY + GlobalPositionTarget.IGNORE_AFZ + GlobalPositionTarget.FORCE + GlobalPositionTarget.IGNORE_YAW + GlobalPositionTarget.IGNORE_YAW_RATE
    target_msg.latitude = lat
    target_msg.longitude = lon
    target_msg.altitude = alt

    # Publish target location message
    target_pub.publish(target_msg)

# Set up callback for receiving approach vector updates
def vector_callback(data):
    # Get approach vector and speed from data
    x = data.x
    y = data.y
    z = data.z
    s = data.speed

    # Create PositionTarget message with approach vector and speed
    vector_msg = PositionTarget()
    vector_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    vector_msg.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE
    vector_msg.position = Vector3(x, y, z)
    vector_msg.velocity = Vector3(s, s, s)

    # Publish approach vector message
    vector_pub.publish(vector_msg)

# Set up subscriber for receiving GPS location updates
rospy.Subscriber('/mavros/global_position/global', NavSatFix, gps_callback)

# Set up subscriber for receiving approach vector updates
rospy.Subscriber('/approach_vector', Vector3, vector_callback)

# Spin ROS node
rospy.spin()
