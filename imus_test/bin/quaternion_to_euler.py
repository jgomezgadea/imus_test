#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def get_rotation(msg):
    global roll, pitch, yaw
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    return yaw

'''
def px_cube(msg):
    msg_pub = Float64()
    msg_pub.data = get_rotation(msg)
    pub_px_cube.publish(msg_pub)


def px_2_4_8(msg):
    msg_pub = Float64()
    msg_pub.data = get_rotation(msg)
    pub_px_248.publish(msg_pub)


def myahrs(msg):
    msg_pub = Float64()
    msg_pub.data = get_rotation(msg)
    pub_myarhm.publish(msg_pub)




sub_px_cube = rospy.Subscriber('/px_cube/mavros/imu/data', Imu, px_cube)
sub_px_248 = rospy.Subscriber('/px_2_4_8/mavros/imu/data', Imu, px_2_4_8)
sub_myarhm = rospy.Subscriber('/myahrs/imu/data', Imu, myahrs)
pub_px_cube = rospy.Publisher('/px_cube/yaw', Float64, queue_size=1)
pub_px_248 = rospy.Publisher('/px_2_4_8/yaw', Float64, queue_size=1)
pub_myarhm = rospy.Publisher('/myahrs/yaw', Float64, queue_size=1)

'''

rospy.init_node('my_quaternion_to_euler')
print ("-------------------------------------------------")
sub = {}
pub = {}
imus = []
imus = rospy.get_param('quaternion_to_euler_imus')
for imu in imus:
    print imu
    function = "\n\
def "+imu["name"]+"_cb(msg):\n\
\tmsg_pub = Float64()\n\
\tmsg_pub.data = get_rotation(msg)\n\
\tpub['"+imu['name']+"'].publish(msg_pub)"
    exec(function)
    sub[imu["name"]] = rospy.Subscriber(imu["topic"], Imu, locals()[imu["name"]+"_cb"])
    pub[imu["name"]] = rospy.Publisher(imu["publish"], Float64, queue_size=1)
    print imu["name"] + " " + imu["topic"] + " " + imu["publish"]

r = rospy.Rate(10)
while not rospy.is_shutdown():

    # get a parameter from our parent namespace

    r.sleep()
