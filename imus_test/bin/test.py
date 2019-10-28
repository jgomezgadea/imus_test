#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import Imu
from imus_test_msgs.msg import TestValue
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import statistics

# importing copy module
import copy


class Test:
    def __init__(self):

        self.px4_cube_topic_base_ = "/px_cube/mavros/imu/"
        self.px_2_4_8_topic_base_ = "/px_2_4_8/mavros/imu/"
        self.myahrs_topic_base_ = "/myahrs/imu/"
        self.yaw_arm_topic_ = "/joint_0/command"
        self.base_arm_topic_ = "/joint_1/command"
        self.joint_state_topic_ = "/joint_states"
        self.pub_yaw_arm_ = rospy.Publisher(
            self.yaw_arm_topic_, Float64, queue_size=1)
        self.pub_base_arm_ = rospy.Publisher(
            self.base_arm_topic_, Float64, queue_size=1)
        rospy.Subscriber(self.px4_cube_topic_base_+"data",
                         Imu, self.callBackOrientationPx4)
        rospy.Subscriber(self.px4_cube_topic_base_+"temperature_imu",
                         Temperature, self.callBackTemperaturePx4)
        rospy.Subscriber(self.px_2_4_8_topic_base_+"data",
                         Imu, self.callBackOrientationPx248)
        rospy.Subscriber(self.px_2_4_8_topic_base_+"temperature_imu",
                         Temperature, self.callBackTemperaturePx248)
        rospy.Subscriber(self.myahrs_topic_base_+"data",
                         Imu, self.callBackOrientationMyAhrs)
        rospy.Subscriber(self.myahrs_topic_base_+"temperature",
                         Float64, self.callBackTemperatureMyAhrs)
        rospy.Subscriber(self.joint_state_topic_,
                         JointState, self.callBackJointState)

        self.pub_yaw_arm_.publish(0.0)
        self.pub_base_arm_.publish(0.0)
        self.data_orientation_ = {
            "px4": [],
            "px_2_4_8": [],
            "myahrs": []
        }
        self.data_temperature_ = {
            "px4": [],
            "px_2_4_8": [],
            "myahrs": []
        }

        self.data_joint_state_ = {}
        self.first_joint_state = False

        self.read = False
        self.seconds = 0

    def clearData(self):
        aux_read = self.read
        self.read = False
        self.data_orientation_ = {
            "px4": [],
            "px_2_4_8": [],
            "myahrs": []
        }
        self.data_temperature_ = {
            "px4": [],
            "px_2_4_8": [],
            "myahrs": []
        }
        self.read = aux_read

    def callBackOrientationPx4(self, data):
        if self.read:
            self.data_orientation_["px4"].append(data.orientation)

    def callBackOrientationPx248(self, data):
        if self.read:
            self.data_orientation_["px_2_4_8"].append(data.orientation)

    def callBackOrientationMyAhrs(self, data):
        if self.read:
            self.data_orientation_["myahrs"].append(data.orientation)

    def callBackTemperaturePx4(self, data):
        if self.read:
            self.data_temperature_["px4"].append(data.temperature)

    def callBackTemperaturePx248(self, data):
        if self.read:
            self.data_temperature_["px_2_4_8"].append(data.temperature)

    def callBackTemperatureMyAhrs(self, data):
        if self.read:
            self.data_temperature_["myahrs"].append(data.data)

    def callBackJointState(self, data):
        for i, name in enumerate(data.name):
            self.data_joint_state_[name] = data.position[i]

    def getAverage(self):
        aux_read = self.read
        self.read = False
        data_radians = {
            "px4": [],
            "px_2_4_8": [],
            "myahrs": []
        }

        px4_min = float("inf")
        px4_max = float("-inf")
        for px4_quaternion_orientation in self.data_orientation_["px4"]:
            euler = euler_from_quaternion([px4_quaternion_orientation.x, px4_quaternion_orientation.y,
                                           px4_quaternion_orientation.z, px4_quaternion_orientation.w])
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
            data_radians["px4"].append(yaw)
            if(px4_min > yaw):
                px4_min = yaw
            if(px4_max < yaw):
                px4_max = yaw
        if(len(data_radians["px4"]) == 0):
            data_radians["px4"].append(0)

        px_2_4_8_min = float("inf")
        px_2_4_8_max = float("-inf")
        for px_2_4_8_quaternion_orientation in self.data_orientation_["px_2_4_8"]:
            euler = euler_from_quaternion([px_2_4_8_quaternion_orientation.x, px_2_4_8_quaternion_orientation.y,
                                           px_2_4_8_quaternion_orientation.z, px_2_4_8_quaternion_orientation.w])
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
            data_radians["px_2_4_8"].append(yaw)
            if(px_2_4_8_min > yaw):
                px_2_4_8_min = yaw
            if(px_2_4_8_max < yaw):
                px_2_4_8_max = yaw

        if(len(data_radians["px_2_4_8"]) == 0):
            data_radians["px_2_4_8"].append(0)

        myahrs_min = float("inf")
        myahrs_max = float("-inf")
        for myahrs_quaternion_orientation in self.data_orientation_["myahrs"]:
            euler = euler_from_quaternion([myahrs_quaternion_orientation.x, myahrs_quaternion_orientation.y,
                                           myahrs_quaternion_orientation.z, myahrs_quaternion_orientation.w])
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
            data_radians["myahrs"].append(yaw)
            if(myahrs_min > yaw):
                myahrs_min = yaw
            if(myahrs_max < yaw):
                myahrs_max = yaw
        if(len(data_radians["myahrs"]) == 0):
            data_radians["myahrs"].append(0)

        data = {
            "px4_average": statistics.mean(data_radians["px4"]),
            "px_2_4_8_average": statistics.mean(data_radians["px_2_4_8"]),
            "myahrs_average": statistics.mean(data_radians["myahrs"]),
            "px4_std": statistics.stdev(data_radians["px4"]),
            "px_2_4_8_std": statistics.stdev(data_radians["px_2_4_8"]),
            "myahrs_std": statistics.stdev(data_radians["myahrs"]),
            "px4_number_data": len(data_radians["px4"]),
            "px_2_4_8_number_data": len(data_radians["px_2_4_8"]),
            "myahrs_data": len(data_radians["myahrs"]),
            "px4_min": px4_min,
            "px4_max": px4_max,
            "px4_diff": px4_max-px4_min,
            "px_2_4_8_min": px_2_4_8_min,
            "px_2_4_8_max": px_2_4_8_max,
            "px_2_4_8_diff": px_2_4_8_max-px_2_4_8_min,
            "myahrs_min": myahrs_min,
            "myahrs_max": myahrs_max,
            "myahrs_diff": myahrs_max-myahrs_min,

        }

        self.read = aux_read
        return data

    def noiseTest(self, time=5):
        self.clearData()
        print 'Noise test, time ' + str(time)
        self.read = True
        rospy.sleep(time)
        self.read = False
        data = self.getAverage()
        print 'Result: '
        print 'Sensor\t #Data\t Average\t \t \t std\t \t min\t \t max\t \t diff \t temperature'
        print 'px cube\t ' + str(data["px4_number_data"])+"\t "+str(data["px4_average"])+"\t "+str(data["px4_std"])+"\t "+str(data["px4_min"])+"\t "+str(data["px4_max"])+"\t "+str(data["px4_diff"]) + "\t " + str(self.data_temperature_["px4"][-1])
        print 'Px 2.4\t ' + str(data["px_2_4_8_number_data"])+"\t "+str(data["px_2_4_8_average"])+"\t "+str(data["px_2_4_8_std"])+str(data["px_2_4_8_min"])+"\t "+str(data["px_2_4_8_max"])+"\t "+str(data["px_2_4_8_diff"]) + "\t " + str(self.data_temperature_["px_2_4_8"][-1])
        print 'myahrs\t ' + str(data["myahrs_data"])+"\t "+str(data["myahrs_average"])+"\t "+str(data["myahrs_std"])+str(data["myahrs_min"])+"\t "+str(data["myahrs_max"])+"\t "+str(data["myahrs_diff"]) + "\t " + str(self.data_temperature_["myahrs"][-1])
        self.clearData()

    def baseInPosition(self, position, error=0.001):
        position_error_sup = position + error
        position_error_inf = position - error
        if (self.data_joint_state_["joint_1"] < position_error_sup and self.data_joint_state_["joint_1"] > position_error_inf):
            return True
        else:
            return False

    def moveTest(self):
        print "Test in move"
        ref_px_4 = 0.0
        ref_px_2_4 = 0.0
        ref_myahrs = 0.0
        self.clearData()
        self.pub_base_arm_.publish(0.0)
        while not self.baseInPosition(0.0, 0.1):
            rospy.sleep(0.0)
        self.read = True
        rospy.sleep(1)
        self.read = False
        ref_px_4 = self.data_orientation_["px_2_4_8"][-1]
        ref_px_4 = self.data_joint_state_[
            "joint_1"] - euler_from_quaternion([ref_px_4.x, ref_px_4.y, ref_px_4.z, ref_px_4.w])[2]
        ref_px_2_4 = self.data_orientation_["px4"][-1]
        ref_px_2_4 = self.data_joint_state_["joint_1"] - euler_from_quaternion(
            [ref_px_2_4.x, ref_px_2_4.y, ref_px_2_4.z, ref_px_2_4.w])[2]
        ref_myahrs = self.data_orientation_["myahrs"][-1]
        ref_myahrs = self.data_joint_state_["joint_1"] - euler_from_quaternion(
            [ref_myahrs.x, ref_myahrs.y, ref_myahrs.z, ref_myahrs.w])[2]
        data_radians = {
            "px4": [],
            "px_2_4_8": [],
            "myahrs": [],
            "arm": []
        }
        self.pub_base_arm_.publish(3.14)
        while not self.baseInPosition(3.14, 0.1):
            self.read = True
            rospy.sleep(0.1)
            data_radians["arm"].append(self.data_joint_state_["joint_1"])
            self.read = False
            aux_px4 = self.data_orientation_["px4"][-1]
            aux_px4 = euler_from_quaternion(
                [aux_px4.x, aux_px4.y, aux_px4.z, aux_px4.w])[2] + ref_px_4
            data_radians["px4"].append(copy.deepcopy(aux_px4))
            aux_px_2_4_8 = self.data_orientation_["px_2_4_8"][-1]
            aux_px_2_4_8 = euler_from_quaternion(
                [aux_px_2_4_8.x, aux_px_2_4_8.y, aux_px_2_4_8.z, aux_px_2_4_8.w])[2] + ref_px_2_4
            data_radians["px_2_4_8"].append(copy.deepcopy(aux_px_2_4_8))
            aux_myahrs = self.data_orientation_["myahrs"][-1]
            aux_myahrs = euler_from_quaternion(
                [aux_myahrs.x, aux_myahrs.y, aux_myahrs.z, aux_myahrs.w])[2] + ref_myahrs
            data_radians["myahrs"].append(copy.deepcopy(aux_myahrs))
        self.pub_base_arm_.publish(-3.14)
        while not self.baseInPosition(-3.14, 0.1):
            self.read = True
            rospy.sleep(0.1)
            data_radians["arm"].append(self.data_joint_state_["joint_1"])
            self.read = False
            aux_px4 = self.data_orientation_["px4"][-1]
            aux_px4 = euler_from_quaternion(
                [aux_px4.x, aux_px4.y, aux_px4.z, aux_px4.w])[2] + ref_px_4
            data_radians["px4"].append(copy.deepcopy(aux_px4))
            aux_px_2_4_8 = self.data_orientation_["px_2_4_8"][-1]
            aux_px_2_4_8 = euler_from_quaternion(
                [aux_px_2_4_8.x, aux_px_2_4_8.y, aux_px_2_4_8.z, aux_px_2_4_8.w])[2] + ref_px_2_4
            data_radians["px_2_4_8"].append(copy.deepcopy(aux_px_2_4_8))
            aux_myahrs = self.data_orientation_["myahrs"][-1]
            aux_myahrs = euler_from_quaternion(
                [aux_myahrs.x, aux_myahrs.y, aux_myahrs.z, aux_myahrs.w])[2] + ref_myahrs
            data_radians["myahrs"].append(copy.deepcopy(aux_myahrs))

        px4_max_diff = float("-inf")
        px4_min_diff = float("inf")
        px_2_4_8_max_diff = float("-inf")
        px_2_4_8_min_diff = float("inf")
        myahrs_max_diff = float("-inf")
        myahrs_min_diff = float("inf")

        for i in range(len(data_radians["myahrs"])):
            data_radians["px4"][i] = data_radians["px4"][i] - \
                data_radians["arm"][i]
            if (px4_max_diff < data_radians["px4"][i]):
                px4_max_diff = data_radians["px4"][i]
            if (px4_min_diff > data_radians["px4"][i]):
                px4_min_diff = data_radians["px4"][i]
            data_radians["px_2_4_8"][i] = data_radians["px_2_4_8"][i] - \
                data_radians["arm"][i]
            if (px_2_4_8_max_diff < data_radians["px_2_4_8"][i]):
                px_2_4_8_max_diff = data_radians["px_2_4_8"][i]
            if (px_2_4_8_min_diff > data_radians["px_2_4_8"][i]):
                px_2_4_8_min_diff = data_radians["px_2_4_8"][i]
            data_radians["myahrs"][i] = data_radians["myahrs"][i] - \
                data_radians["arm"][i]
            if (myahrs_max_diff < data_radians["myahrs"][i]):
                myahrs_max_diff = data_radians["myahrs"][i]
            if (myahrs_min_diff > data_radians["myahrs"][i]):
                myahrs_min_diff = data_radians["myahrs"][i]

        data = {
            "px4_max_diff": px4_max_diff,
            "px4_min_diff": px4_min_diff,
            "px4_average": statistics.mean(data_radians["px4"]),
            "px4_std": statistics.stdev(data_radians["px4"]),
            "px_2_4_8_max_diff": px_2_4_8_max_diff,
            "px_2_4_8_min_diff": px_2_4_8_min_diff,
            "px_2_4_8_average": statistics.mean(data_radians["px_2_4_8"]),
            "px_2_4_8_std": statistics.stdev(data_radians["px_2_4_8"]),
            "myahrs_max_diff": myahrs_max_diff,
            "myahrs_min_diff": myahrs_min_diff,
            "myahrs_average": statistics.mean(data_radians["myahrs"]),
            "myahrs_std": statistics.stdev(data_radians["myahrs"])
        }

        print "Finish test, number of data: " + str(len(data_radians["myahrs"]))
        print 'Result: '
        print 'Sensor\t\tAverage\tStd\tMax diff \tMin diff'
        print 'px cube\t '+str(data["px4_average"])+"\t "+str(data["px4_std"])+"\t "+str(data["px4_min_diff"])+"\t "+str(data["px4_max_diff"])
        print 'Px 2.4\t '+str(data["px_2_4_8_average"])+"\t "+str(data["px_2_4_8_std"])+"\t "+str(data["px_2_4_8_min_diff"])+"\t "+str(data["px_2_4_8_max_diff"])
        print 'myahrs\t '+str(data["myahrs_average"])+"\t "+str(data["myahrs_std"])+"\t "+str(data["myahrs_min_diff"])+"\t "+str(data["myahrs_max_diff"])

    def executeTest(self):
        self.noiseTest()
        self.moveTest()


if __name__ == '__main__':
    rospy.init_node('test_imus', anonymous=True)
    test = Test()
    test.executeTest()
