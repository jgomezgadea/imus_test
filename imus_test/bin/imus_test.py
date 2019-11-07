#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import Imu
from imus_test_msgs.msg import ArrayTestValue
from imus_test_msgs.msg import TestValue
from imus_test_msgs.srv import NoiseTest, NoiseTestResponse
from imus_test_msgs.srv import MoveTest, MoveTestResponse
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from dynamixel_controllers.srv import SetSpeed
import statistics
import math
# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg


# importing copy module
import copy


class ImuData:
    def __init__(self, name_imu, topic_imu_yaw, topic_imu_temperature="not_temperature", is_the_temperature_message_float_64=False, calibration_service="", type_calibration_srv=""):
        print("Imu " + name_imu + " added")
        self.name = name_imu
        self.topic_imu_yaw = topic_imu_yaw
        self.topic_imu_temperature = topic_imu_temperature
        self.is_the_temperature_message_float_64 = is_the_temperature_message_float_64
        self.data_imu = []
        self.data_temperature = []
        self.read = False
        self.reference_yaw = 0
        self.service_proxy_calibration = None
        self.service_calibration_srv = None

        self.yaw_sub = rospy.Subscriber(
            self.topic_imu_yaw, Float64, self.imuYawCallBack)
        self.yaw_sub = None
        if(self.is_the_temperature_message_float_64):
            rospy.Subscriber(self.topic_imu_temperature, Float64,
                             self.imuTemperatureCallBack)
        else:
            rospy.Subscriber(self.topic_imu_temperature, Temperature,
                             self.imuTemperatureCallBack)
        self.calibration_service = calibration_service
        self.type_calibration_srv = type_calibration_srv
        self.has_calibration_service = False
        if(self.calibration_service != "" and self.type_calibration_srv):
            self.has_calibration_service = True
            try:
                message_pkg = self.type_calibration_srv.split('/')[0]
                self.type_calibration_srv = self.type_calibration_srv.split(
                    '/')[1]
                import_calibration = "from " + message_pkg + ".srv import " + \
                    self.type_calibration_srv + ","+self.type_calibration_srv+"Request"

                exec(import_calibration)

                create_proxy_calibration_service = "self.service_proxy_calibration = rospy.ServiceProxy(self.calibration_service," + \
                    self.type_calibration_srv + ")"
                exec(create_proxy_calibration_service)
                exec("self.service_calibration_srv = " +
                     self.type_calibration_srv+"Request()")
                print(
                    self.name + " - Added calibration")
            except Exception, e:
                rospy.logerr(
                    self.name + " - Error to try add the calibration service: " + str(e))
                self.has_calibration_service = False
                self.calibration_service = ""
                self.type_calibration_srv = ""

    def imuYawCallBack(self, data):
        if self.read:
            self.data_imu.append(data.data)

    def imuTemperatureCallBack(self, data):
        if self.read:
            if(self.is_the_temperature_message_float_64):
                self.data_temperature.append(data.data)
            else:
                self.data_temperature.append(data.temperature)

    def clearData(self):
        self.read = False
        self.data_imu = []
        self.data_temperature = []

    def getLastValue(self):
        try:
            if(len(self.data_temperature) <= 0):
                return {"yaw": self.data_imu[len(self.data_imu)-1], "temperature": 0.0}
            else:
                return {"yaw": self.data_imu[len(self.data_imu)-1], "temperature": self.data_temperature[len(self.data_temperature)-1]}
        except Exception, e:
            print(self.name + " - Error in the index: " + str(e))
            return {"yaw": 0.0, "temperature": 0.0}

    def startRead(self):
        self.read = True

    def stopRead(self):
        self.read = False

    def getStadisticData(self):
        self.read = False
        test_value = TestValue()
        test_value.device_name = self.name
        test_value.type_test = "Noise test"
        if (len(self.data_imu) == 0):
            test_value.type_test = "Not data received"
            return test_value
        test_value.average_yaw = statistics.mean(self.data_imu)
        test_value.std_yaw = statistics.stdev(self.data_imu)
        test_value.max_yaw = max(self.data_imu)
        test_value.min_yaw = min(self.data_imu)
        test_value.diff_yaw = test_value.max_yaw - test_value.min_yaw
        if (len(self.data_temperature) > 0):
            test_value.min_temperature = min(self.data_temperature)
            test_value.max_temperature = max(self.data_temperature)
            test_value.average_temperature = statistics.mean(
                self.data_temperature)
            test_value.std_temperature = statistics.stdev(
                self.data_temperature)
        return test_value

    def setReferenceYaw(self, yaw):
        self.reference_yaw = yaw

    def getReferenceYaw(self):
        return self.reference_yaw

    def calibrate(self):
        print "The imu call " + self.name + " has a calibration " + str(self.has_calibration_service)
        if(self.has_calibration_service):
            rospy.wait_for_service(self.calibration_service)
            try:
                self.service_proxy_calibration(
                    self.service_calibration_srv)
                print "%s - Send to calibrate", self.name
                return True
            except Exception, e:
                print self.name + " exception to call calibrate : " + str(e)
                return False
        return True


class Test():
    def __init__(self):
        self.imus = {}
        self.joint_roll = "/joint_2/command"
        self.join_base_command_topic = "/joint_1/command"
        self.joint_base_state = "/joint_1/state"
        self.pub_roll_arm = None
        self.pub_base_arm = None
        self.current_yaw = 0
        self.goal_yaw = 0
        self.arm_moving = False
        self.states = {
            0: "PreIdle",
            1: "Idle",
            2: "TestNoise",
            3: "TestMove",
        }
        self.STATE_PREIDLE = 0
        self.STATE_IDLE = 1
        self.STATE_TEST_NOISE = 2
        self.STATE_TEST_MOVE = 3
        self.currentState = self.STATE_PREIDLE
        self.test_noise_srv = None
        self.change_motor = False
        self.change_value_motor = False
        self.seconds_need_to_calibrate_before_test = 60

    def readParameters(self):
        if rospy.has_param('imus_test'):
            params = rospy.get_param('imus_test')
            if "test_params" in params:
                if "seconds_need_to_calibrate_before_test" in params["test_params"]:
                    self.seconds_need_to_calibrate_before_test = params[
                        "test_params"]["seconds_need_to_calibrate_before_test"]
            if "imus" in params:
                imus_param = params["imus"]
                print(imus_param)
                for imu_param in imus_param:
                    topic_temperature = "not_temperature"
                    if "topic_imu_temperature" in imu_param:
                        topic_temperature = imu_param["topic_imu_temperature"]
                    service_calibrate = ""
                    if "service_calibrate" in imu_param:
                        service_calibrate = imu_param["service_calibrate"]
                    type_calibrate_srv = ""
                    if "type_calibrate_srv" in imu_param:
                        type_calibrate_srv = imu_param["type_calibrate_srv"]
                    is_the_temperature_message_float_64 = False
                    if "is_the_temperature_message_float_64" in imu_param:
                        is_the_temperature_message_float_64 = imu_param[
                            "is_the_temperature_message_float_64"]
                    self.imus[imu_param["name"]] = ImuData(
                        imu_param["name"], imu_param["topic_imu_yaw"], topic_temperature, is_the_temperature_message_float_64, service_calibrate, type_calibrate_srv)

            if "arm" in params:
                arm = params["arm"]
                if "joint_roll" in arm:
                    self.joint_roll = arm["self.joint_roll"]
                if "join_base_command_topic" in arm:
                    self.join_base_command_topic = arm["join_base_command_topic"]
                if "joint_base_state" in arm:
                    self.joint_base_state = arm["joint_base_state"]
        self.pub_roll_arm = rospy.Publisher(
            self.joint_roll, Float64, queue_size=1)
        self.pub_base_arm = rospy.Publisher(
            self.join_base_command_topic, Float64, queue_size=1)
        self.sub_joint_state_base = rospy.Subscriber(
            self.joint_base_state, JointState, self.callBackBaseState)
        self.service_noise_test = rospy.Service(
            'noise_test', NoiseTest, self.testNoiseService)
        self.service_noise_test = rospy.Service(
            'calibrate_all_imus', NoiseTest, self.calibrate_all_imus)
        self.service_move_test = rospy.Service(
            'move_test', MoveTest, self.testMoveService)
        self.pub_moving_test_data = rospy.Publisher(
            "moving_test_data", ArrayTestValue, queue_size=1)

    def callBackBaseState(self, data):
        if(self.current_yaw != data.current_pos):
            self.change_value_motor = True
        else:
            self.change_value_motor = False
        self.current_yaw = data.current_pos
        data_without_error = data.current_pos-data.error
        if(data.is_moving):
            self.arm_moving = True
        else:
            self.arm_moving = False
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(
            0, 0, data.current_pos)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)

    def startArm(self):
        # rospy.sleep(5)
        rospy.sleep(1)
        self.change_motor = False
        connections = self.pub_roll_arm.get_num_connections()
        self.pub_roll_arm.publish(0.0)
        self.pub_base_arm.publish(0.0)
        while connections <= 0:
            connections = self.pub_roll_arm.get_num_connections()
            self.pub_roll_arm.publish(0.0)
            self.pub_base_arm.publish(0.0)
        rospy.sleep(1)
        while(self.arm_moving):
            None
        print("Moving arm to position 0,0")
        print(str(self.arm_moving))
        while(self.arm_moving == True):
            print("Moving to position")
        print("The base is in 0,0")

    def moveBaseArm(self, yaw):
        self.change_motor = False
        self.pub_base_arm.publish(yaw)
        connections = self.pub_roll_arm.get_num_connections()
        while connections <= 0:
            self.pub_base_arm.publish(yaw)
        rospy.sleep(0.5)
        print("Moving to position - "+str(yaw))
        self.goal_yaw = yaw

    def setMotorVelocity(self, value):
        # Set velocity motors:
        rospy.wait_for_service('/joint_1/set_speed')
        try:
            set_speed = rospy.ServiceProxy('/joint_1/set_speed', SetSpeed)
            set_speed_response = set_speed(value)
            return set_speed_response
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return None

    def stateLogicMachine(self):
        if (self.currentState == self.STATE_PREIDLE):
            self.statePreidle()

    def pubDataTestMoving(self, message):
        self.pub_moving_test_data.publish(message)

    def statePreidle(self):
        self.readParameters()
        self.startArm()
        self.currentState = self.STATE_IDLE

    def testNoiseService(self, req):
        response = NoiseTestResponse()
        if (self.currentState != self.STATE_IDLE):
            response.success = False
            response.message = "The node is bussy, with other test"
            return response
        self.currentState = self.STATE_TEST_NOISE
        noise = self.testNoise(req.seconds)
        print noise
        response.noise = noise
        self.currentState = self.STATE_IDLE
        response.success = True
        return response

    def calibrate_all_imus(self, req):

        response = NoiseTestResponse()
        if (self.currentState != self.STATE_IDLE):
            print("Current state: ") + str(self.currentState)
            response.success = False
            response.message = "The node is bussy, with other test"
            return response
        self.currentState = self.STATE_TEST_NOISE
        if(self.call_calibrations(req.seconds)):
            response.success = True
        else:
            response.message = "Problem to try calibrate the Imus"
            response.success = False
        self.currentState = self.STATE_IDLE
        return response

    def testNoise(self, seconds):
        self.startArm()
        self.call_calibrations(self.seconds_need_to_calibrate_before_test)
        for key_imu in self.imus:
            self.imus[key_imu].clearData()
            self.imus[key_imu].startRead()
        time_in_execution = 0
        while time_in_execution != seconds:
            print("Noise test seconds:" +
                  str(time_in_execution) + "/" + str(seconds))
            rospy.sleep(1)
            time_in_execution = time_in_execution + 1
        data = ArrayTestValue()
        for key_imu in self.imus:
            data.values.append(self.imus[key_imu].getStadisticData())
            self.imus[key_imu].clearData()
        return data

    def testMoveService(self, req):
        response = MoveTestResponse()
        if (self.currentState != self.STATE_IDLE):
            response.success = False
            response.message = "The node is bussy, with other test"
            return response
        self.currentState = self.STATE_TEST_MOVE
        self.moveBaseArm(req.init_pose)
        while(self.arm_moving == True):
            print("Moving to position")
            rospy.sleep(0.05)
        self.call_calibrations(self.seconds_need_to_calibrate_before_test)
        velocity_motor = self.setMotorVelocity(req.velocity_angular)
        if (velocity_motor == None):
            response.success = False
            response.message = "Not communication with the /joint_1/set_speed service"
            return response
        index = 0
        val = int(math.ceil(abs(req.final_pose - req.init_pose)/req.increment_pose))
        rev = 1
        if (req.final_pose - req.init_pose) < 0:
            rev = -1
        for x in range(0, req.repetition):
            for key_imu in self.imus:
                self.imus[key_imu].clearData()
                self.imus[key_imu].startRead()
            self.moveBaseArm(req.init_pose)
            while(self.arm_moving == True):
                print("Moving to init position")
                rospy.sleep(0.05)
            print("make test moving " + str(x))
            for y in range(1, val+1):
                self.moveBaseArm(req.init_pose+(req.increment_pose*y*rev))
                while(self.arm_moving == True):
                    if (self.change_value_motor == True):
                        self.send_stadists(
                            "Repetition number " + str(x) + " to " + str(req.repetition))
                        self.change_value_motor = False
                    # rospy.sleep(0.01)
                for w in range(1, 50):  # 5 seconds
                    self.send_stadists(
                        "Repetition number " + str(x) + " to " + str(req.repetition) + ", in stop")
                    rospy.sleep(0.1)

        for key_imu in self.imus:
            self.imus[key_imu].clearData()
        response.success = True
        response.message = "Finished test"
        print "Finsihed test"
        self.currentState = self.STATE_IDLE
        return response

    def send_stadists(self, type_test):
        print("sended")
        stadists = ArrayTestValue()
        arm_value = TestValue()
        arm_value.device_name = "arm value"
        arm_value.imu_value = self.current_yaw
        stadists.values.append(arm_value)
        for key_imu in self.imus:
            aux = TestValue()
            aux.device_name = key_imu
            aux.type_test = "Test moving - " + type_test
            last_value = self.imus[key_imu].getLastValue()
            aux.imu_value = last_value["yaw"]
            aux.diff_yaw = arm_value.imu_value - aux.imu_value
            aux.temperature_value = last_value["temperature"]

            stadists.values.append(aux)
        self.pubDataTestMoving(stadists)

    def call_calibrations(self, seconds):
        print("Calibration")
        time_in_execution = 0
        for key_imu in self.imus:
            if(not self.imus[key_imu].calibrate()):
                return False
        while time_in_execution != seconds:
            print("Calibration seconds:" +
                  str(time_in_execution) + "/" + str(seconds))
            rospy.sleep(1)
            time_in_execution = time_in_execution + 1
        return True


if __name__ == '__main__':
    rospy.init_node('test_imus', anonymous=True)
    test = Test()
    rate = rospy.Rate(4)  # 10hz
    while not rospy.is_shutdown():
        test.stateLogicMachine()
