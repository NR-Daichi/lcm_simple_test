import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import JointState
from lcm_msg import lcm_float32Array, lcm_float32, lcm_JointState3
import lcm
class rosDebuggerLcm:
    def __init__(self):
        rospy.init_node('rosdebugger', anonymous=True)
        self.rate = rospy.Rate(250)

        self.ros_pub = rospy.Publisher('rosDebugger', Float32MultiArray, queue_size=1)
        self.pub_LCM_Delay_Pb2Pj = rospy.Publisher('/Lcm_delay', Float32, queue_size=1)
        self.ros_pub_joint_state_deg = rospy.Publisher('/joint_states_deg/LcmCnt2Pb', JointState, queue_size=1)
        self.ros_pub_joint_state_error = rospy.Publisher('/joint_states_error/LcmCnt2Pb', JointState, queue_size=1)
        self.ros_time_delay = Float32()
        self.joint_state_deg = JointState()
        self.joint_state_error = JointState()

        self.lcm_gain_tune_data = lcm_float32Array()
        self.lcm_gain_tune_data.num_data = 15
        self.lcm_gain_tune_data.data = np.zeros(15)

        self.gain_tune_data = np.zeros(15)
        rospy.Subscriber("/gain_tune", Float32MultiArray, self.gain_tune_cb,queue_size=1)

        self.lc = lcm.LCM()
        sub_JointStateTarget_lcm = self.lc.subscribe('/lcm_delay', self.lcm_DelayTarget)
        sub_JointStateDeg_lcm = self.lc.subscribe('/joint_states_deg/LcmCnt2Pb', self.lcm_JointStateDeg)
        sub_JointStateDegError_lcm = self.lc.subscribe('/joint_states_deg_error/LcmCnt2Pb', self.lcm_JointStateDegError)
        sub_JointState = self.lc.subscribe('/lcmDebugger', self.sub_lcm_JointState)
        self.counter=0

    def gain_tune_cb(self,data: Float32MultiArray):
        # self.lcm_gain_tune_data.num_data = len(data.data)
        for i in range(len(data.data)):
            self.lcm_gain_tune_data.data[i] = data.data[i]
        # self.lc.publish('/lcm_gain_tune', self.lcm_gain_tune_data.encode())

    def lcm_DelayTarget(self,channel, data):
        msg = lcm_float32.decode(data)
        self.ros_time_delay.data = msg.data
        self.counter +=1
        print("subscribe lcm_delay", self.counter)

    def lcm_JointStateDeg(self,channel, data):
        msg = lcm_JointState3.decode(data)
        self.joint_state_deg.name = msg.name
        self.joint_state_deg.position = msg.postiion
        self.joint_state_deg.velocity = msg.velocity



    def lcm_JointStateDegError(self, channel, data):
        msg = lcm_JointState3.decode(data)
        self.joint_state_error.name = msg.name
        self.joint_state_error.position = msg.postiion
        self.joint_state_error.velocity = msg.velocity

    def publish(self):
        self.pub_LCM_Delay_Pb2Pj.publish(self.ros_time_delay)
        self.ros_pub_joint_state_deg.publish(self.joint_state_deg)
        self.ros_pub_joint_state_error.publish(self.joint_state_error)
        # self.lc.publish('/lcm_gain_tune', self.lcm_gain_tune_data.encode())

if __name__ == "__main__":
    lce = rosDebuggerLcm()
    while True:
        try:
            lce.lc.handle_timeout(0)
            lce.publish()
            lce.rate.sleep()
        except KeyboardInterrupt:
            break

    # def publish_data(self,data):
    #     # self.data.layout.dim = data.size()
    #     self.data.data=data.copy()
    #     self.pub.publish(self.data)



