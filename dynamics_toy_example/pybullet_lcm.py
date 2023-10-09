#!/usr/bin/env python3

import time
import numpy as np
import csv

import pybullet as p
import pybullet_data
from sensor_msgs.msg import JointState
from lcm_msg import lcm_JointState3, lcm_float32, lcm_float32Array
import lcm
import select
from copy import deepcopy

class pybullet_lcm:
    def __init__(self):

        self.sim_time_step = 1.0/500.0
        self.loop_rate = 500.0
        self.g = 9.8
        # self.spam_Pos = [0, 0, 0.5]
        self.spam_Pos = [0, 0, 0.265]
        self.spam_Ori_Euler = [0.0, 0.0, 0.0]
        self.origOriQuat = [0.0, 0.0, 0.0, 1.0]
        self.origPosOrt_euler = [0.0,  0.0, 0.0]
        self.counter = 0
        self.fix_robot = True

        self.joint_name_act = ['L_hip_roll_joint', 'L_hip_pitch_joint', 'L_knee_pitch_joint']
        self.joint_name_to_index_dict = {'L_hip_roll_joint':1, 'L_hip_pitch_joint':2, 'L_knee_pitch_joint':3}
        self.joint_index_to_name_dict = { 1: 'L_hip_roll_joint', 2:'L_hip_pitch_joint', 3:'L_knee_pitch_joint'}

        self.initRobot()
        self.setupEnvironment()
        self.numJoint = p.getNumJoints(self.robotId)

        self.joint_state_lcm = lcm_JointState3()
        self.joint_state_lcm.num_joint=3
        self.joint_state_lcm.num_pos = 3

        self.joint_target_lcm = lcm_JointState3()
        self.joint_state_lcm.num_joint=3
        self.joint_state_lcm.num_pos = 3

        self.init_time = time.time_ns() / (10**9)

        self.sub_time_list = list()
        self.pre_sub_time = time.time_ns()

        self.node_time_list =list()
        self.pre_node_time = time.time_ns()

        self.cur_time_list =list()
        self.pre_cur_time = time.time_ns()

        ################
        # ros publisher
        ################
        self.lc = lcm.LCM()
        self.lcm_delay_data = lcm_float32()
        sub_JointStateTarget_lcm = self.lc.subscribe('/joint_states/target/LcmCnt2Pb', self.lcm_JointStateTarget)

        #################
        # test
        #################
        self.cmd_data =[]
        self.jointStatePubTimeList = []
        self.torquePubTimeList = []

        self.torque_rec_time_list =[]
        self.torque_rec_time_list.append('/time')

        self.torque_time_LCM_list=[]
        self.torque_time_LCM_list.append('/LCM')
        self.torque_time_Delay_LCM_list=[]
        self.torque_time_Delay_LCM_list.append('/LCM_delay')
        self.torque_time_LCM_torque_list=[]
        self.torque_time_LCM_torque_list.append('/LCM_torque')

        self.first_time = 0
        self.flag_torque =False


    def lcm_JointStateTarget(self, channel, data):


        print('sub torque')

        msg = lcm_JointState3.decode(data)

        self.joint_target_lcm.name = msg.name
        self.joint_target_lcm.postiion = msg.postiion
        self.joint_target_lcm.velocity = msg.velocity
        self.joint_target_lcm.time1 = msg.time1
        self.joint_target_lcm.effort = list(msg.effort)
        self.joint_target_lcm.effort.append(0)

        cur_time = time.time_ns()
        sub_time = (cur_time - self.pre_sub_time)/(10**9)
        self.sub_time_list.append(sub_time)
        self.pre_sub_time = cur_time

        self.flag_torque = True

    def initRobot(self):
        physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setGravity(0, 0, -self.g)
        planeId = p.loadURDF("plane.urdf")

        cubeStartOrientation = p.getQuaternionFromEuler(self.spam_Ori_Euler)
        self.robotId = p.loadURDF("/home/nr_daichi/software/test_bullet_ws/src/test_bullet/test_description/urdf/bipedal_test_robot.urdf", self.spam_Pos, cubeStartOrientation,
                                # useMaximalCoordinates=1, ## New feature in Pybullet
                                flags=p.URDF_USE_INERTIA_FROM_FILE)

        # initial offset
        origPosOrt_quat = p.getBasePositionAndOrientation(self.robotId)
        self.origOriQuat = origPosOrt_quat[1]
        self.origPosOrt_euler = list(p.getEulerFromQuaternion(origPosOrt_quat[1]))

    def setupEnvironment(self):

        # set friction corn coefficients
        for link_index in range(p.getNumJoints(self.robotId)):
            info = p.getJointInfo(self.robotId, link_index)
            link_name = info[12].decode('utf-8')

        # set simulation time step
        p.setTimeStep(self.sim_time_step)

        # set simulation sub step
        p.setPhysicsEngineParameter(numSubSteps=1)

        # set simulation maximum solver iteration
        # p.setPhysicsEngineParameter(numSolverIterations=200)

        sim_setting = p.getPhysicsEngineParameters()
        print('simulation setup: ',sim_setting)

        # disable velocity control and do troque control
        numJoint = p.getNumJoints(self.robotId)
        jointIndice = [*range(numJoint)]
        p.setJointMotorControlArray(self.robotId, jointIndices=jointIndice, controlMode=p.VELOCITY_CONTROL,
                                    forces=np.zeros(numJoint))
        print('turn off velocity control')

        if self.fix_robot:
            cid = p.createConstraint(self.robotId, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0.5])
            print('fixed robot')

    def updateJointState(self):
        if p.isConnected(0):
            cur_time = (time.time_ns()/ (10 ** 9) - self.init_time)
            joint_msg = JointState()

            for joint_index in self.joint_index_to_name_dict:
                joint_state = p.getJointState(self.robotId, joint_index)
                joint_msg.name.append(self.joint_index_to_name_dict[joint_index])
                joint_msg.position.append(joint_state[0])
                joint_msg.velocity.append(joint_state[1])
                joint_msg.effort.append(joint_state[3])  # applied effort in last sim step

            self.joint_state_lcm.postiion = joint_msg.position
            self.joint_state_lcm.velocity = joint_msg.velocity
            self.joint_state_lcm.effort = joint_msg.effort
            self.joint_state_lcm.name = joint_msg.name
            self.joint_state_lcm.time1 = cur_time

    def updateJointTorque(self):
        if self.flag_torque:
            cur_time = (time.time_ns() / (10 ** 9) - self.init_time)
            self.torque_rec_time_list.append(time.time_ns() / (10 ** 9) - self.init_time)
            self.torque_time_LCM_list.append(self.joint_target_lcm.time1)
            self.torque_time_LCM_torque_list.append(self.joint_target_lcm.effort[-1])
            self.torque_time_Delay_LCM_list.append(cur_time - self.joint_target_lcm.time1)

            if p.isConnected(0):
                joint_indices = []
                effort = []

                for jname in self.joint_target_lcm.name:
                    if jname in self.joint_name_act:
                        joint_indices.append(self.joint_name_to_index_dict[jname])
                        effort.append(self.joint_target_lcm.effort[self.joint_name_to_index_dict[jname]-1])

                p.setJointMotorControlArray(self.robotId, jointIndices=joint_indices, controlMode=p.TORQUE_CONTROL,
                                            forces=effort)


    def publish(self):
        self.lc.publish('/joint_states/Pb2LcmCnt', self.joint_state_lcm.encode())

        self.lcm_gain_tune_data = lcm_float32Array()
        self.lcm_gain_tune_data.data = np.zeros(12)
        self.lcm_gain_tune_data.num_data = 12
        # self.lc.publish('/lcm_gain_tune', self.lcm_gain_tune_data.encode())

        if len(self.torque_time_Delay_LCM_list) > 2:
            self.lcm_delay_data.data = self.torque_time_Delay_LCM_list[-1]
            self.lc.publish('/lcm_delay', self.lcm_delay_data.encode())

        self.cmd_data.append(2)

    def simUpdateData(self):
        if p.isConnected(0):
            self.updateJointState()
            # self.updateJointTorque(joint_name, effort)

            self.cmd_data.append(1)

    def simUpdateDynamics(self):
        if p.isConnected(0):
            self.counter +=1
            p.stepSimulation()
            # self.ros_rate.sleep()
            # time.sleep(self.loop_rate)
            self.cmd_data.append(5)

    def ros_run(self):
        while p.isConnected(0):
            start_time = time.time_ns()/(10**9)
            self.cur_time_list.append(start_time)

            self.simUpdateData()
            self.publish()

            if not self.flag_torque:
                self.lc.handle()

            rfds, wfds, efds = select.select([self.lc.fileno()], [], [], 0)
            while rfds:
                self.lc.handle()
                rfds, wfds, efds = select.select([self.lc.fileno()], [], [], 0)

            self.updateJointTorque()
            self.simUpdateDynamics()
            mid_time = time.time_ns()/(10**9)

            if self.counter > 100:
                next_wakeUpTime = (1.0/self.loop_rate) * (self.counter-100) + self.first_time
            else:
                self.first_time = time.time_ns() / (10**9)
                next_wakeUpTime = start_time + 1.0/self.loop_rate

            left_time = next_wakeUpTime - mid_time

            if left_time>0:
                print('@@@@@@@@@@@@ sleep @@@@@@@@@@@@@@')
                time.sleep(left_time)

            end_tme = time.time_ns() / (10**9)
            node_time = (end_tme - start_time)
            self.node_time_list.append(node_time)
            print("sim node:", node_time, "frq:", 1 / node_time)

            self.flag_torque = False

        self.end_sim()


    def end_sim(self):
        self.saveCmdData()
        self.saveData()
        self.saveCmmData()
        if p.isConnected(0):
            p.disconnect()

    def saveData(self):

        header = ['/time', '/sim_time', '/sim_hz', '/sub_time', '/sub_hz']

        with open('../sim_data.csv', 'w') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            for i in range(len(self.cur_time_list)):
                try:
                    data = [self.cur_time_list[i], self.node_time_list[i], 1/self.node_time_list[i], self.sub_time_list[i], 1/self.sub_time_list[i]]
                except:
                    data = [self.cur_time_list[i], self.node_time_list[i], 1 / self.node_time_list[i], 0, 0]
                writer.writerow(data)
            print('sim data is saved')

    def saveCmdData(self):
        header = ['/time', '/sim_cmd', '/Joint_State_pub_time', '/torque_pub_time']
        with open('../simulation_cmd_data.csv', 'w') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            for i in range(len(self.cmd_data)):
                try:
                    data = [i, self.cmd_data[i], self.jointStatePubTimeList[i], self.torquePubTimeList[i]]
                except:
                    data = [i, self.cmd_data[i], 0, 0]
                writer.writerow(data)
            print('simulation command data is saved')

    def saveCmmData(self):
        with open('../cmmunication_data.csv', 'w') as f:
            writer = csv.writer(f)
            for i in range(len(self.torque_rec_time_list)):
                data = [self.torque_rec_time_list[i], self.torque_time_LCM_list[i],self.torque_time_Delay_LCM_list[i], self.torque_time_LCM_torque_list[i]]
                writer.writerow(data)
            print('cmmunication_data is saved')


if __name__ == "__main__":
    lce = pybullet_lcm()
    lce.ros_run()