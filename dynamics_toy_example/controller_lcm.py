#!/usr/bin/env python3
import csv
import time
import numpy as np
import select
import pybullet as p

from sensor_msgs.msg import JointState, Imu
# from test_controller.scripts.rosDebugger import rosDebugger

from lcm_msg import lcm_JointState3, lcm_float32, lcm_float32Array
import lcm

class test_controller:
    def __init__(self):

        self.end_effector_link_names=['L_foot_pitch', 'R_foot_pitch']
        self.loop_rate = 500.0
        self.cntrl_frq = 500
        self.g = 9.8

        self.counter = 0
        self.t = 0

        self.num_joint = 3

        self.joint_name = ['L_hip_roll_joint', 'L_hip_pitch_joint', 'L_knee_pitch_joint']

        self.kp = np.array([200, 200, 200])
        self.kd = np.array([27, 24, 28])  # 99, 20, 20
        self.target_joint_pos = np.zeros(self.num_joint)
        self.effort_lcm = np.zeros(self.num_joint)

        self.states = dict()
        self.states['vel'] = np.zeros(3)
        self.states['pos'] = np.zeros(3)

        # self.ros_debugger = rosDebugger()
        self.lcm_gain_tune_data = np.zeros(9)

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
        sub_JointState = self.lc.subscribe('/joint_states/Pb2LcmCnt', self.sub_lcm_JointState)
        sub_lcm_gain = self.lc.subscribe('/lcm_gain_tune', self.sub_lcm_gain_tune)

        self.joint_state_LCM = lcm_JointState3()
        self.joint_state_LCM.num_joint = 3
        self.joint_state_LCM.num_pos = 3
        self.joint_state_LCM.postiion = np.zeros(3)
        self.joint_state_LCM.velocity = np.zeros(3)
        self.joint_state_LCM.name = ["","",""]
        self.joint_state_LCM.effort = np.zeros(3)

        self.lcm_JointStateTarget = lcm_JointState3()
        self.lcm_JointStateTarget.num_joint = 3
        self.lcm_JointStateTarget.num_pos = 3
        self.lcm_JointStateTarget.postiion = np.zeros(3)
        self.lcm_JointStateTarget.velocity = np.zeros(3)
        self.lcm_JointStateTarget.name = ["","",""]
        self.lcm_JointStateTarget.effort = np.zeros(3)

        self.joint_state_deg_LcmCnt2Pj = lcm_JointState3()
        self.joint_state_deg_LcmCnt2Pj.num_joint = 3
        self.joint_state_deg_LcmCnt2Pj.num_pos = 3
        self.joint_state_deg_LcmCnt2Pj.postiion = np.zeros(3)
        self.joint_state_deg_LcmCnt2Pj.velocity = np.zeros(3)
        self.joint_state_deg_LcmCnt2Pj.name = ["","",""]
        self.joint_state_deg_LcmCnt2Pj.effort = np.zeros(3)

        self.joint_state_error_deg_LcmCnt2Pj = lcm_JointState3()
        self.joint_state_error_deg_LcmCnt2Pj.num_joint = 3
        self.joint_state_error_deg_LcmCnt2Pj.num_pos = 3
        self.joint_state_error_deg_LcmCnt2Pj.postiion = np.zeros(3)
        self.joint_state_error_deg_LcmCnt2Pj.velocity = np.zeros(3)
        self.joint_state_error_deg_LcmCnt2Pj.name = ["","",""]
        self.joint_state_error_deg_LcmCnt2Pj.effort = np.zeros(3)

        ##############
        # test
        ##############
        self.cmd_rec = []
        self.js_renew = False

        self.counter = 0
        self.first_time=0

    def sub_lcm_gain_tune(self, channel, data):
        msg = lcm_float32Array.decode(data)
        self.lcm_gain_tune_data = msg.data
        print("sub_gain")

    def sub_lcm_JointState(self, channel, data):
        print("sub joint states")
        msg = lcm_JointState3.decode(data)

        self.joint_state_LCM.postiion = msg.postiion
        self.joint_state_LCM.velocity = msg.velocity
        self.joint_state_LCM.name = msg.name
        self.joint_state_LCM.time1 = msg.time1

        cur_time = time.time_ns()
        sub_time = (cur_time - self.pre_sub_time)/(10**9)
        self.sub_time_list.append(sub_time)
        self.pre_sub_time = cur_time

        self.js_renew = True
        self.cmd_rec.append(1)

    def updateCounter(self):
        self.counter += 1
        self.t = self.counter * 1/self.cntrl_frq

    def updateJointState(self):

        self.joint_state_deg_LcmCnt2Pj.name = self.joint_state_LCM.name
        self.joint_state_deg_LcmCnt2Pj.postiion = np.rad2deg(self.joint_state_LCM.postiion)
        self.joint_state_deg_LcmCnt2Pj.velocity = self.joint_state_LCM.velocity

        self.joint_state_error_deg_LcmCnt2Pj.name = self.joint_state_LCM.name
        self.joint_state_error_deg_LcmCnt2Pj.postiion = np.rad2deg(self.joint_state_LCM.postiion) - np.rad2deg(self.target_joint_pos)

        self.states['vel'] = self.joint_state_LCM.velocity
        self.states['pos'] = self.joint_state_LCM.postiion

        self.cmd_rec.append(1)

    def setTarget(self):
        for i in range(self.num_joint):
            self.target_joint_pos[i] = np.deg2rad(self.lcm_gain_tune_data[i])
            pass
        self.cmd_rec.append(2)

    def setTargetSin(self):
        self.target_joint_pos[0] = 0
        self.target_joint_pos[1] = np.deg2rad(30) * np.sin(2*np.pi * self.t)
        self.target_joint_pos[2] = np.deg2rad(30) * np.sin(2*np.pi * self.t)
        self.cmd_rec.append(2)

    def control(self):
        for i in range(self.num_joint):
            self.kp[i] = self.lcm_gain_tune_data[i+3]
            self.kd[i] = self.lcm_gain_tune_data[i + 6]
            try:
                self.effort_lcm[i] = self.kp[i] * (self.target_joint_pos[i] - self.states['pos'][i]) + self.kd[i] * (0 - self.states['vel'][i])
            except:
                pass

        self.cmd_rec.append(3)

    def publish(self):
        self.lcm_JointStateTarget.time2 = time.time_ns() / (10 ** 9)
        self.lcm_JointStateTarget.time1 = self.joint_state_LCM.time1
        self.lcm_JointStateTarget.name = self.joint_name.copy()
        self.lcm_JointStateTarget.effort = self.effort_lcm.tolist()
        self.lcm_JointStateTarget.postiion = np.rad2deg(self.target_joint_pos).tolist()
        self.lcm_JointStateTarget.velocity = np.zeros(3)
        self.lc.publish('/joint_states/target/LcmCnt2Pb',self.lcm_JointStateTarget.encode())
        self.lc.publish('/joint_states_deg/LcmCnt2Pb', self.joint_state_deg_LcmCnt2Pj.encode())
        self.lc.publish('/joint_states_deg_error/LcmCnt2Pb', self.joint_state_error_deg_LcmCnt2Pj.encode())

        self.cmd_rec.append(4)

    def saveData(self):

        header = ['/time', '/control_time', '/control_hz', '/sub_time', '/sub_hz']

        with open('../controller_data.csv', 'w') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            for i in range(len(self.cur_time_list)):
                try:
                    data = [self.cur_time_list[i], self.node_time_list[i], 1/self.node_time_list[i], self.sub_time_list[i], 1/self.sub_time_list[i]]
                except:
                    data = [self.cur_time_list[i], self.node_time_list[i], 1 / self.node_time_list[i], 0, 0]
                writer.writerow(data)
            print('control data is saved')

    def saveCmdData(self):
        header = ['/time', '/control_cmd']
        with open('../controller_cmd_data.csv', 'w') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            for i in range(len(self.cmd_rec)):
                data = [i, self.cmd_rec[i]]
                writer.writerow(data)
            print('control command data is saved')

    def ros_run(self):
        while True:
            try:
                start_time = time.time_ns()/(10**9)

                self.updateCounter()
                self.lc.handle()

                # insure that the joint state is updated
                while not self.js_renew:
                    print(" 888888888  js renews 00000000000")
                    self.lc.handle()
                self.js_renew = False

                # if there is msg in bufffer, read them (insure that no messsage is stucked in buffer)
                rfds, wfds, efds = select.select([self.lc.fileno()], [], [], 0)
                while rfds:
                    self.lc.handle()
                    rfds, wfds, efds = select.select([self.lc.fileno()], [], [], 0)


                self.updateJointState()
                self.setTargetSin()
                self.control()
                self.publish()
                mid_time = time.time_ns() / (10**9)


                if self.counter > 100:
                    next_wakeUpTime = (1.0 / self.loop_rate) * (self.counter - 100) + self.first_time
                else:
                    self.first_time = time.time_ns() / (10 ** 9)
                    next_wakeUpTime = start_time + 1.0 / self.loop_rate

                left_time = next_wakeUpTime - mid_time

                if left_time>0:
                    time.sleep(left_time)

                cur_time = time.time_ns()
                self.cur_time_list.append(cur_time/(10**9))
                node_time = (cur_time - self.pre_node_time)/(10**9)
                print("contrller node:", node_time, "frq:", 1/node_time)
                self.node_time_list.append(node_time)
                self.pre_node_time = cur_time

            except KeyboardInterrupt:
                break

        self.saveData()
        self.saveCmdData()


if __name__ == "__main__":
    lce = test_controller()
    lce.ros_run()
