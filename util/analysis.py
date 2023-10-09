import csv
import numpy as np
import matplotlib.pyplot as plt

class anays_py_comm():
    def __init__(self):

        output = list()
        # with open("../scripts/controller_data.csv", 'r') as file:
        # with open("../../controller_data.csv", 'r') as file:
        with open("../../sim_data.csv", 'r') as file:
        # with open("../scripts/sim_frq_data_log.csv", 'r') as file:
          test = np.asarray(list(csv.reader(file)))
          test2 = test [2::, :]
          self.test3= test2.astype('float')

    def getStaticsProHz(self):

        data_cntrl_dt = list()
        for i in range(len(self.test3[:,2])):
            if self.test3[i,2] != 0:
                data_cntrl_dt.append(self.test3[i,2])
        data_cntrl_dt = np.asarray(data_cntrl_dt)
        self.data_cntrl_hz = data_cntrl_dt

        num_sample = self.data_cntrl_hz.shape[0]

        data_cntrl_HZ_mean = np.mean(self.data_cntrl_hz)
        data_cntrl_HZ_min = np.min(self.data_cntrl_hz)
        data_cntrl_HZ_max = np.max(self.data_cntrl_hz)

        data_90_pe = np.percentile(self.data_cntrl_hz, 90)
        data_10_pe = np.percentile(self.data_cntrl_hz, 10)

        data_95_pe = np.percentile(self.data_cntrl_hz, 95)
        data_5_pe = np.percentile(self.data_cntrl_hz, 5)

        data_50_pe = np.percentile(self.data_cntrl_hz, 50)

        print("#####################")
        print("node statistic")
        print("#####################")
        print("num_sample:",num_sample, "mean: {:.2f}".format(data_cntrl_HZ_mean), 'min: {:.2f}'.format(data_cntrl_HZ_min), 'max {:.2f}'.format(data_cntrl_HZ_max))
        print("95th: {:.2f}".format(data_95_pe), '5th: {:.2f}'.format(data_5_pe))
        print("90th: {:.2f}".format(data_90_pe), '10th: {:.2f}'.format(data_10_pe))
        print("50th: {:.2f}".format(data_50_pe))

    def getStaticsNodeHz(self):

        data_dt = list()
        for i in range(len(self.test3[:,4])):
            if self.test3[i,4] != 0:
                data_dt.append(self.test3[i,4])
        data_dt = np.asarray(data_dt)

        self.data_sim_hz = data_dt
        num_sample = self.data_sim_hz.shape[0]

        data_hz_mean = np.mean(self.data_sim_hz)
        data_hz_min = np.min(self.data_sim_hz)
        data_hz_max = np.max(self.data_sim_hz)

        data_90_pe = np.percentile(self.data_sim_hz, 90)
        data_10_pe = np.percentile(self.data_sim_hz, 10)

        data_95_pe = np.percentile(self.data_sim_hz, 95)
        data_5_pe = np.percentile(self.data_sim_hz, 5)

        data_50_pe = np.percentile(self.data_sim_hz, 50)

        print("#####################")
        print("sub statistic")
        print("#####################")
        print("num_sample:",num_sample, "mean: {:.2f}".format(data_hz_mean), 'min: {:.2f}'.format(data_hz_min), 'max {:.2f}'.format(data_hz_max))
        print("95th: {:.2f}".format(data_95_pe), '5th: {:.2f}'.format(data_5_pe))
        print("90th: {:.2f}".format(data_90_pe), '10th: {:.2f}'.format(data_10_pe))
        print("50th: {:.2f}".format(data_50_pe))

    def getCommDropRcv(self):
        data_sig_float = self.test3[1::, 3]
        data_sig_int = data_sig_float.astype('int')

        counter = 0
        for i in range(len(data_sig_int)-1):
            if data_sig_int[i+1] - data_sig_int[i] == 1:
                pass

            else:
                counter += 1
                # print("i:",i,"data_sig[i+1]:",data_sig_int[i+1], "data_sig[i]:",data_sig_int[i])

        print("total signal drops num: ",counter, " percent:{:.4f}".format(counter/len(data_sig_int-1) * 100 ),' %')

    def getCommDropSndRcv(self):

        data_trq_sig_float = list()
        for i in range(len(self.test3[:,5])):
            if self.test3[i,5] != 0:
                data_trq_sig_float.append(self.test3[i,5])

        data_trq_sig_float = np.asarray(data_trq_sig_float)
        data_trq_sig_int = data_trq_sig_float.astype('int')

        counter = 0
        for i in range(len(data_trq_sig_int)-1):
            if data_trq_sig_int[i+1] - data_trq_sig_int[i] == 1:
                pass
            else:
                counter += 1
                # print("i:",i,"data_trq_sig[i+1]:",data_trq_sig_int[i+1], "data_trq_sig[i]:",data_trq_sig_int[i])

        print("total signal drops in pub torque num: ",counter, " percent:{:.4f}".format(counter/len(data_trq_sig_int-1) * 100 ),' %')

    def drawGraph(self):

        plt.figure(1)
        plt.plot(self.data_sim_hz)
        plt.ylabel('Communication Frequency [Hz]')
        plt.title("simulation Frequency")

        plt.figure(2)
        plt.plot(self.data_cntrl_hz)
        plt.ylabel('Communication Frequency [Hz]')
        plt.title("Control Frequency")
        plt.show()


obj = anays_py_comm()
obj.getStaticsProHz()
obj.getStaticsNodeHz()
obj.drawGraph()
