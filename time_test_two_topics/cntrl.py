import numpy as np
import time
from lcm_msg.lcm_float32 import lcm_float32
import lcm
import select
class cntrl:
    def __init__(self):
        self.send_time_stamp_from_cntrl = lcm_float32()
        self.msg_rcv_time_stamp_from_sim = 0
        self.loop_rate = 100
        self.counter = 0

        self.lc = lcm.LCM()
        sub_rcv_msg_from_sim = self.lc.subscribe('/msg/send_time_stamp_from_sim', self.sub_msg_from_sim)
        sub_rcv_msg_from_sim = self.lc.subscribe('/msg/randam1', self.sub_msg_from_sim_random1)


    def sub_msg_from_sim(self, channel, data):
        msg = lcm_float32.decode(data)
        self.msg_rcv_time_stamp_from_sim = msg.data

        cur_time = time.time_ns() / (10 ** 9)
        cur_time -= cur_time - cur_time % 10
        print("cntrl rcv time stamp: {:.4f}".format(self.msg_rcv_time_stamp_from_sim), " at {:.4f}".format(cur_time))

    def sub_msg_from_sim_random1(self, channel, data):
        msg = lcm_float32.decode(data)
        print('rcv random topix')

    def pub_msg_from_cntrl(self):
        self.counter += 1

        self.send_time_stamp_from_cntrl.data = self.msg_rcv_time_stamp_from_sim
        # self.send_time_stamp_from_cntrl.data = self.counter * 0.0001
        self.lc.publish('/msg/send_time_stamp_from_cntrl', self.send_time_stamp_from_cntrl.encode())
        cur_time = time.time_ns()/(10**9)
        cur_time -= cur_time - cur_time % 10
        print("cntrl send time stamp: {:.4f}".format(self.send_time_stamp_from_cntrl.data), " at {:.4f}".format( cur_time))

    def main(self):
        while True:
            start_time = time.time_ns()
            # self.lc.handle()
            # self.lc.handle()
            # test=self.lc.fileno()

            rfds, wfds, efds = select.select([self.lc.fileno()], [], [], 0)
            while rfds:
                rfds, wfds, efds = select.select([self.lc.fileno()], [], [], 0)
                if rfds:
                    self.lc.handle()

            self.pub_msg_from_cntrl()
            mid_time = time.time_ns()

            process_time = (mid_time - start_time) / (10 ** 9)

            left_time = 1.0 / self.loop_rate - process_time
            if left_time > 0:
                time.sleep(left_time)
            total_time = (time.time_ns() - start_time) / (10 ** 9)
            print("cntrl: process time {:.4f}".format( process_time), "total_time {:.4f}".format(total_time))

if __name__ == "__main__":
    obj = cntrl()
    obj.main()