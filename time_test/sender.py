import numpy as np
import time
from lcm_msg.lcm_float32 import lcm_float32
import lcm

class sim:
    def __init__(self):
        self.send_time_stamp_from_sim = lcm_float32()
        self.msg_return_time_stamp = 0
        self.loop_rate = 500

        self.delay_msg = lcm_float32()
        self.delay = 0
        self.lc = lcm.LCM()
        sub_rcv_msg_from_cntrl = self.lc.subscribe('/msg/send_time_stamp_from_cntrl', self.sub_msg_from_cntrl)

    def sub_msg_from_cntrl(self,channel, data):
        msg = lcm_float32.decode(data)
        self.msg_return_time_stamp = msg.data

        cur_time = time.time_ns()/(10**9)
        cur_time -= cur_time - cur_time % 10
        self.delay = cur_time - self.msg_return_time_stamp
        print("sim rcv time stamp: {:.4f}".format(self.msg_return_time_stamp), " at {:.4f}".format(cur_time), " delay:{:.4f}".format(self.delay))

    def pub_msg_from_sim(self):
        cur_time = time.time_ns() / (10 ** 9)
        cur_time -= cur_time - cur_time % 10
        self.send_time_stamp_from_sim.data = cur_time
        self.lc.publish('/msg/send_time_stamp_from_sim', self.send_time_stamp_from_sim.encode())

        cur_time = time.time_ns()/(10**9)
        cur_time -= cur_time - cur_time % 10
        print("sim send time stamp: {:.4f}".format(self.send_time_stamp_from_sim.data), " at {:.4f}".format(cur_time))

        self.delay_msg.data = self.delay
        self.lc.publish('/delay', self.delay_msg.encode())

    def main(self):
        while True:
            start_time = time.time_ns()
            self.pub_msg_from_sim()
            self.lc.handle()
            mid_time = time.time_ns()

            process_time = (mid_time - start_time) / (10**9)

            left_time = 1.0/self.loop_rate - process_time
            if left_time>0:
                time.sleep(left_time)
            total_time = (time.time_ns() - start_time) / (10**9)
            print("process time {:.4f}".format(process_time), "total_time {:.4f}".format(total_time))

if __name__ == "__main__":
    obj = sim()
    obj.main()

