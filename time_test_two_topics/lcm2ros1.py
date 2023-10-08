import numpy as np
import time
from lcm_msg.lcm_float32 import lcm_float32
import lcm

from std_msgs.msg import Float32
import rospy, tf


class lcm2ros1:
    def __init__(self):
        rospy.init_node("lcm2ros1")
        self.msg_send_time_stamp_from_sim = Float32
        self.msg_send_time_stamp_from_cntrl = Float32
        self.time_delay = Float32()

        self.loop_rate = 500
        self.lc = lcm.LCM()
        # sub_rcv_msg_from_sim = self.lc.subscribe('/msg/send_time_stamp_from_sim', self.sub_msg_from_sim)
        # sub_rcv_msg_from_cntrl = self.lc.subscribe('/msg/send_time_stamp_from_cntrl', self.sub_msg_from_cntrl)
        sub_delay = self.lc.subscribe('/delay', self.sub_time_delay)

        self.pub_time_delay= rospy.Publisher('/time_delay', Float32, queue_size=1)
        self.pub_send_time_stamp_from_sim  = rospy.Publisher('/send_time_stamp_from_sim', Float32, queue_size=1)
        self.pub_send_time_stamp_from_cntrl = rospy.Publisher('/send_time_stamp_from_cntrl', Float32, queue_size=1)

    def sub_time_delay(self, channel, data):
        msg = lcm_float32.decode(data)
        self.time_delay.data = msg.data

    def main(self):
        while True:
            start_time = time.time_ns()
            self.lc.handle_timeout(0)
            self.pub_time_delay.publish(self.time_delay)
            mid_time = time.time_ns()

            process_time = (mid_time - start_time) / (10 ** 9)

            left_time = 1.0 / self.loop_rate - process_time
            time.sleep(left_time)

if __name__ == "__main__":
    obj = lcm2ros1()
    obj.main()