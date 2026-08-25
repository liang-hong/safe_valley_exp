#!/usr/bin/env python3
"""GPS 时钟偏置节点。

订阅 /mavros/time_reference（~1Hz，GPS PPS 秒脉冲，source=fcu），计算
    bias = header.stamp - time_ref
即本机 ROS 系统时钟与 GPS 绝对时间（GPST）的偏差——本质是常数（晶振固定偏差）
叠加缓慢漂移。本节点在滑动窗口（默认 10s）内取均值平滑 PPS/网络抖动，以 1Hz
发布 std_msgs/Float64（秒）到 `gps_bias`（latch）。

GPS 失锁（time_reference 超过 lockout_s 未更新）时停止发布新值，接收端用
"最后一次接收时刻"判断新鲜度并降级（回退相对时钟，不修正）。

邻机经 topology bridge 把 /UAVn/gps_bias 转发到本机 master，接收端用
    T_own = T_other - bias_other + bias_own
把邻机话题时间戳修正到本机时间轴（消除机间时钟漂移对 intent 时间对齐的影响）。

用法：rosrun safe_valley_exp gps_bias_node.py
参数（私有）：~window_s(10.0) ~publish_rate_hz(1.0) ~lockout_s(3.0)
"""
import rospy
from collections import deque

from sensor_msgs.msg import TimeReference
from std_msgs.msg import Float64


class GpsBiasNode:
    def __init__(self):
        self.window_s = float(rospy.get_param("~window_s", 10.0))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 1.0))
        self.lockout_s = float(rospy.get_param("~lockout_s", 3.0))

        self.samples = deque()          # [(now_s, bias_s), ...] 滑动窗口
        self.last_bias_s = None         # 最近窗口均值
        self.last_update_s = None       # 最近一次 time_reference 到达时刻

        self.pub = rospy.Publisher("gps_bias", Float64, queue_size=1, latch=True)
        rospy.Subscriber("/mavros/time_reference", TimeReference,
                         self.on_time_ref, queue_size=10, tcp_nodelay=True)
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate_hz), self.publish)

    def on_time_ref(self, msg):
        now = rospy.Time.now().to_sec()
        bias = (msg.header.stamp - msg.time_ref).to_sec()
        self.samples.append((now, bias))
        cutoff = now - self.window_s
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()
        self.last_bias_s = sum(b for _, b in self.samples) / len(self.samples)
        self.last_update_s = now

    def publish(self, event):
        if self.last_bias_s is None:
            return  # 尚无 GPS 时间参考，不发
        if self.last_update_s is not None and \
                rospy.Time.now().to_sec() - self.last_update_s > self.lockout_s:
            # GPS 失锁：停止发布新值（latch 保留最后一个；接收端按接收时刻判新鲜）
            return
        self.pub.publish(Float64(self.last_bias_s))


if __name__ == "__main__":
    rospy.init_node("gps_bias_node", anonymous=False)
    GpsBiasNode()
    rospy.spin()
