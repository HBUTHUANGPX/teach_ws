#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import numpy as np
import tf
import tf.transformations
import math
from scipy.spatial.transform import Rotation as R


class PID:
    P: float
    I: float
    D: float
    error_1: float
    error_2: float
    out_put: float

    def __init__(self, _name) -> None:
        self.name = _name
        self.P = rospy.get_param(self.name + "/p")
        self.I = rospy.get_param(self.name + "/i")
        self.D = rospy.get_param(self.name + "/d")
        self.error_1 = 0.0
        self.error_2 = 0.0
        self.out_put = 0.0

    def calc_PD(
        self, target_p: float, target_d: float, now_p: float, now_d: float
    ) -> None:
        self.error_1 = self.P * (target_p - now_p)
        self.error_2 = self.D * (target_d - now_d)
        self.out_put = self.error_1 + self.error_2

    def reload(self) -> None:
        self.P = rospy.get_param(self.name + "/p")
        self.I = rospy.get_param(self.name + "/i")
        self.D = rospy.get_param(self.name + "/d")

    def set_zero_out(self) -> None:
        self.out_put = 0.0


class motor:

    inner_PID: PID

    def __init__(self, _ns_name, _mt_name) -> None:
        self.init_date()
        self.name = "/" + _ns_name + "/" + _mt_name
        f, r, h = _mt_name.split("_")
        if f == "f":
            _f = "front"
        else:
            _f = "rear"
        if r == "r":
            _r = "right"
        else:
            _r = "left"
        if h == "h":
            _h = "hang"
            self.mt_ctr_target = "pos"
            self.inner_PID = PID("/" + _ns_name + "/pid/" + _h)
        else:
            _h = "dong"
            self.mt_ctr_target = "vel"
            self.inner_PID = PID("/" + _ns_name + "/pid/" + _h)

        self.joint_name = _f + "_" + _r + "_" + _h + "_joint"

        self.effort_pub = rospy.Publisher(
            self.name + "_j_e_controller/command", Float64, queue_size=1
        )

        self.joint_state_sub = rospy.Subscriber(
            "/" + _ns_name + "/joint_states",
            JointState,
            self.joint_state_callback,
            queue_size=1,
        )

    def init_date(self):
        self.current_pos = 0.0
        self.current_vel = 0.0
        self.current_acc = 0.0
        self.current_jerk = 0.0

        self.latest_pos = 0.0
        self.latest_vel = 0.0
        self.latest_acc = 0.0
        self.latest_jerk = 0.0

        self.target_vel = 0.0
        self.target_pos = 0.0
        self.target_acc = 0.0
        self.target_jerk = 0.0

        self.pub_msg = Float64()

    def joint_state_callback(self, msg):
        try:
            # 获取关节的索引
            index = msg.name.index(self.joint_name)
            self.latest_pos = self.current_pos
            self.latest_vel = self.current_vel
            self.latest_acc = self.current_acc
            self.latest_jerk = self.current_jerk
            # 获取当前关节位置和速度
            self.current_pos = msg.position[index]
            self.current_vel = msg.velocity[index]
            self.current_acc = (self.current_vel - self.latest_vel) / 0.001
            self.current_jerk = (self.current_acc - self.latest_acc) / 0.001

            if self.mt_ctr_target == "pos":
                self.inner_PID.calc_PD(
                    target_p=self.target_pos,
                    target_d=self.target_vel,
                    now_p=self.current_pos,
                    now_d=self.current_vel,
                )
            elif self.mt_ctr_target == "vel":
                self.inner_PID.calc_PD(
                    target_p=self.target_vel,
                    target_d=self.target_acc,
                    now_p=self.current_vel,
                    now_d=self.current_acc,
                )
            self.pub_msg.data = self.inner_PID.out_put
            self.effort_pub.publish(self.pub_msg)
        except ValueError:
            rospy.logwarn("Joint %s not found in joint_states", self.joint_name)


from typing import List


class dlbb:
    def __init__(self) -> None:
        self.hang_mt_l: List[motor] = []
        self.dong_mt_l: List[motor] = []
        self.init_motor_list()
        self._sub_cmd = rospy.Subscriber("cmd_vel", Twist, self.sub_cmd, queue_size=1)
        self._sub_imu = rospy.Subscriber("trunk_imu", Imu, self.sub_imu, queue_size=1)
        self.abs_yaw: float = 0.0
        self.tf_listener = tf.TransformListener()

    def sub_cmd(self, msg: Twist):
        # print(msg.linear.x)

        # test 1 =====================
        # for d_mt in self.dong_mt_l:
        #     d_mt.target_vel = msg.linear.x / 0.06

        # test 2 =========================
        rospy.loginfo("====================")
        _x_vel = msg.linear.x
        _y_vel = msg.linear.y
        _w_vel = msg.angular.z
        # 重映射
        __x_vel = _x_vel * math.cos(self.abs_yaw) - _y_vel * math.sin(self.abs_yaw)
        __y_vel = _x_vel * math.sin(self.abs_yaw) + _y_vel * math.cos(self.abs_yaw)
        # print(self.abs_yaw)
        # 假设无打滑
        vel = np.zeros(4)
        pos = np.zeros(4)
        cos_wz = _w_vel * 0.171
        x_sign = [-1, 1, -1, 1]
        y_sign = [1, 1, -1, -1]
        for ℹ in range(4):
            x_vel = __x_vel + x_sign[i] * cos_wz
            y_vel = __y_vel + y_sign[i] * cos_wz
            pos[i] = math.atan2(y_vel, x_vel)
            vel[i] = math.sqrt(x_vel * x_vel + y_vel * y_vel)
            # print(x_vel, y_vel, cos_wz, pos[i], vel[i])
            self.dong_mt_l[i].target_vel = vel[i] / 0.06
            self.hang_mt_l[i].target_pos = pos[i]
        ...

    def sub_imu(self, msg: Imu):
        quaternion = [
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ]  # [x, y, z, w]
        r = R.from_quat(quaternion)
        euler = r.as_euler("xyz", degrees=False)
        self.abs_yaw = euler[0]
        print(self.abs_yaw)
        ...

    def init_motor_list(self):
        _f = ["f", "r"]
        _r = ["l", "r"]
        _h = ["d", "h"]
        _ns_name = "dlbb"

        for f in _f:
            for r in _r:
                mt_name = f + "_" + r + "_" + _h[0]
                self.dong_mt_l.append(motor(_ns_name, mt_name))
                mt_name = f + "_" + r + "_" + _h[1]
                self.hang_mt_l.append(motor(_ns_name, mt_name))


if __name__ == "__main__":
    rospy.init_node("haha")
    HPX_dlbb = dlbb()
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
