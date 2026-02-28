#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from qcar2_interfaces.msg import MotorCommands


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class LaneFollowCmd(Node):
    def __init__(self):
        super().__init__("lane_follow_cmd_safe")

        self.declare_parameter("cmd_topic", "/qcar2_motor_speed_cmd")
        self.declare_parameter("off_topic", "/lane_off")
        self.declare_parameter("conf_topic", "/lane_conf")

        # Steering
        self.declare_parameter("kp", 0.65)
        self.declare_parameter("kd", 0.12)
        self.declare_parameter("max_steer", 0.65)
        self.declare_parameter("steer_lpf_hz", 6.0)
        self.declare_parameter("deadband", 0.02)
        self.declare_parameter("invert_steer", False)

        # Speed
        self.declare_parameter("v_straight", 0.22)
        self.declare_parameter("v_min", 0.12)
        self.declare_parameter("turn_slow_k", 0.10)

        # Confidence
        self.declare_parameter("conf_ok", 0.35)
        self.declare_parameter("conf_good", 0.65)
        self.declare_parameter("v_weak", 0.14)

        # Timeout behavior
        self.declare_parameter("cmd_timeout_s", 1.20)
        self.declare_parameter("timeout_creep", True)

        cmd_topic = self.get_parameter("cmd_topic").value
        off_topic = self.get_parameter("off_topic").value
        conf_topic = self.get_parameter("conf_topic").value

        self.pub = self.create_publisher(MotorCommands, cmd_topic, 10)
        self.sub_off = self.create_subscription(Float32, off_topic, self.cb_off, 10)
        self.sub_conf = self.create_subscription(Float32, conf_topic, self.cb_conf, 10)

        self.off = 0.0
        self.conf = 0.0
        self.last_msg_t = time.time()

        self.err_prev = 0.0
        self.steer_f = 0.0
        self.t_prev = time.time()

        self.timer = self.create_timer(1.0 / 50.0, self.tick)
        self.get_logger().info(
            f"LaneFollowCmdSafe pub={cmd_topic} sub={off_topic},{conf_topic}"
        )

    def _lpf_alpha(self, dt, hz):
        if hz <= 0:
            return 1.0
        rc = 1.0 / (2.0 * math.pi * hz)
        return dt / (dt + rc)

    def cb_off(self, msg: Float32):
        x = float(msg.data)
        if not math.isfinite(x):
            return
        self.off = float(clamp(x, -1.0, 1.0))
        self.last_msg_t = time.time()

    def cb_conf(self, msg: Float32):
        x = float(msg.data)
        if not math.isfinite(x):
            return
        self.conf = float(clamp(x, 0.0, 1.0))
        self.last_msg_t = time.time()

    def publish_cmd(self, steer, v):
        m = MotorCommands()
        m.motor_names = ["steering_angle", "motor_throttle"]
        m.values = [float(steer), float(v)]
        self.pub.publish(m)

    def tick(self):
        now = time.time()
        dt = max(1e-3, now - self.t_prev)
        self.t_prev = now

        timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        v_min = float(self.get_parameter("v_min").value)

        if (now - self.last_msg_t) > timeout_s:
            if bool(self.get_parameter("timeout_creep").value):
                self.publish_cmd(0.0, v_min)
            else:
                self.publish_cmd(0.0, 0.0)
            return

        kp = float(self.get_parameter("kp").value)
        kd = float(self.get_parameter("kd").value)
        max_steer = float(self.get_parameter("max_steer").value)
        steer_lpf_hz = float(self.get_parameter("steer_lpf_hz").value)
        deadband = float(self.get_parameter("deadband").value)
        invert = bool(self.get_parameter("invert_steer").value)

        v_straight = float(self.get_parameter("v_straight").value)
        turn_slow_k = float(self.get_parameter("turn_slow_k").value)

        conf_ok = float(self.get_parameter("conf_ok").value)
        conf_good = float(self.get_parameter("conf_good").value)
        v_weak = float(self.get_parameter("v_weak").value)

        err = self.off
        if abs(err) < deadband:
            err = 0.0
        if invert:
            err = -err

        derr = (err - self.err_prev) / dt
        self.err_prev = err

        steer_raw = kp * err + kd * derr

        a = self._lpf_alpha(dt, steer_lpf_hz)
        self.steer_f = (1 - a) * self.steer_f + a * steer_raw
        steer = float(clamp(self.steer_f, -max_steer, max_steer))

        v = v_straight - turn_slow_k * abs(self.off)
        v = float(clamp(v, v_min, v_straight))

        if self.conf < conf_ok:
            v = min(v, v_weak)
            steer = float(clamp(steer, -0.7 * max_steer, 0.7 * max_steer))
        elif self.conf < conf_good:
            v = min(v, 0.85 * v_straight)

        self.publish_cmd(steer, v)


def main():
    rclpy.init()
    n = LaneFollowCmd()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()