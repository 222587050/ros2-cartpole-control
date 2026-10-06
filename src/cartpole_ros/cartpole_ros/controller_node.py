"""Denetleyici düğümü: durumu dinler, LQR veya PID ile kuvvet hesaplayıp yayınlar.

Abone   : /cartpole/state (std_msgs/Float64MultiArray)
Yayınlar: /cartpole/force (std_msgs/Float64)
Parametre `controller_type`: "lqr" veya "pid"
"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

from .controllers import LQRController, PIDController
from .dynamics import Params


class ControllerNode(Node):
    def __init__(self):
        super().__init__("cartpole_controller")
        for name, default in [("M", 1.0), ("m", 0.1), ("l", 0.5), ("g", 9.81), ("b", 0.1)]:
            self.declare_parameter(name, default)
        self.declare_parameter("dt", 0.005)
        self.declare_parameter("controller_type", "lqr")
        self.declare_parameter("u_max", 30.0)
        self.declare_parameter("lqr_q_diag", [1.0, 1.0, 10.0, 1.0])
        self.declare_parameter("lqr_r", 0.1)
        for name, default in [("pid_kp", 60.0), ("pid_ki", 0.0), ("pid_kd", 10.0),
                              ("pid_kx", 2.0), ("pid_kv", 4.0)]:
            self.declare_parameter(name, default)

        p = Params(*(self.get_parameter(n).value for n in ("M", "m", "l", "g", "b")))
        self.dt = float(self.get_parameter("dt").value)
        self.u_max = float(self.get_parameter("u_max").value)
        kind = str(self.get_parameter("controller_type").value).lower()

        if kind == "lqr":
            self.controller = LQRController(
                p, q_diag=tuple(self.get_parameter("lqr_q_diag").value),
                r=float(self.get_parameter("lqr_r").value))
            self.get_logger().info(f"LQR denetleyici, K = {np.round(self.controller.K, 2).tolist()}")
        elif kind == "pid":
            def g(n):
                return float(self.get_parameter(n).value)
            self.controller = PIDController(g("pid_kp"), g("pid_ki"), g("pid_kd"), g("pid_kx"), g("pid_kv"))
            self.get_logger().info("PID denetleyici (açı + konum PD)")
        else:
            raise ValueError(f"Bilinmeyen controller_type: {kind!r} (lqr veya pid olmalı)")

        self.force_pub = self.create_publisher(Float64, "/cartpole/force", 10)
        self.create_subscription(Float64MultiArray, "/cartpole/state", self.on_state, 10)

    def on_state(self, msg: Float64MultiArray):
        state = np.array(msg.data, dtype=float)
        if state.shape != (4,):
            self.get_logger().warn(f"Beklenmeyen durum boyutu: {state.shape}")
            return
        u = float(np.clip(self.controller.compute(state, self.dt), -self.u_max, self.u_max))
        out = Float64()
        out.data = u
        self.force_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
