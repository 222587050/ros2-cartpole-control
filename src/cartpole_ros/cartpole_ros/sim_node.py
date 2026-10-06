"""Simülatör düğümü: fiziği entegre eder, durumu yayınlar, kuvveti dinler.

Yayınlar : /cartpole/state  (std_msgs/Float64MultiArray: [x, x_dot, theta, theta_dot])
Abone    : /cartpole/force  (std_msgs/Float64: arabaya uygulanan kuvvet, N)
Servisler: /cartpole/push   (std_srvs/Trigger: arabaya kısa bir dış itme uygular)
           /cartpole/reset  (std_srvs/Trigger: simülasyonu başlangıca döndürür)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import Trigger

from .dynamics import Params, rk4_step


class SimNode(Node):
    def __init__(self):
        super().__init__("cartpole_sim")
        for name, default in [("M", 1.0), ("m", 0.1), ("l", 0.5), ("g", 9.81), ("b", 0.1)]:
            self.declare_parameter(name, default)
        self.declare_parameter("dt", 0.005)
        self.declare_parameter("initial_angle_deg", 10.0)
        self.declare_parameter("track_limit", 2.4)
        self.declare_parameter("push_force", 15.0)
        self.declare_parameter("push_duration", 0.2)
        self.declare_parameter("wait_for_controller", True)

        self.p = Params(*(self.get_parameter(n).value for n in ("M", "m", "l", "g", "b")))
        self.dt = float(self.get_parameter("dt").value)
        self.track = float(self.get_parameter("track_limit").value)
        self.push_force = float(self.get_parameter("push_force").value)
        self.push_duration = float(self.get_parameter("push_duration").value)

        self.state = self._initial_state()
        self.force = 0.0        # son alınan kontrol kuvveti (sıfırıncı derece tutucu)
        self.push_left = 0.0    # kalan dış itme süresi (s)
        self.failed = False
        self.wait = bool(self.get_parameter("wait_for_controller").value)

        self.state_pub = self.create_publisher(Float64MultiArray, "/cartpole/state", 10)
        self.create_subscription(Float64, "/cartpole/force", self.on_force, 10)
        self.create_service(Trigger, "/cartpole/push", self.on_push)
        self.create_service(Trigger, "/cartpole/reset", self.on_reset)
        self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f"Simülatör başladı: dt={self.dt}s, başlangıç açısı={self.get_parameter('initial_angle_deg').value}°")

    def _initial_state(self):
        th0 = math.radians(float(self.get_parameter("initial_angle_deg").value))
        return np.array([0.0, 0.0, th0, 0.0])

    def on_force(self, msg: Float64):
        self.force = float(msg.data)
        if self.wait:
            self.wait = False
            self.get_logger().info("Denetleyici bağlandı, simülasyon başlıyor.")

    def on_push(self, request, response):
        self.push_left = self.push_duration
        response.success = True
        response.message = f"{self.push_force} N itme, {self.push_duration} s"
        return response

    def on_reset(self, request, response):
        self.state = self._initial_state()
        self.force = 0.0
        self.push_left = 0.0
        self.failed = False
        self.wait = bool(self.get_parameter("wait_for_controller").value)
        response.success = True
        response.message = "Simülasyon sıfırlandı"
        return response

    def step(self):
        if not self.failed and not self.wait:
            u = self.force
            if self.push_left > 0.0:
                u += self.push_force
                self.push_left -= self.dt
            self.state = rk4_step(self.state, u, self.p, self.dt)
            if abs(self.state[2]) > math.pi / 2 or abs(self.state[0]) > self.track:
                self.failed = True
                self.get_logger().error(
                    f"Başarısız: x={self.state[0]:.2f} m, theta={math.degrees(self.state[2]):.1f}°. "
                    "Yeniden başlatmak için /cartpole/reset servisini çağır.")
        msg = Float64MultiArray()
        msg.data = [float(v) for v in self.state]
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
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
