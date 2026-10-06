"""Kayıt düğümü: durum ve kuvveti CSV'ye yazar (sonra tools/plot_log.py ile çizilir).

Abone: /cartpole/state, /cartpole/force
Parametre `output_file`: CSV yolu (varsayılan /tmp/cartpole_log.csv)
"""
import csv

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class LoggerNode(Node):
    def __init__(self):
        super().__init__("cartpole_logger")
        self.declare_parameter("output_file", "/tmp/cartpole_log.csv")
        path = str(self.get_parameter("output_file").value)
        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["t", "x", "x_dot", "theta", "theta_dot", "force"])
        self.force = 0.0
        self.t0 = None
        self.create_subscription(Float64, "/cartpole/force", self.on_force, 10)
        self.create_subscription(Float64MultiArray, "/cartpole/state", self.on_state, 10)
        self.get_logger().info(f"Kayıt dosyası: {path}")

    def on_force(self, msg: Float64):
        self.force = float(msg.data)

    def on_state(self, msg: Float64MultiArray):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        self.writer.writerow([f"{now - self.t0:.4f}", *[f"{v:.6f}" for v in msg.data], f"{self.force:.4f}"])

    def destroy_node(self):
        if not self.file.closed:
            self.file.flush()
            self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
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
