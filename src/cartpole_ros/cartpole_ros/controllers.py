"""PID ve LQR denetleyicileri (ROS'tan bağımsız)."""
import numpy as np
from scipy.linalg import solve_continuous_are

from .dynamics import Params, linearized_matrices


class PIDController:
    """u = kp*theta + ki*int(theta) + kd*theta_dot + kx*x + kv*x_dot"""

    def __init__(self, kp=60.0, ki=0.0, kd=10.0, kx=2.0, kv=4.0):
        self.kp, self.ki, self.kd, self.kx, self.kv = kp, ki, kd, kx, kv
        self.integral = 0.0

    def compute(self, state, dt):
        x, xd, th, thd = state
        self.integral += th * dt
        return self.kp * th + self.ki * self.integral + self.kd * thd + self.kx * x + self.kv * xd

    def reset(self):
        self.integral = 0.0


class LQRController:
    """u = -K s, K = R^-1 B^T P (sürekli zaman cebirsel Riccati denklemi)."""

    def __init__(self, p: Params, q_diag=(1.0, 1.0, 10.0, 1.0), r=0.1):
        A, B = linearized_matrices(p)
        Q = np.diag(q_diag)
        R = np.array([[r]])
        P = solve_continuous_are(A, B, Q, R)
        self.K = (np.linalg.inv(R) @ B.T @ P).flatten()

    def compute(self, state, dt=None):
        return float(-self.K @ np.asarray(state))

    def reset(self):
        pass
