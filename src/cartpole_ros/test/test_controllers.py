import numpy as np

from cartpole_ros.controllers import LQRController, PIDController
from cartpole_ros.dynamics import Params, linearized_matrices, rk4_step


def run(controller, theta0_deg=10.0, T=10.0, dt=0.005, u_max=30.0):
    p = Params()
    s = np.array([0.0, 0.0, np.deg2rad(theta0_deg), 0.0])
    for _ in range(int(T / dt)):
        u = float(np.clip(controller.compute(s, dt), -u_max, u_max))
        s = rk4_step(s, u, p, dt)
        if abs(s[2]) > np.pi / 2 or abs(s[0]) > 2.4:
            return s, False
    return s, True


def test_lqr_closed_loop_is_stable():
    p = Params()
    A, B = linearized_matrices(p)
    K = LQRController(p).K
    eig = np.linalg.eigvals(A - B @ K.reshape(1, 4))
    assert np.all(eig.real < 0)


def test_lqr_balances_from_10_degrees():
    s, ok = run(LQRController(Params()))
    assert ok
    assert abs(s[2]) < 0.02 and abs(s[0]) < 0.05


def test_pid_with_position_term_balances():
    s, ok = run(PIDController())
    assert ok
    assert abs(s[2]) < 0.02


def test_pid_without_position_term_drifts_off_track():
    # Sadece açıya bakan PID çubuğu tutar ama araba raydan çıkar
    _, ok = run(PIDController(kx=0.0, kv=0.0))
    assert not ok
