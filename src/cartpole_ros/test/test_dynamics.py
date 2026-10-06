import numpy as np

from cartpole_ros.dynamics import Params, linearized_matrices, nonlinear_rhs, rk4_step


def test_upright_equilibrium_has_zero_derivative():
    p = Params()
    assert np.allclose(nonlinear_rhs(np.zeros(4), 0.0, p), 0.0)


def test_linearization_matches_nonlinear_for_small_angle():
    p = Params()
    A, B = linearized_matrices(p)
    s = np.array([0.0, 0.0, 1e-4, 0.0])
    u = 0.01
    lin = A @ s + (B * u).flatten()
    assert np.allclose(lin, nonlinear_rhs(s, u, p), atol=1e-6)


def test_uncontrolled_pole_falls_over():
    p = Params()
    s = np.array([0.0, 0.0, np.deg2rad(5), 0.0])
    for _ in range(int(2.0 / 0.005)):
        s = rk4_step(s, 0.0, p, 0.005)
    assert abs(s[2]) > np.deg2rad(45)


def test_system_is_open_loop_unstable():
    A, _ = linearized_matrices(Params())
    assert np.max(np.linalg.eigvals(A).real) > 1.0
