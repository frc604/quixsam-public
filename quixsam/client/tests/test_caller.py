import pytest
import logging

from gtsam.gtsam.symbol_shorthand import L, X

from ..data_utils import *
from ..gtsam_caller import GTSAMCaller

import matplotlib

matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
import gtsam.utils.plot as gtsam_plot

from gtsam import KeySet

logging.basicConfig(level=logging.INFO)


def compose_odometry(odometry: Odometry, delta: Odometry):
    return Odometry(
        odometry.id + delta.id,
        odometry.x + delta.x,
        odometry.y + delta.y,
        odometry.theta + delta.theta,
        delta.sigmaX,
        delta.sigmaY,
        delta.sigmaTheta,
    )


@pytest.fixture(scope="function")
def caller():
    priori = Odometry(0, 0, 0, 0, 1e-1, 1e-1, 1e-1)
    landmarks = [Landmark(0, 10, 0, 10, epsilon, epsilon, epsilon), Landmark(1, 10, 10, 10, epsilon, epsilon, epsilon)]

    return GTSAMCaller(priori, landmarks)


def test_init():
    priori = Odometry(0, 0, 0, 0, 1, 1, 1)
    landmarks = [Landmark(0, 10, 0, 10, 0, 0, 0), Landmark(1, 10, 10, 10, 0, 0, 0)]

    caller = GTSAMCaller(priori, landmarks)

    key_set = KeySet()
    key_set.insert(L(0))
    key_set.insert(L(1))
    key_set.insert(X(0))

    assert caller.graph.keys().equals(key_set)


def test_reset(caller):
    priori = Odometry(0, 1, 1, 1, 1e-1, 1e-1, 1e-1)
    landmarks = [Landmark(0, 0, 0, 10, 0, 0, 0), Landmark(1, 10, 10, 0, 0, 0, 0)]

    caller.reset(priori, landmarks)

    graph = gtsam.NonlinearFactorGraph()
    graph.add(gtsam.PriorFactorPoint3(L(0), gtsam.Point3(0, 0, 10), make_noise_model([0, 0, 0])))
    graph.add(gtsam.PriorFactorPoint3(L(1), gtsam.Point3(10, 10, 0), make_noise_model([0, 0, 0])))
    graph.add(
        gtsam.PriorFactorPose3(
            X(0),
            gtsam.Pose3(gtsam.Pose2(1, 1, 1)),
            make_noise_model([epsilon, epsilon, 0.1, 0.1, 0.1, epsilon]),
        )
    )

    assert caller.graph.equals(graph, 1e-9)


def test_estimate(caller):
    vision = []
    odometry = []

    previous_odometry = caller.priori

    num_nodes = 100

    for i in range(1, num_nodes):
        simulated_pose = previous_odometry.compose(
            Odometry(
                1,
                np.random.normal(0, 0.02),  # x
                (1 + np.random.normal(0, 0.02)) * (1 if int((i-1) / 10) % 2 == 0 else -1),  # y
                np.random.normal(0, 0.05),  # theta
                0.02,
                0.02,
                0.05,
            )
        )

        previous_odometry = simulated_pose
        odometry.append(simulated_pose)

        if i % 10 == 0:
            if int(i / 10) % 2 == 0:
                vision.append(Vision(i, 0, 0, np.pi / 4, 1e-3, 1e-3))
                vision.append(Vision(i, 1, cart2besph(10, 10, 10)[0], cart2besph(10, 10, 10)[1], 1e-3, 1e-3))
            else:
                vision.append(
                    Vision(i, 0, cart2besph(10, -10, 10)[0], cart2besph(10, -10, 10)[1], 1e-3, 1e-3)
                )
                vision.append(Vision(i, 1, 0, np.pi / 4, 1e-3, 1e-3))

        caller.update(odometry, vision)
        odometry.clear()
        vision.clear()

        current_estimate = caller.solve()

    fig = plt.figure(0)
    gtsam_plot.plot_point3(0, current_estimate.atPoint3(L(0)), '.', P=caller.isam.marginalCovariance(L(0)))
    gtsam_plot.plot_point3(0, current_estimate.atPoint3(L(1)), '.', P=caller.isam.marginalCovariance(L(1)))
    for idx in range(0, i):
        covariance = caller.isam.marginalCovariance(X(idx))
        gtsam_plot.plot_pose3(0, current_estimate.atPose3(X(idx)), axis_length=0.5)
        gtsam_plot.plot_pose3(0, current_estimate.atPose3(X(idx)), axis_length=0.5, P=covariance)

    gtsam_plot.set_axes_equal(0)
    plt.show()
