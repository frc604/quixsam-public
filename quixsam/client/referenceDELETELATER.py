import time

import gtsam_unstable
import gtsam
import numpy as np
from gtsam.symbol_shorthand import L
from gtsam.symbol_shorthand import X

import matplotlib.pyplot as plt
import gtsam.utils.plot as gtsam_plot

from utils import besph2cart, cart2besph

np.random.seed(0)

# NUM_NODES = 3 * 60  # 3 minutes at 1 Hz
NUM_NODES = 100
PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([1e-6, 1e-6, 1e-1, 1e-1, 1e-1, 1e-6], dtype=np.float)  # roll, pitch, yaw, x, y, z
)
PRIOR_LANDMARK_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-6, 1e-6, 1e-6], dtype=np.float))
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([1e-6, 1e-6, 1e-1, 5e-2, 5e-2, 1e-6], dtype=np.float)  # roll, pitch, yaw, x, y, z
)
BEARING_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-3, 1e-3], dtype=np.float))
ATTITUDE_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-6, 1e-6], dtype=np.float))
ELEVATION_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-6], dtype=np.float))

parameters = gtsam.ISAM2Params()

isam = gtsam.ISAM2(parameters)

graph = gtsam.NonlinearFactorGraph()
estimate = gtsam.Values()

# Add landmarks
estimate.insert(L(0), gtsam.Point3(10, 0, 10))
graph.add(gtsam.PriorFactorPoint3(L(0), gtsam.Point3(10, 0, 10), PRIOR_LANDMARK_NOISE))
estimate.insert(L(1), gtsam.Point3(10, 10, 10))
graph.add(gtsam.PriorFactorPoint3(L(1), gtsam.Point3(10, 10, 10), PRIOR_LANDMARK_NOISE))

# Begin
simulated_pose = gtsam.Pose3()
for i in range(0, NUM_NODES + 1):
    print(f"Node: {i}")
    s = time.time()

    # Add new pose estimate
    estimate.insert(X(i), simulated_pose)

    # Add odometry factor
    if i == 0:
        graph.add(gtsam.PriorFactorPose3(X(0), simulated_pose, PRIOR_NOISE))
    else:
        graph.add(
            gtsam.BetweenFactorPose3(
                X(i - 1),
                X(i),
                pose_delta,
                ODOMETRY_NOISE,
            )
        )

    # Add landmark factor at each end
    if i % 10 == 0:
        if int(i / 10) % 2 == 0:
            # Right side
            graph.add(
                gtsam.BearingFactor3D(
                    X(i),
                    L(0),
                    gtsam.Unit3(besph2cart(0, np.pi / 4)),
                    BEARING_NOISE
                )
            )
            graph.add(
                gtsam.BearingFactor3D(
                    X(i),
                    L(1),
                    gtsam.Unit3(besph2cart(cart2besph(10, 10, 10)[0], cart2besph(10, 10, 10)[1])),
                    BEARING_NOISE,
                )
            )
        else:
            # Left side
            graph.add(
                gtsam.BearingFactor3D(
                    X(i),
                    L(0),
                    gtsam.Unit3(besph2cart(cart2besph(10, -10, 10)[0], cart2besph(10, -10, 10)[1])),
                    BEARING_NOISE,
                )
            )
            graph.add(
                gtsam.BearingFactor3D(
                    X(i),
                    L(1),
                    gtsam.Unit3(besph2cart(0, np.pi / 4)),
                    BEARING_NOISE
                )
            )

    # Attitude factor (keep poses flat)
    graph.add(
        gtsam.Pose3AttitudeFactor(X(i), gtsam.Unit3(np.array([0, 0, -1])), ATTITUDE_NOISE)
    )  # down vector

    # Keep poses on the ground
    graph.add(gtsam_unstable.RelativeElevationFactor(X(i), L(0), -10, ELEVATION_NOISE))
    graph.add(gtsam_unstable.RelativeElevationFactor(X(i), L(1), -10, ELEVATION_NOISE))

    # Move in the +Y direction for 10 steps, then -Y for 10 steps, then +Y...
    pose_delta = gtsam.Pose3(
        gtsam.Pose2(
            np.random.normal(0, 0.05),  # x
            (1 + np.random.normal(0, 0.05)) * (1 if int(i / 10) % 2 == 0 else -1),  # y
            np.random.normal(0, 0.1),  # theta
        )
    )

    # Solve incremental
    isam.update(graph, estimate)
    graph.resize(0)
    estimate.clear()

    current_estimate = isam.calculateEstimate()
    print(current_estimate.atPose3(X(i)))

    # Simulate motion
    simulated_pose = current_estimate.atPose3(X(i)).compose(pose_delta)

fig = plt.figure(0)
print(isam.marginalCovariance(L(0)))
gtsam_plot.plot_point3(0, current_estimate.atPoint3(L(0)), '.', P=isam.marginalCovariance(L(0)))
gtsam_plot.plot_point3(0, current_estimate.atPoint3(L(1)), '.', P=isam.marginalCovariance(L(1)))
for idx in range(i):
    covariance = isam.marginalCovariance(X(idx))
    gtsam_plot.plot_pose3(0, current_estimate.atPose3(X(idx)), axis_length=0.5)
    gtsam_plot.plot_pose3(0, current_estimate.atPose3(X(idx)), axis_length=0.5, P=covariance)
    print("======================")
    print(idx)
    print(covariance)

gtsam_plot.set_axes_equal(0)
plt.show()
