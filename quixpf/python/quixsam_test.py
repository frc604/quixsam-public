import matplotlib.pyplot as plt
from multiprocessing import Process
from networktables import NetworkTables
import numpy as np
import time
import transformations as tf

from helpers import cart2besph, get_point_in_frame, plot_field, load_traj
from particle_filter import (
    LANDMARKS,
    OUR_LANDMARK_IDs,
    OPPOSING_LANDMARK_IDs,
    ROBOT_TO_CAMERA,
)
from quixsam import Quixsam


def mock_robot():
    NetworkTables.initialize()
    quixsam_table = NetworkTables.getTable("quixsam")
    odometry_table = quixsam_table.getSubTable("odometry")
    vision_table = quixsam_table.getSubTable("vision")
    estimates_table = quixsam_table.getSubTable("estimates")

    # Setup estimates listener
    xs = []
    ys = []

    def estimate_received(table, key, value, is_new):
        _, x, y, _ = value
        xs.append(x)
        ys.append(y)

    estimates_table.addEntryListener(estimate_received)

    traj = load_traj("./trajectories/drive_everywhere.csv")
    for id in range(traj.shape[0]):
        # Mock odometry
        x, y, theta = traj[id, 1:4]
        odometry_table.putNumberArray(
            str(id), [id, x, y, theta, 0, 0, 0]
        )

        # Mock vision
        facing_our_goal = np.cos(theta) >= 0  # Our goal is in the +x direction
        landmarks = (
            [LANDMARKS[id] for id in OUR_LANDMARK_IDs]
            if facing_our_goal
            else [LANDMARKS[id] for id in OPPOSING_LANDMARK_IDs]
        )
        T = tf.compose_matrix(translate=[x, y, 0], angles=[0, 0, theta])
        vision_array = [id]
        for landmark in landmarks:
            mx, my, mz = get_point_in_frame(
                T.dot(ROBOT_TO_CAMERA), [landmark.x, landmark.y, landmark.z]
            )
            be, ev = cart2besph(mx, my, mz)
            # Simulate FOV
            if abs(be) < np.deg2rad(30):
                vision_array += [be, ev, 0, 0]
        if len(vision_array) == 17:
            vision_table.getEntry(str(id)).setNumberArray(vision_array)
        NetworkTables.flush()
        time.sleep(0.02)

    # Plot result vs. ground truth
    _, ax = plt.subplots()
    plt.axis("equal")
    plot_field(ax)
    plt.plot(traj[:, 1], traj[:, 2])  # Ground truth
    plt.scatter(xs, ys, marker=".", color="r")  # Estimate
    plt.show()


def run_quixsam():
    Quixsam("localhost").run()


if __name__ == "__main__":
    # Start Quixsam
    qs = Process(target=run_quixsam)
    qs.start()

    # Start running mock robot
    p = Process(target=mock_robot)
    p.start()

    # Join threads
    qs.join()
    p.join()
