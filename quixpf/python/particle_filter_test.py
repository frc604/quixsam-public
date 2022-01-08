import matplotlib.pyplot as plt
from multiprocessing import Process
import numpy as np
import time
import transformations as tf

from data_utils import Odometry, Vision
from helpers import cart2besph, get_point_in_frame, plot_field, load_traj
from particle_filter import ParticleFilter, LANDMARKS, OUR_LANDMARK_IDs, OPPOSING_LANDMARK_IDs, ROBOT_TO_CAMERA, UpdateType
from plotter import Plotter


def run_traj(num_particles, plotter_queue):
    traj = load_traj('./trajectories/drive_everywhere.csv')

    fig, ax = plt.subplots()
    plt.axis('equal')
    plot_field(ax)
    
    pf = ParticleFilter(num_particles, Odometry(0, traj[0, 1], traj[0, 2], traj[0, 3]))
    noisy_T = tf.compose_matrix(translate=[ traj[0, 1],  traj[0, 2], 0], angles=[0, 0, traj[0, 3]])
    # pf.plot_particles(plt)
    # pf.plot_estimate(plt)

    for i in range(1, traj.shape[0]):
        print(f"------ Timestep: {i}")

        x, y, theta = traj[i, 1:4]
        last_x, last_y, last_theta = traj[i - 1, 1:4]

        # Compute deltas
        T = tf.compose_matrix(translate=[x, y, 0], angles=[0, 0, theta])
        last_T = tf.compose_matrix(translate=[last_x, last_y, 0], angles=[0, 0, last_theta])
        M = np.linalg.inv(last_T).dot(T)
        _, _, angles, trans, _ = tf.decompose_matrix(M)
        dx, dy, _ = trans
        _, _, dtheta = angles

        # Simulate noisy deltas
        xNoise = 3e-1 * abs(dx)  # Scale uncertainty by distance
        yNoise = 3e-1 * abs(dy)
        thetaNoise = 1e-6
        dx += np.random.normal(0.0, xNoise)
        dy += np.random.normal(0.0, yNoise)
        dtheta += np.random.normal(0.0, thetaNoise)
        noisy_T = noisy_T.dot(tf.compose_matrix(translate=[dx, dy, 0], angles=[0, 0, dtheta]))
        _, _, noisy_angles, noisy_trans, _ = tf.decompose_matrix(noisy_T)
        noisy_x, noisy_y, _ = noisy_trans
        _, _, noisy_theta = noisy_angles

        # Do predict
        s = time.time()
        pf.predict(Odometry(i, noisy_x, noisy_y, noisy_theta))
        print(f"Predict: {time.time() - s}")

        # Simulate measurements
        facing_our_goal = np.cos(theta) >= 0  # Our goal is in the +x direction
        landmarks = [LANDMARKS[id] for id in OUR_LANDMARK_IDs] if facing_our_goal else [LANDMARKS[id] for id in OPPOSING_LANDMARK_IDs]
        measurements = []
        for landmark in landmarks:
            mx, my, mz = get_point_in_frame(T.dot(ROBOT_TO_CAMERA), [landmark.x, landmark.y, landmark.z])
            be, ev = cart2besph(mx, my, mz)
            # Simulate FOV
            if abs(be) < np.deg2rad(30):
                measurements.append(Vision(0, be, ev))

        # Do update & resample
        s = time.time()
        update_type = pf.update(measurements)
        print(f"Update & resample: {time.time() - s}")

        has_vision = update_type != UpdateType.NONE
        facing_our_goal = update_type == UpdateType.OUR_GOAL

        # pf.plot_particles(plt, has_vision)
        pf.plot_estimate(plt, has_vision, facing_our_goal)
        plotter_queue.put((pf.particles, pf.get_best_estimate(), has_vision, facing_our_goal))

    plt.plot(traj[:, 1], traj[:, 2])
    plt.show()


if __name__ == '__main__':
    NUM_PARTICLES = 10000
    # Start plotter thread
    plotter = Plotter(NUM_PARTICLES)
    plotter_queue = plotter.start()

    # Start particle filtler thread
    t = Process(target=run_traj, args=(NUM_PARTICLES, plotter_queue,))
    t.start()

    print("Waiting for IO thread to join...")
    t.join()
    print("Waiting for graph window process to join...")
    plotter.join()
