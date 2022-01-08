import math
from typing import List
from helpers import (
    in2m,
    cart2besph,
    get_point_in_frame,
    get_x,
    set_x,
    get_y,
    set_y,
    set_yaw,
)
import numpy as np
import enum
import transformations as tf

from data_utils import Odometry, Vision, Landmark

# SIR Particle Filter

ROBOT_TO_CAMERA = tf.compose_matrix(
    translate=[in2m(11.1), 0, in2m(34.6)], angles=[0, np.deg2rad(-13.7), 0]
)

GOAL_CENTER_X = 15.98 * 0.5  # m
GOAL_CENTER_Y = -1.68  # m
GOAL_CENTER_Z = 2.49  # m
# GOAL_CENTER_X = in2m(153)  # Claudius's Garage
# GOAL_CENTER_Y = in2m(10)
# GOAL_CENTER_Z = in2m(82)

UPPER_LANDMARK_Y_OFFSET = 0.44  # m
LOWER_LANDMARK_Y_OFFSET = 0.22  # m
GOAL_BOTTOM_Z_OFFSET = 0.76 * 0.5  # m

LANDMARKS_PER_TARGET = 4
LANDMARKS = [
    # Our goal
    Landmark(
        0,
        GOAL_CENTER_X,
        GOAL_CENTER_Y + UPPER_LANDMARK_Y_OFFSET,
        GOAL_CENTER_Z,
    ),  # Top left when facing landmark.
    Landmark(
        1,
        GOAL_CENTER_X,
        GOAL_CENTER_Y - UPPER_LANDMARK_Y_OFFSET,
        GOAL_CENTER_Z,
    ),  # Top right when facing landmark.
    Landmark(
        2,
        GOAL_CENTER_X,
        GOAL_CENTER_Y - LOWER_LANDMARK_Y_OFFSET,
        GOAL_CENTER_Z - GOAL_BOTTOM_Z_OFFSET,
    ),  # Bottom right when facing landmark.
    Landmark(
        3,
        GOAL_CENTER_X,
        GOAL_CENTER_Y + LOWER_LANDMARK_Y_OFFSET,
        GOAL_CENTER_Z - GOAL_BOTTOM_Z_OFFSET,
    ),  # Bottom left when facing landmark.
    # Opposing goal
    Landmark(
        4,
        -GOAL_CENTER_X,
        -GOAL_CENTER_Y - UPPER_LANDMARK_Y_OFFSET,
        GOAL_CENTER_Z,
    ),  # Top left when facing landmark.
    Landmark(
        5,
        -GOAL_CENTER_X,
        -GOAL_CENTER_Y + UPPER_LANDMARK_Y_OFFSET,
        GOAL_CENTER_Z,
    ),  # Top right when facing landmark.
    Landmark(
        6,
        -GOAL_CENTER_X,
        -GOAL_CENTER_Y + LOWER_LANDMARK_Y_OFFSET,
        GOAL_CENTER_Z - GOAL_BOTTOM_Z_OFFSET,
    ),  # Bottom right when facing landmark.
    Landmark(
        7,
        -GOAL_CENTER_X,
        -GOAL_CENTER_Y - LOWER_LANDMARK_Y_OFFSET,
        GOAL_CENTER_Z - GOAL_BOTTOM_Z_OFFSET,
    ),  # Bottom left when facing landmark.
]
OUR_LANDMARK_IDs = [0, 1, 2, 3]
OPPOSING_LANDMARK_IDs = [4, 5, 6, 7]

PARTICLE_LIKELIHOOD_ACCEPT_TOLERANCE = 1.0
FRACTION_OF_PARTICLES_OVER_TOLERANCE = 0.3


class UpdateType(enum.Enum):
    NONE = 0
    OUR_GOAL = 1
    OPPOSING_GOAL = 2


class ParticleFilter:
    def __init__(self, num_particles: int, priori: Odometry):
        self.num_particles = num_particles
        self.landmarks = LANDMARKS  # TODO: make this configurable
        # Keep track of last odometry to compute deltas.
        self.last_odom = priori
        self.initialize()

    def initialize(self):
        # TODO: support reinitialization
        # Represent particles as an Nx4x4 matrix
        # N is the number of particles and each 4x4 matrix is a 3D transformation matrix
        self.particles = np.repeat(
            np.eye(4)[np.newaxis, :, :], self.num_particles, axis=0
        )
        # We need to keep track of the non-wrapping continuous yaw separately, since the 3D rotation matrix loses this info.
        self.particle_continuous_yaw = np.empty((self.num_particles, 1))
        self.particle_weights = np.full(
            self.num_particles, 1.0 / float(self.num_particles)
        )
        self.particle_indexes = np.array(
            range(self.num_particles)
        )  # Used for resampling

        # Priori sigmas
        sigmaX = 5e-2
        sigmaY = 5e-2
        sigmaTheta = 1e-3
        xs = np.random.normal(
            self.last_odom.x, sigmaX, self.num_particles
        )
        ys = np.random.normal(
            self.last_odom.y, sigmaY, self.num_particles
        )
        self.particle_continuous_yaw = np.random.normal(
            self.last_odom.theta, sigmaTheta, self.num_particles
        )
        set_x(self.particles, xs)
        set_y(self.particles, ys)
        set_yaw(self.particles, self.particle_continuous_yaw)

    def predict(self, odometry: Odometry):
        # Compute delta motion
        T = tf.compose_matrix(
            translate=[odometry.x, odometry.y, 0], angles=[0, 0, odometry.theta]
        )
        last_T = tf.compose_matrix(
            translate=[self.last_odom.x, self.last_odom.y, 0],
            angles=[0, 0, self.last_odom.theta],
        )
        self.last_odom = odometry

        M = np.linalg.inv(last_T).dot(T)
        _, _, angles, trans, _ = tf.decompose_matrix(M)
        dx, dy, _ = trans
        _, _, dtheta = angles

        # Precompute gaussian noise to apply to x, y, and theta for each particle.
        xSigma = 3e-1 * abs(dx)
        ySigma = 3e-1 * abs(dy)
        thetaSigma = 1e-6
        xNoise = np.random.normal(
            0.0, xSigma, size=self.num_particles
        )  # Scale uncertainty by distance
        yNoise = np.random.normal(0.0, ySigma, size=self.num_particles)
        thetaNoise = np.random.normal(0.0, thetaSigma, size=self.num_particles)

        # Transform each particle by value + noise.
        transforms = np.repeat(np.eye(4)[np.newaxis, :, :], self.num_particles, axis=0)
        set_x(transforms, dx + xNoise)
        set_y(transforms, dy + yNoise)
        dTheta = dtheta + thetaNoise
        set_yaw(transforms, dTheta)

        self.particles = np.matmul(self.particles, transforms)
        self.particle_continuous_yaw += dTheta

    def update(self, measurements: List[Vision]):
        if measurements is None or len(measurements) != LANDMARKS_PER_TARGET:
            return UpdateType.NONE

        # Use best estimate yaw to determine which goal we are looking at.
        # TODO: This should be done per-particle, but using best estimate is way easier.
        _, _, yaw = self.get_best_estimate()
        if np.cos(yaw) >= 0:  # Our goal is in the +x direction
            landmarks = [LANDMARKS[id] for id in OUR_LANDMARK_IDs]
            update_type = UpdateType.OUR_GOAL
        else:
            landmarks = [LANDMARKS[id] for id in OPPOSING_LANDMARK_IDs]
            update_type = UpdateType.OPPOSING_GOAL

        measurements = self.order_vision(measurements)
        camera_T = np.matmul(self.particles, ROBOT_TO_CAMERA)
        landmark_likelihoods = np.empty((self.num_particles, len(measurements)))
        for i, (measurement, landmark) in enumerate(zip(measurements, landmarks)):
            # Compute the expected coordinate of the landmark in the camera frame
            point = get_point_in_frame(camera_T, [landmark.x, landmark.y, landmark.z])

            # Compare the expected measurement with the actual measurement
            expected_measurement = cart2besph(point[:, 0], point[:, 1], point[:, 2])
            delta_measurement = expected_measurement - np.array(
                [measurement.bearing, measurement.elevation]
            )
            # TODO: Figure out a cleaner way to do this
            delta_measurement_stacked = np.expand_dims(delta_measurement, axis=2)
            delta_measurement_stacked_transpose = np.expand_dims(
                delta_measurement, axis=1
            )

            # Compute the likelihood of the measurement
            beSigma = 5e-2
            elSigma = 5e-2
            covariance_matrix = np.diag(np.array([beSigma, elSigma]))
            repeated_inv_covariance = np.repeat(
                np.linalg.inv(covariance_matrix)[np.newaxis, :, :],
                self.num_particles,
                axis=0,
            )
            scalar_term = 1.0 / np.sqrt(
                ((2 * np.pi) ** 2) * np.linalg.det(covariance_matrix)
            )
            main_term = -0.5 * np.matmul(
                np.matmul(delta_measurement_stacked_transpose, repeated_inv_covariance),
                delta_measurement_stacked,
            )
            landmark_likelihoods[:, i] = np.ndarray.flatten(
                scalar_term * np.exp(main_term)
            )

        # Compute the likelihood of observing all landmarks
        total_likelihoods = np.exp(np.sum(np.log(landmark_likelihoods), axis=1))

        # Check if |FRACTION_OF_PARTICLES_OVER_TOLERANCE| have a likelihood greater than |PARTICLE_LIKELIHOOD_ACCEPT_TOLERANCE|
        over_tolerance = np.greater(
            total_likelihoods, PARTICLE_LIKELIHOOD_ACCEPT_TOLERANCE
        )
        if (
            np.sum(over_tolerance)
            > FRACTION_OF_PARTICLES_OVER_TOLERANCE * self.num_particles
        ):
            # Normalize likelihoods
            total_likelihoods /= np.sum(total_likelihoods)
            # Update particle weight
            self.particle_weights = total_likelihoods

            # Only resample if weights have been updated
            self.resample()
            return update_type
        return UpdateType.NONE

    def resample(self):
        chosen_indices = np.random.choice(
            self.particle_indexes, size=self.num_particles, p=self.particle_weights
        )
        self.particles = self.particles[chosen_indices, :, :]
        self.particle_continuous_yaw = self.particle_continuous_yaw[chosen_indices]
        self.particle_weights = np.full(
            self.num_particles, 1.0 / float(self.num_particles)
        )

    def get_best_estimate(self):
        weighted_x = np.sum(get_x(self.particles) * self.particle_weights)
        weighted_y = np.sum(get_y(self.particles) * self.particle_weights)
        weighted_yaw = np.sum(self.particle_continuous_yaw * self.particle_weights)
        return weighted_x, weighted_y, weighted_yaw

    def plot_particles(self, plt, has_vision=False):
        plt.scatter(
            get_x(self.particles),
            get_y(self.particles),
            s=(self.particle_weights * 100),
            marker=".",
        )
        weighted_x = np.sum(get_x(self.particles) * self.particle_weights)
        weighted_y = np.sum(get_y(self.particles) * self.particle_weights)
        plt.scatter(
            weighted_x, weighted_y, marker="o", color="C2" if has_vision else "C3"
        )
        plt.plot()

    def plot_estimate(self, plt, has_vision=False, facing_our_goal=True):
        x, y, _ = self.get_best_estimate()
        plt.scatter(
            x,
            y,
            s=5,
            marker="o",
            color=("g" if facing_our_goal else "b") if has_vision else "r",
        )
        plt.plot()

    @staticmethod
    def order_vision(measurements: List[Vision]):
        # Order the measuments CW starting with the "top left" measurment.
        points = []
        for measurment in measurements:
            points.append(
                [-measurment.bearing, measurment.elevation]
            )  # Bearing left is positive so it needs to be flipped for ordering.

        points = np.array(points)
        center = points.mean(axis=0)

        # Compute the angle about the centroid for each point.
        # Store (angle, index) tuples.
        angle_index = []
        for index, point in enumerate(points):
            delta = point - center
            angle_index.append((math.atan2(delta[1], delta[0]), index))

        ordered_measurements = []
        # Sorting by negative angle when angles are in the range of [-2*PI, 2*PI] is clockwise from top left.
        for _, index in sorted(angle_index, key=lambda x: -x[0]):
            ordered_measurements.append(measurements[index])

        return ordered_measurements
