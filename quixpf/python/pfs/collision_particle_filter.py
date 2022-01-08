from dataclasses import dataclass, field

from matplotlib.patches import Polygon
from numpy.core.defchararray import array
from helpers import cart2besph, dynamics_model, plot_robot, get_point_relative_to_robot
import numpy as np
import matplotlib.pyplot as plt
import cProfile

# SIR Particle Filter

NUMBER_OF_PARTICLES = 1000

landmarks = np.array([[10, 0, 0]])

priori = np.array([0, 0, 0])  # x, y, theta
priori_sigma = np.array([0.1, 0.1, 0.1])  # x, y, theta

particles = np.empty((NUMBER_OF_PARTICLES, 4))  # x, y, theta, weight


@dataclass
class BaseConvexPolytope:
    xy: np.ndarray = field(default = None)
    V: np.ndarray = field(default = None)
    A: np.ndarray = field(default = None)
    b: np.ndarray = field(default = None)

    def check_intersection(self, other: 'BaseConvexPolytope'):
        # Check if any of the this's vertices are in other.
        for vertex in self.V:
            if all(other.A.dot(vertex) <= other.b):
                return False

        # Next check if any of the other's vertices are in  this.
        for vertex in other.V:
            if all(self.A.dot(vertex) <= self.b):
                return False
        return True


@dataclass
class RectanglePolytope(BaseConvexPolytope):
    xc: float = field(default = 0.0)
    yc: float = field(default = 0.0)
    w: float = field(default = 0.0)
    h: float = field(default = 0.0)
    theta: float = field(default = 0.0)

    def __post_init__(self):
        self.xy = np.array([[- self.w / 2, - self.h / 2], [- self.w / 2, + self.h / 2], [+ self.w / 2, + self.h / 2],
                           [+ self.w / 2, - self.h / 2], [- self.w / 2, - self.h / 2]]) @ self.R() + np.array([[self.xc, self.yc]])

        self.V = self.xy[:-1,:]

        self.A = np.array([[1, 0],
                       [0, 1],
                       [-1, 0],
                       [0, -1]]) @ self.R()

        A = np.eye(2) @ self.R()
        l = np.linalg.solve(A.T, np.array([[-self.xc], [-self.yc]])).squeeze() + np.array([self.w / 2, self.h / 2])
        u = np.linalg.solve(A.T, np.array([[self.xc], [self.yc]])).squeeze() + np.array([self.w / 2, self.h / 2])
        self.b = np.concatenate([u, l])

    def R(self):
        return np.array([[np.cos(self.theta), np.sin(self.theta)], [-np.sin(self.theta), np.cos(self.theta)]])


field = [
    RectanglePolytope(xc=0, yc=25, w=40, h=10, theta=0.0),
    RectanglePolytope(xc=0, yc=-25, w=40, h=10, theta=0.0),
    RectanglePolytope(xc=25, yc=0, w=10, h=40, theta=0.0),
    RectanglePolytope(xc=-25, yc=0, w=10, h=40, theta=0.0)

]

def initialize():
    # Init a random distribution of parircles based on priori std devs
    particles[:, 0] = np.random.normal(priori[0], priori_sigma[0], NUMBER_OF_PARTICLES)
    particles[:, 1] = np.random.normal(priori[1], priori_sigma[1], NUMBER_OF_PARTICLES)
    particles[:, 2] = np.random.normal(priori[2], priori_sigma[2], NUMBER_OF_PARTICLES)
    particles[:, 3] = np.full(NUMBER_OF_PARTICLES, 1.0 / float(NUMBER_OF_PARTICLES))


def predict(dt, state_sigma, u):
    # Update State with dynamics model
    for particle in particles:
        state = particle[0:3] + (dynamics_model(particle[0:3], u) * dt)

        # Add gaussian noise to update particles
        particle[0] = np.random.normal(state[0], state_sigma[0])
        particle[1] = np.random.normal(state[1], state_sigma[1])
        particle[2] = np.random.normal(state[2], state_sigma[2])


# Currently assuming only a single landmark
def update(landmark_id, measurement, measurement_sigma):
    landmark = landmarks[landmark_id]

    likelihoods = []

    for particle in particles:
        # Calculate bearing/elevation to target relative to the particle (robot)
        x, y, z = get_point_relative_to_robot(particle[:3], landmark)
        estimated_measurement = cart2besph(x, y, z)

        covariance_matrix = np.diag(measurement_sigma)
        delta_measurement = estimated_measurement - measurement

        scalar_term = (1.0 / np.sqrt(((2 * np.pi) ** 2) * np.linalg.det(covariance_matrix)))
        main_term = -0.5 * np.transpose(delta_measurement).dot(np.linalg.inv(covariance_matrix)).dot(delta_measurement)

        likelihood = scalar_term * np.exp(main_term)
        for polytope in field:
            if not polytope.check_intersection(RectanglePolytope(xc=particle[0], yc=particle[1], w=1, h=1, theta=particle[2])):
                likelihood = 0.0

        likelihoods.append(likelihood)

    likelihoods = np.array(likelihoods)

    # Normalize likelihoods
    likelihoods /= np.sum(likelihoods)

    # Update particle weight
    particles[:, 3] = likelihoods


def resample():
    temp_particles = np.copy(particles)
    temp_likelihoods = temp_particles[:, 3]

    particle_indexes = np.array(range(NUMBER_OF_PARTICLES))

    for i in range(0, NUMBER_OF_PARTICLES):
        chosen_index = np.random.choice(particle_indexes, p=temp_likelihoods)
        particles[i] = temp_particles[chosen_index]


fig, ax = plt.subplots(dpi=600)


def plot_particles():
    # for particle in particles:
    #     plot_robot(ax, particle[0:3])
    plt.scatter(particles[:, 0], particles[:, 1], s=(particles[:, 3] * 100), marker='.')


plt.axis('equal')

initialize()

plot_particles()


def run():
    x = 0
    for i in range(1, 7):
        predict(1, [2, 2, 0.01], [i, 0])

        landmark = landmarks[0]

        x += i
        pose = (x, 0, 0)

        # Simulate measurement
        mx, my, mz = get_point_relative_to_robot(pose, landmark)
        # print("Ground Truth: {}".format(cart2besph(mx, my, mz)))

        update(0, cart2besph(mx, my, mz), np.array([0.001, 0.001]))
        resample()

        plot_particles()


run()

best_particle = np.zeros(4)
for particle in particles:
    if particle[3] > best_particle[3]:
        best_particle = particle
print(best_particle)

for polytope in field:
    ax.add_patch(Polygon(polytope.xy))

x = RectanglePolytope(xc = 0.0, yc = 0.0, w = 10, h = 10, theta = 0.0)

plt.show()
