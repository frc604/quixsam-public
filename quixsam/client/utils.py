from typing import List

import gtsam
import numpy as np


epsilon = 1e-6

def vector2(x: float, y: float):
    """Create 2d double numpy array."""
    return np.array([x, y], dtype=np.float)


def vector3(x: float, y: float, z: float):
    """Create 3d double numpy array."""
    return np.array([x, y, z], dtype=np.float)


def make_noise_model(noise: List[float]):
    """Creates an diagonal noise model who's size is that of the array provided."""
    return gtsam.gtsam.noiseModel.Diagonal.Sigmas(np.array(noise, dtype=np.float))


def besph2cart(bearing: float, elevation: float):
    """Converts a bearing and elevation in spherical coordinates to a 3D cartesian point on the unit circle."""
    return np.array(
        [
            np.cos(bearing) * np.cos(elevation),
            np.sin(bearing) * np.cos(elevation),
            np.sin(elevation),
        ]
    )


def cart2besph(x: float, y: float, z: float):
    """Converts the direction of a 3D cartesian point to a bearing and elevation in spherical coordinates."""
    return np.array([np.arctan2(y, x), np.arctan2(z, np.sqrt(np.power(x, 2) + np.power(y, 2)))])
