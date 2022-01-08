from ..data_utils import *


def test_besph2cart():
    assert np.allclose(
        besph2cart(-np.pi / 4, np.pi / 4), np.array([0.5, -0.5, np.sqrt(2) / 2]), 1e-4
    )
    assert np.allclose(
        besph2cart(np.pi / 3, np.pi / 6), np.array([np.sqrt(3) / 4, 0.75, 0.5]), 1e-4
    )


def test_cart2besph():
    assert np.allclose(
        cart2besph(0.5, -0.5, np.sqrt(2) / 2), np.array([-np.pi / 4, np.pi / 4]), 1e-4
    )
    assert np.allclose(
        cart2besph(np.sqrt(3) / 4, 0.75, 0.5), np.array([np.pi / 3, np.pi / 6]), 1e-4
    )


def test_data_landmark():
    assert (
        Landmark(0, 1, 1, 1, 0.1, 0.1, 0.1)
        .getNoiseModel()
        .equals(
            gtsam.gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1], dtype=np.float)),
            1e-9,
        )
    )


def test_data_vision():
    assert (
        Vision(0, 0, 1, 1, 0.1, 0.1)
        .getNoiseModel()
        .equals(
            gtsam.gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1], dtype=np.float)),
            1e-9,
        )
    )


def test_data_odometry():
    assert (
        Odometry(0, 1, 1, 1, 0.1, 0.1, 0.1)
        .getNoiseModel()
        .equals(
            gtsam.gtsam.noiseModel.Diagonal.Sigmas(
                np.array([epsilon, epsilon, 0.1, 0.1, 0.1, epsilon], dtype=np.float)
            ),
            1e-9,
        )
    )

def test_odometry_compose():
    print(Odometry(0, 1, 1, 1, 0.1, 0.1, 0.1).compose(Odometry(1, 1, 1, 1, 0.1, 0.1, 0.1)).x)
    print(Odometry(0, 1, 1, 1, 0.1, 0.1, 0.1).compose(Odometry(1, 1, 1, 1, 0.1, 0.1, 0.1)).y)
    print(Odometry(0, 1, 1, 1, 0.1, 0.1, 0.1).compose(Odometry(1, 1, 1, 1, 0.1, 0.1, 0.1)).theta)
    print(Odometry(0, 1, 1, 1, 0.1, 0.1, 0.1).compose(Odometry(1, 1, 1, 1, 0.1, 0.1, 0.1)).sigmaX)
    print(Odometry(0, 1, 1, 1, 0.1, 0.1, 0.1).compose(Odometry(1, 1, 1, 1, 0.1, 0.1, 0.1)).sigmaY)
    print(Odometry(0, 1, 1, 1, 0.1, 0.1, 0.1).compose(Odometry(1, 1, 1, 1, 0.1, 0.1, 0.1)).sigmaTheta)


