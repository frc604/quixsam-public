import pytest
import logging

from ..data_utils import *
from ..nt_manager import NTManager

logging.basicConfig(level=logging.INFO)


@pytest.fixture(scope="function")
def table_odometry(nt):
    return nt.getTable("quixsam").getSubTable("odometry")


@pytest.fixture(scope="function")
def table_vision(nt):
    return nt.getTable("quixsam").getSubTable("vision")


@pytest.fixture(scope="function")
def table_landmarks(nt):
    return nt.getTable("quixsam").getSubTable("landmarks")


@pytest.fixture(scope="function")
def table_priori(nt):
    return nt.getTable("quixsam").getSubTable("priori")


@pytest.fixture(scope="function")
def table_landmarks(nt):
    return nt.getTable("quixsam").getSubTable("landmarks")


@pytest.fixture(scope="function")
def manager(nt, table_odometry, table_vision, table_priori):
    table_priori.putNumberArray("0", [0, 1, 0, 1, 0, 1, 0])
    table_odometry.putNumberArray("0", [0, 0, 0, 0, 0, 0, 0])
    table_vision.putNumberArray("0", [0, 0, 0, 0, 0, 0])

    return NTManager(nt)


def test_init(nt, table_odometry, table_vision, table_priori):
    table_priori.putNumberArray("0", [0, 1, 0, 1, 0, 1, 0])
    table_odometry.putNumberArray("0", [0, 0, 0, 0, 0, 0, 0])
    table_vision.putNumberArray("0", [0, 0, 0, 0, 0, 0])

    NTManager(nt)


def test_priori(manager):
    assert manager.get_priori() == Odometry(0, 1, 0, 1, 0, 1, 0)


def test_landmarks(manager, table_landmarks):
    table_landmarks.putNumberArray("Upper Right", [0, 1, 1, 1, 0.1, 0.1, 0.1])
    table_landmarks.putNumberArray("Lower Left", [1, -1, -1, 1, 0.2, 0.2, 0.2])

    assert manager.get_landmarks() == [
        Landmark(0, 1, 1, 1, 0.1, 0.1, 0.1),
        Landmark(1, -1, -1, 1, 0.2, 0.2, 0.2),
    ]


def test_update(manager, table_odometry, table_vision):
    table_odometry.putNumberArray("604.0", [0, 1, 2, 3, 0.1, 0.1, 0.1])
    table_odometry.putNumberArray("604.1", [1, 2, 4, 6, 0.2, 0.2, 0.2])

    table_vision.putNumberArray("604.604", [0, 1, 3, 5, 0.6, 0.6])
    table_vision.putNumberArray("604.605", [1, 6, 8, 10, 0.9, 0.9])

    manager.update()

    assert manager.odometry_buffer == [
        Odometry(0, 1, 2, 3, 0.1, 0.1, 0.1),
        Odometry(1, 2, 4, 6, 0.2, 0.2, 0.2),
    ]
    assert manager.vision_buffer == [
        Vision(0, 1, 3, 5, 0.6, 0.6),
        Vision(1, 6, 8, 10, 0.9, 0.9),
    ]
