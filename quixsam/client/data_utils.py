from .utils import *
from dataclasses import dataclass


@dataclass
class Landmark:
    id: int
    x: float
    y: float
    z: float
    sigmaX: float
    sigmaY: float
    sigmaZ: float

    def getNoiseModel(self):
        return make_noise_model([self.sigmaX, self.sigmaY, self.sigmaZ])


@dataclass
class Vision:
    id: int
    landmarkID: int
    bearing: float
    elevation: float
    sigmaBearing: float
    sigmaElevation: float

    def getNoiseModel(self):
        return make_noise_model([self.sigmaBearing, self.sigmaElevation])

    def getUnit3(self):
        return gtsam.Unit3(besph2cart(self.bearing, self.elevation))


@dataclass
class Odometry:
    id: int
    x: float
    y: float
    theta: float
    sigmaX: float
    sigmaY: float
    sigmaTheta: float

    def getNoiseModel(self):
        return make_noise_model([epsilon, epsilon, self.sigmaTheta, self.sigmaX, self.sigmaY, epsilon])

    def getPose3(self):
        return gtsam.Pose3(gtsam.Pose2(self.x, self.y, self.theta))

    def compose(self, delta):
        composed_pose = self.getPose3().compose(delta.getPose3())

        return Odometry(
            self.id + delta.id,
            composed_pose.x(),
            composed_pose.y(),
            composed_pose.rotation().yaw(),
            delta.sigmaX,
            delta.sigmaY,
            delta.sigmaTheta,
        )

@dataclass
class PublishW:
    id: int
    x: float
    y: float
    theta: float

    def getPose3(self):
        return gtsam.Pose3(gtsam.Pose2(self.x, self.y, self.theta))
