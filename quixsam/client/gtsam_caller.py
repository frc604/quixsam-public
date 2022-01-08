import gtsam_unstable

from .data_utils import *

from gtsam.symbol_shorthand import L
from gtsam.symbol_shorthand import X


class GTSAMCaller:
    def __init__(self, priori: Odometry, landmarks: List[Landmark]):
        self.priori = priori
        self.landmarks = landmarks

        self.previous_measurement = priori
        self.latest_pose_estimate = priori.getPose3()

        self.parameters = gtsam.ISAM2Params()

        self.reset(priori, landmarks)

    def update(self, odometry: List[Odometry], vision: List[Vision]):
        for measurement in odometry:
            pose = measurement.getPose3()
            id_ = X(measurement.id)

            pose_delta = self.previous_measurement.getPose3().between(pose)
            self.latest_pose_estimate = self.latest_pose_estimate.compose(pose_delta)
            self.estimate.insert(id_, self.latest_pose_estimate)
            self.graph.add(
                gtsam.BetweenFactorPose3(
                    X(self.previous_measurement.id),
                    id_,
                    pose_delta,
                    measurement.getNoiseModel(),
                )
            )

            self.previous_measurement = measurement
            self.graph.add(
                gtsam.Pose3AttitudeFactor(
                    id_, gtsam.Unit3(np.array([0, 0, -1])), make_noise_model([epsilon, epsilon])
                )
            )
            self.graph.add(
                gtsam_unstable.RelativeElevationFactor(id_, L(0), -10, make_noise_model([epsilon]))
            )

        for measurement in vision:
            self.graph.add(
                gtsam.BearingFactor3D(
                    X(measurement.id),
                    L(measurement.landmarkID),
                    measurement.getUnit3(),
                    measurement.getNoiseModel(),
                )
            )

    def solve(self):
        self.isam.update(self.graph, self.estimate)
        latest_estimate = self.isam.calculateEstimate()
        self.latest_pose_estimate = latest_estimate.atPose3(X(self.previous_measurement.id))

        self.graph.resize(0)
        self.estimate.clear()
        return latest_estimate

    def reset(self, priori: Odometry = None, landmarks: List[Landmark] = None):
        if landmarks is None:
            landmarks = self.landmarks
        if priori is None:
            priori = self.priori

        self.isam = gtsam.ISAM2(self.parameters)

        self.graph = gtsam.NonlinearFactorGraph()
        self.estimate = gtsam.Values()

        for landmark in landmarks:
            id_ = L(landmark.id)
            position = gtsam.Point3(landmark.x, landmark.y, landmark.z)

            self.estimate.insert(id_, position)
            self.graph.add(gtsam.PriorFactorPoint3(id_, position, landmark.getNoiseModel()))

        priori_pose = priori.getPose3()
        self.estimate.insert(X(0), priori_pose)
        self.graph.add(gtsam.PriorFactorPose3(X(0), priori_pose, priori.getNoiseModel()))
