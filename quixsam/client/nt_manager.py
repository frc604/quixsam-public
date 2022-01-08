import time
import logging

from .data_utils import *

from networktables import NetworkTables

logger = logging.getLogger("NetworkTablesManager")


class NTManager:
    landmarks = []
    vision_buffer = []
    odometry_buffer = []

    def __init__(self, network_tables: NetworkTables):
        self.root_table = network_tables.getTable("quixsam")

        self.vision_table = self.root_table.getSubTable("vision")
        self.odometry_table = self.root_table.getSubTable("odometry")
        self.landmarks_table = self.root_table.getSubTable("landmarks")

        logger.info("Waiting for zero odometry entry.")
        while not np.array_equal(np.asarray(self.odometry_table.getEntry("0").value), [0.0] * 7):
            time.sleep(0.1)

        logger.info("Successfully received zero entry!")
        self.odometry_table.getEntry("0").delete()

        logger.info("Waiting for zero vision entry.")
        while not np.array_equal(np.asarray(self.vision_table.getEntry("0").value), [0.0] * 6):
            time.sleep(0.1)

        logger.info("Successfully received zero entry!")
        self.vision_table.getEntry("0").delete()

        self.prioriTable = self.root_table.getSubTable("priori")

        self.estimatesTable = self.root_table.getSubTable("estimates")

    def get_landmarks(self):
        for key in self.landmarks_table.getKeys():
            try:
                value = self.landmarks_table.getEntry(key).value
                if all(np.isfinite(value)):
                    self.landmarks.append(
                        Landmark(
                            int(value[0]),
                            value[1],
                            value[2],
                            value[3],
                            value[4],
                            value[5],
                            value[6],
                        )
                    )
                else:
                    raise RuntimeError("Invalid Landmark: " + str(int(value[0])))
            except ValueError:
                raise RuntimeError("Invalid Landmark!")
        return self.landmarks

    def get_priori(self):
        value = self.prioriTable.getEntry("0").value
        if (value is None) or (not all(np.isfinite(value))):
            raise RuntimeError("Invalid Priori!")
        else:
            return Odometry(
                int(value[0]),
                value[1],
                value[2],
                value[3],
                value[4],
                value[5],
                value[6],
            )

    def update(self):
        for key in self.odometry_table.getKeys():
            try:
                value = self.odometry_table.getEntry(key).value
                id_ = int(value[0])
                if np.isfinite(id_) and all(np.isfinite(value)):
                    self.odometry_buffer.append(
                        Odometry(
                            id_,
                            value[1],
                            value[2],
                            value[3],
                            value[4],
                            value[5],
                            value[6],
                        )
                    )
                else:
                    logging.critical("Invalid Odometry Data: " + str(key))
                    pass
            except ValueError:
                logging.critical("Invalid Odometry Data!")
                pass
            self.odometry_table.getEntry(key).delete()

        for key in self.vision_table.getKeys():
            try:
                value = self.vision_table.getEntry(key).value
                id_ = int(value[0])
                if np.isfinite(id_) and all(np.isfinite(value)):
                    self.vision_buffer.append(
                        Vision(id_, int(value[1]), value[2], value[3], value[4], value[5])
                    )
                else:
                    pass
            except ValueError:
                logging.critical("Invalid Vision Data: " + str(time))
                pass
            self.vision_table.getEntry(key).delete()

    def publishPose(self, pose: Pose3):
        