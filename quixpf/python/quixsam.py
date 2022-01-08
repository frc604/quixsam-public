import logging
from nt_manager import NTManager
from multiprocessing import Process
from particle_filter import ParticleFilter, UpdateType
from plotter import Plotter
import time

NUM_PARTICLES = 1000


class Quixsam:
    def __init__(self, server=None):
        self.server = server

    def run(self):
        # Start plotter thread
        plotter = Plotter(NUM_PARTICLES)
        plotter_queue = plotter.start()

        # Wait for first odometry to come in to start particle filter
        nt_manager = NTManager(self.server)

        pf = None
        while pf is None:
            logging.info("Waiting for first odometry...")
            odometry, _ = nt_manager.get_next()
            if odometry is not None:
                pf = ParticleFilter(NUM_PARTICLES, odometry)
                logging.info("First odometry received!")
            time.sleep(0.1)

        while True:
            odometry, vision = nt_manager.get_next()
            if odometry is not None:
                s = time.time()
                pf.predict(odometry)
                update_type = pf.update(vision)

                # Publish estimate
                x, y, theta = pf.get_best_estimate()
                nt_manager.publish_estimate(odometry.id, x, y, theta)

                # Handle plotting
                has_vision = update_type != UpdateType.NONE
                facing_our_goal = update_type == UpdateType.OUR_GOAL
                plotter_queue.put(
                    (pf.particles, pf.get_best_estimate(), has_vision, facing_our_goal)
                )
                if odometry.id % 5 == 0:
                    logging.info(
                        "Iter {} time: {:.3f} {}".format(
                            odometry.id,
                            time.time() - s,
                            "VISION" if has_vision else "",
                        )
                    )
            time.sleep(0.001)


if __name__ == "__main__":
    Quixsam("10.6.4.2").run()
