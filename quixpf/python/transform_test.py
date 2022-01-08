from helpers import *

ROBOT_TO_CAMERA = tf.compose_matrix(translate=[in2m(11.1), 0, in2m(34.6)], angles=[0, np.deg2rad(-13.7), 0])

x, y, theta = 20.0, 1.0, 1.0

T = tf.compose_matrix(translate=[x, y, 0], angles=[0, 0, theta])

print(T)

mx, my, mz = get_point_in_frame(T.dot(ROBOT_TO_CAMERA), [0.0, 15.0, 10.0])
be, ev = cart2besph(mx, my, mz)

print("mx:", mx, "my:", my, "mz:", mz)

print(be)
print(ev)