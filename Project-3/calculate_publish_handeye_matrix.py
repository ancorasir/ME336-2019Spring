#!/usr/bin/env python2

import rospy
from tf import TransformBroadcaster, TransformerROS, transformations as tfs
import tf
from geometry_msgs.msg import Transform
import numpy as np

rospy.init_node('handeye_calibration_publisher')
print("Publishing handeye matrix!")
while rospy.get_time() == 0.0:
    pass

d = np.load("calibration_data.npz")
observed_pts = d['arr_0']
measured_pts = d['arr_1']
def get_rigid_transform(A, B):
    assert len(A) == len(B)
    N = A.shape[0]; # Total points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - np.tile(centroid_A, (N, 1)) # Centre the points
    BB = B - np.tile(centroid_B, (N, 1))
    H = np.dot(np.transpose(AA), BB) # Dot is matrix multiplication for array
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0: # Special reflection case
       Vt[2,:] *= -1
       R = np.dot(Vt.T, U.T)
    t = np.dot(-R, centroid_A.T) + centroid_B.T
    return R, t

R, t = get_rigid_transform(observed_pts, measured_pts)

# q = tf.transformations.quaternion_from_matrix(R)
al, be, ga = tf.transformations.euler_from_matrix(R, 'sxyz')
q = tf.transformations.quaternion_from_euler(al, be, ga, axes='sxyz')

broad = TransformBroadcaster()

rate = rospy.Rate(50)

while not rospy.is_shutdown():
    broad.sendTransform((t[0],t[1],t[2]), (q[0],q[1],q[2],q[3]), rospy.Time.now(), "camera_color_optical_frame", "base_link")  # takes ..., child, parent
    rate.sleep()
