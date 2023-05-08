import math
import numpy as np

def rotation_matrix_to_euler_angles(rot_matrix):
    """Convert a 3x3 rotation matrix to a yaw-pitch-roll vector in radians."""
    
    # Extract the yaw (y), pitch (p), and roll (r) angles from the rotation matrix
    # using the conventions of the aerospace industry
    y = math.atan2(rot_matrix[1,0], rot_matrix[0,0])
    p = math.atan2(-rot_matrix[2,0], math.sqrt(rot_matrix[2,1]**2 + rot_matrix[2,2]**2))
    r = math.atan2(rot_matrix[2,1], rot_matrix[2,2])

    # Convert the yaw, pitch, and roll angles to degrees if desired
    y = math.degrees(y)
    p = math.degrees(p)
    r = math.degrees(r)
    
    return np.array([y, p, r])

