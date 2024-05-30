import numpy as np

class QuaternionAboutAxis:
    @staticmethod
    def quaternion_about_axis(angle, axis):
        axis = np.array(axis)
        axis = axis / np.linalg.norm(axis)
        theta = np.array(angle)
        quat = np.zeros(4)
        quat[0] = np.cos(theta / 2.0)
        quat[1:] = axis * np.sin(theta / 2.0)

        return quat