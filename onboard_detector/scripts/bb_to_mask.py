import numpy as np

def points_in_box(box, points):
    corners = box.corners()

    p1 = corners[:, 0]
    p_x = corners[:, 4]
    p_y = corners[:, 2]
    p_z = corners[:, 1]

    i = p_x - p1
    j = p_y - p1
    k = p_z - p1

    v = points - p1.reshape((-1, 1))

    iv = np.dot(i, v)
    jv = np.dot(j, v)
    kv = np.dot(k, v)

    mask_x = np.logical_and(0 <= iv, iv <= np.dot(i, i))
    mask_y = np.logical_and(0 <= jv, jv <= np.dot(j, j))
    mask_z = np.logical_and(0 <= kv, kv <= np.dot(k, k))
    mask = np.logical_and(np.logical_and(mask_x, mask_y), mask_z)

    return mask

class BBox:
    def __init__(self, center, size, orientation):
        self.center = np.array(center)
        self.wlh = np.array(size)
        self.orientation = orientation  # heading in radians, clockwise (+)

    def corners(self,
                width_inflation: float = 1.25,
                length_inflation: float = 1.25, 
                height_inflation: float = 1):
        wlh = self.wlh * np.array([width_inflation, length_inflation, height_inflation]) / 2

        x_signs = [-1, 1]
        y_signs = [-1, 1]
        z_signs = [-1, 1]
        signs = np.array(np.meshgrid(x_signs, y_signs, z_signs)).T.reshape(-1, 3)

        corners = signs * wlh 
        theta = self.orientation
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,              0,             1]
        ])

        # Apply rotation and translation
        rotated = corners @ R.T
        translated = rotated + self.center

        return translated.T  # shape: (3, 8)