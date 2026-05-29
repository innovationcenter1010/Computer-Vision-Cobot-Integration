import numpy as np


class HomographyTranslator:
    """
    Translates image pixel coordinates into real-world coordinates.
    """

    def __init__(self, image_points, real_world_points):
        self.image_points = np.array(image_points, dtype=float)
        self.real_world_points = np.array(real_world_points, dtype=float)

        self.homography_matrix = self.compute_homography(
            self.image_points,
            self.real_world_points
        )

    def compute_homography(self, image_points, real_world_points):
        """
        Computes a 3x3 homography matrix from 4 image points
        and 4 matching real-world points.
        """

        if len(image_points) != 4 or len(real_world_points) != 4:
            raise ValueError("Homography requires exactly 4 image points and 4 real-world points.")

        A = []

        for i in range(4):
            x, y = image_points[i]
            X, Y = real_world_points[i]

            A.append([-x, -y, -1, 0, 0, 0, x * X, y * X, X])
            A.append([0, 0, 0, -x, -y, -1, x * Y, y * Y, Y])

        A = np.array(A)

        _, _, vh = np.linalg.svd(A)

        h = vh[-1, :]
        H = h.reshape(3, 3)
        H = H / H[2, 2]

        return H

    def translate_point(self, x, y):
        """
        Translates one image coordinate into one real-world coordinate.
        """

        image_point = np.array([x, y, 1.0])

        real_point = self.homography_matrix @ image_point
        real_point = real_point / real_point[2]

        return real_point[0], real_point[1]

    def translate_layer(self, layer):
        """
        Adds real_x and real_y to each Can object in layer.order.
        """

        if not hasattr(layer, "order"):
            raise AttributeError("Layer object must have 'order'. Use OrderedLayer before applying homography.")

        for can in layer.order:
            real_x, real_y = self.translate_point(can.center_x, can.center_y)
            can.real_x = real_x
            can.real_y = real_y

        return layer