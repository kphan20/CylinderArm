import numpy as np

# CONSTANTS
d4 = 20
d2_min, d2_max = 0, 400
d3_min, d3_max = 0, 200
e, f = -np.pi / 2, np.pi / 2


def rad_to_deg(rads):
    return rads * 180 / np.pi


def forward(theta1, d2, d3, theta4):
    # Non - matrix
    x = d3 * np.cos(theta1) + d4 * np.cos(theta4) * np.cos(theta1)
    y = d3 * np.sin(theta1) + d4 * np.cos(theta4) * np.sin(theta1)
    z = d2 + d4 * np.sin(theta4)
    print(x, y, z)
    return (x, y, z)


def forward2(theta1, d2, d3, theta4, theta5):
    c1 = np.cos(theta1)
    s1 = np.sin(theta1)
    t01 = np.array([[c1, -s1, 0, 0], [s1, c1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    t12 = np.eye(4)
    t12[2, 3] = d2
    t23 = np.eye(4)
    t23[0, 3] = d3
    c4 = np.cos(theta4)
    s4 = np.sin(theta4)
    t34 = np.array([[c4, 0, s4, 0], [0, 1, 0, 0], [-s4, 0, c4, 0], [0, 0, 0, 1]])
    c5 = np.cos(theta5)
    s5 = np.sin(theta5)
    t45 = np.array([[1, 0, 0, -d4], [0, c5, -s5, 0], [0, s5, c5, 0], [0, 0, 0, 1]])
    return t01 @ t12 @ t23 @ t34 @ t45
    # return t45 @ t34 @ t23 @ t12 @ t01


def inverse(x, y, z, wrist_down=True):
    theta1 = np.arctan2(y, x)

    r2 = x**2 + y**2

    d3 = d3_max if r2 > d3_max**2 else r2**0.5
    if d3 < d3_min:
        print("Invalid d3")
        return

    i = d3 * np.cos(theta1)
    j = d4 * np.cos(theta1)
    k = d3 * np.sin(theta1)
    l = d4 * np.sin(theta1)

    a = j**2 + l**2
    b = 2 * (i * j + k * l)
    c = i**2 + k**2 - r2

    c4 = (-b + (b**2 - 4 * a * c) ** 0.5) / (2 * a)
    if c4 < 0:
        print("negative c4")
        return

    if c4 > 1:
        print("c4 is too large")
        return

    theta4 = np.arccos(c4)
    if (wrist_down and theta4 > 0) or (not wrist_down and theta4 < 0):
        theta4 *= -1

    d2 = z - d4 * np.sin(theta4)
    if d2 < d2_min or d2 > d2_max:
        setting_name = '"down"' if wrist_down else '"up"'
        print(
            f"Setting wrist {setting_name} didn't yield valid configuration. Trying other setting"
        )
        theta4 *= -1
        d2 = z - d4 * np.sin(theta4)
        if d2 < d2_min or d2 > d2_max:
            print("both d2 attempts out of range")
            return

    print(rad_to_deg(theta1), d2, d3, rad_to_deg(theta4))

    return (theta1, d2, d3, theta4)


EPSILON = 1e-5


def extract_euler_angles(matrix: np.ndarray) -> tuple:
    """
    Extract the Euler angles from a 4x4 homogeneous transformation matrix, using an
    intrinsic z-y'-x" convention
    :param matrix: 4x4 homogeneous transformation matrix
    :return: Tuple of Euler angles (in degrees) in the format (psi, theta, phi)
    """
    euler_angles = None

    # ================================== YOUR CODE HERE ==================================
    if np.abs(np.abs(matrix[2, 0]) - 1) > EPSILON:
        theta = -np.arcsin(matrix[2, 0])
        cos_theta = np.cos(theta)
        psi = np.arctan2(matrix[2, 1] / cos_theta, matrix[2, 2] / cos_theta)
        phi = np.arctan2(matrix[1, 0] / cos_theta, matrix[0, 0] / cos_theta)
    else:
        phi = 0
        if matrix[2, 0] < 0:
            theta = np.pi / 2
            psi = np.arctan2(matrix[0, 1], matrix[0, 2])
        else:
            theta = -np.pi / 2
            psi = np.arctan2(-matrix[0, 1], -matrix[0, 2])

    euler_angles = (phi, theta, psi)
    # ====================================================================================

    return tuple(angle for angle in euler_angles)


def turn_zero(x):
    return 0 if abs(x) < EPSILON else x


def inverse2(
    x, y, z, y_rot, x_rot
):  # cannot do z rotation yet, intrinsic y and x notation
    if abs(y) < EPSILON:
        y = 0
    if abs(x) < EPSILON:
        x = 0
    theta1 = np.arctan2(y, x)
    print(theta1)

    if not (e <= y_rot <= f):
        print("invalid y rotation")
        return

    theta4 = y_rot

    r2 = x**2 + y**2

    i = np.cos(theta1)
    j = d4 * np.cos(theta1) * np.cos(theta4)
    k = np.sin(theta1)
    l = d4 * np.sin(theta1) * np.cos(theta4)

    a = i**2 + k**2
    b = 2 * (i * j + k * l)
    c = j**2 + l**2 - r2

    disc = (b**2 - 4 * a * c) ** 0.5
    d3 = (-b - disc) / (2 * a)

    d3 = turn_zero(d3)

    if d3 < d3_min or d3 > d3_max:
        d3 = (-b + disc) / (2 * a)
        if d3 < d3_min or d3 > d3_max:
            print("no valid d3 found")
            return

    d2 = z - d4 * np.sin(theta4)
    if d2 < d2_min or d2 > d2_max:
        print("no valid d2 found")

    print(rad_to_deg(theta1), d2, d3, rad_to_deg(theta4), rad_to_deg(x_rot))

    return (theta1, d2, d3, theta4, x_rot)


theta1 = np.pi / 20
d2 = 0
d3 = 0
theta4 = np.pi / 2
print(rad_to_deg(theta1), d2, d3, rad_to_deg(theta4))

res = inverse(*forward(theta1, d2, d3, theta4))
if res is not None:
    forward(*res)

# june 3rd is the first, eric comes back 8th or 9th
print()
bruh = forward2(theta1, d2, d3, theta4, np.pi)
extracted = extract_euler_angles(bruh)
print(bruh)
print(extracted)
inv = inverse2(*tuple(np.array(bruh[:3, 3])), extracted[1], extracted[2])
if inv is not None:
    print(forward2(*inv))
    print(extract_euler_angles(forward2(*inv)))
