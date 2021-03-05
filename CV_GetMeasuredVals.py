import math
import numpy as np

Z_dist = input("Z distance (in): ")
X_dist = input("X distance (in): ")

Z = float(Z_dist)
X = float(X_dist)

dist = math.sqrt(Z ** 2 + X ** 2)
print("Distance = ", dist, "inches")

angle_rad = np.arctan(X / Z)
angle_deg = angle_rad * 180 / math.pi
print("Angle = ", angle_deg, "degrees;     ", angle_rad, "radians")
