#!/usr/bin/env python
"""Measured Angle Calculation Module for Demo-1. SEED Lab: Group 007.

REQUIREMENTS AND COMPATIBILITY:
Requires install of math and numpy.

PURPOSE:
The purpose of this program is to quickly and easily calculate the measured
angle and distance to an Aruco marker in order to compare to values generated
by the ComputerVision.py module.

METHODS:
This program takes user input in z and x distances in feet and uses basic trig
functions to calculate the distance and angle to those coordinates, and then
displays the result.

INSTRUCTIONS:
Simply type the measured distance in the z direction (straight out) from the
camera to the marker in feet and press enter. Then type the measured distance
in the x direction (to the left or right) from the camera to the marker and
press enter.

OUTPUTS:
The program will print its calculations of distance in inches, and angle in
degrees and radians based on the measured input values.
"""

__author__ = "Jack Woolery"
__email__ = "lwoolery@mines.edu"


import math
import numpy as np

Z_dist = input("Z distance (ft): ")
X_dist = input("X distance (ft): ")

Z = float(Z_dist * 12)
Z = Z
X = float(X_dist * 12)

dist = math.sqrt(Z ** 2 + X ** 2)
print("Distance = ", dist, "inches")

angle_rad = np.arctan(X / Z)
angle_deg = angle_rad * 180 / math.pi
print("Angle = ", angle_deg, "degrees;     ", angle_rad, "radians")
