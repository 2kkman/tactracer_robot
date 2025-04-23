import math

# Given distance to the 270-degree table
distance_270 = 700  # mm

# The angles for the 240-degree and 300-degree tables in radians
angle_240 = math.radians(240)
angle_300 = math.radians(305)
angle_315 = math.radians(315)

# Using trigonometric functions to calculate the distances
# Distance for 240 degrees and 300 degrees can be found using the cosine rule
distance_240 = distance_270 / math.cos(angle_240 - math.radians(270))
distance_300 = distance_270 / math.cos(angle_300 - math.radians(270))
distance_315 = distance_270 / math.cos(angle_315 - math.radians(270))

print(distance_240, distance_300, distance_315)