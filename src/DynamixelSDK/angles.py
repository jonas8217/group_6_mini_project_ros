from math import asin
from math import pi

l = 19.7
b = 16.4
a = asin((b/2)/l)/pi*180

rot_range = 360 - a*2

steps_per_degree = 1024 / rot_range 

to_rotate = 90

print(steps_per_degree)
print(512 + to_rotate * steps_per_degree)

steps_per_degree = 306 / 90

print(steps_per_degree)
print(512 + to_rotate * steps_per_degree)