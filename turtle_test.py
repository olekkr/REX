import time
from turtle import down, forward, goto, home, left, right, setheading, tracer, up

import numpy as np


def move_turtle(angle, distance):
    right(angle)

    forward(distance * 100)


lst = [
    (np.float64(-36.80571167292689), np.float64(0.2)),
    (np.float64(58.89421752175005), np.float64(0.2)),
    (np.float64(-20.444657744547868), np.float64(0.2000000000000001)),
    (np.float64(-47.93000314264262), np.float64(0.19999999999999984)),
    (np.float64(5.116211718925712), np.float64(0.19999999999999993)),
    (np.float64(12.862189883131194), np.float64(0.19999999999999998)),
    (np.float64(3.395849962523215), np.float64(0.20000000000000015)),
    (np.float64(8.673672630153717), np.float64(0.19999999999999996)),
    (np.float64(63.050237908879595), np.float64(0.2000000000000002)),
    (np.float64(-277.32882658809984), np.float64(0.19999999999999996)),
    (np.float64(255.39879086781542), np.float64(0.20000000000000037)),
    (np.float64(-291.53260384503125), np.float64(0.20000000000000004)),
    (np.float64(308.1409785666478), np.float64(0.20000000000000015)),
    (np.float64(-95.83319887053788), np.float64(0.16291307836946559)),
    (np.float64(34.07832811155856), np.float64(0.09820695976609466)),
]

lst2 = [
    np.array([0, 0]),
    np.array([-0.11982068, 0.16013433]),
    np.array([-0.04461301, 0.34545515]),
    np.array([-0.03887568, 0.54537284]),
    np.array([-0.18343572, 0.68358426]),
    np.array([-0.31509465, 0.83413633]),
    np.array([-0.40993612, 1.01021896]),
    np.array([-0.49418097, 1.19161027]),
    np.array([-0.55010733, 1.38363173]),
    np.array([-0.40428492, 1.5205106]),
    np.array([-0.24992266, 1.39334026]),
    np.array([-0.16577259, 1.57477555]),
    np.array([0.03388578, 1.56309071]),
    np.array([0.16638455, 1.71290418]),
    np.array([0.03401446, 1.8078717]),
    [0, 1.9],
]

for x, y in lst2:
    down
    goto(x * 100, y * 100)
    up
time.sleep(1)
up
home()
down
setheading(90)
for ang, dist in lst:
    move_turtle(ang, dist)
while 1:
    time.sleep(1)
