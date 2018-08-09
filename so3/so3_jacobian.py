import numpy as np


def skew_symmetric(v):
    return np.matrix([[0, -v[2], v[1]],
                      [v[2], 0, -v[0]],
                      [-v[1], v[0], 0]])


R = np.matrix([[0.67352, -0.65604, 0.34056],
               [0.26946, 0.64694, 0.71334],
               [0.68831, 0.38868, -0.61251]])
p = np.matrix([1, 2, 3]).transpose()
phi = np.matrix([0.1, 0.2, 0.3]).transpose()

res1 = R*(np.eye(3)+skew_symmetric(phi))*p - R*p
res2 = R*skew_symmetric(phi)*p

res3 = R*skew_symmetric(phi)*p
res4 = -R*skew_symmetric(p)*phi

res5 = skew_symmetric(phi)*p
res6 = -skew_symmetric(p)*phi

print(res5-res6)
