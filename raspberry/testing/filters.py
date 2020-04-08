import numpy as np
import matplotlib.pyplot as plt
from raspberry.testing.kalman import kalman
np.set_printoptions(precision=3, linewidth=100, suppress=True)
# from raspberry.testing.madgwick import madgwick

# %%codecell
with open('./raspberry/testing/200326.log', 'r') as file:
    # with open('./raspberry/testing/200401.log', 'r') as file:
    lines = file.readlines()
data = np.array([[float(d)
                  for d in l.replace('\n', '').split(': ')[1].split(', ')]
                 for l in lines[10::]])[:, 0:6]
# %%codecell
dt = 0.02
x = np.array([90., 0., 0., 0., 0., 0.])
P = np.array([[90., 0., 0., 0., 0., 0.],
              [0., 90., 0., 0., 0., 0.],
              [0., 0., 90., 0., 0., 0.],
              [0., 0., 0., 0.5, 0., 0.],
              [0., 0., 0., 0., 0.5, 0.],
              [0., 0., 0., 0., 0., 0.5]])
u = np.zeros(3)
F = np.array([[1., 0., 0., dt, 0., 0.],
              [0., 1., 0., 0., dt, 0.],
              [0., 0., 1., 0., 0., dt],
              [0., 0., 0., 1., 0., 0.],
              [0., 0., 0., 0., 1., 0.],
              [0., 0., 0., 0., 0., 1.]])
B = np.array([[(dt**2)/2, 0.,        0.],
              [0.,        (dt**2)/2, 0.],
              [0.,        0.,        (dt**2)/2],
              [dt,        0.,        0.],
              [0.,        dt,        0.],
              [0.,        0.,        dt]])
Q = B@B.T
# np.array([[.1,  0.,  0.,  0.,  0.,   0.],
#               [0.,  .1,  0.,  0.,  0.,   0.],
#               [0.,  0.,  .1,  0.,  0.,   0.],
#               [0.,  0.,  0.,  .01, 0.,   0.],
#               [0.,  0.,  0.,  0.,  .01,  0.],
#               [0.,  0.,  0.,  0.,  0.,   .01]])
H = np.array([[1., 0., 0., 0., 0., 0.],
              [0., 1., 0., 0., 0., 0.],
              [0., 0., 1., 0., 0., 0.],
              [0., 0., 0., 1., 0., 0.],
              [0., 0., 0., 0., 1., 0.],
              [0., 0., 0., 0., 0., 1.]])
# R = np.array([[1., 0., 0., 0., 0., 0.],
#               [0., 1., 0., 0., 0., 0.],
#               [0., 0., 1., 0., 0., 0.],
#               [0., 0., 0., .1, 0., 0.],
#               [0., 0., 0., 0., .1, 0.],
#               [0., 0., 0., 0., 0., .1]])*10
R = np.array([[0.10297752, -0.00020321, 0.,
               -0.00084141, -0.00103052, -0.00154832],
              [-0.00020321, 0.05051184, 0.,
               -0.00072003, -0.00022587, -0.00033468],
              [0.,         0.,         0.,
               0.,          0.,          0.],
              [-0.00084141, -0.00072003,  0.,
               0.09887649, -0.00894457, -0.00459549],
              [-0.00103052, -0.00022587,  0.,
               -0.00894457,  0.02453268, -0.00576382],
              [-0.00154832, -0.00033468,  0.,
               -0.00459549, -0.00576382,  0.05649289]])

# %%codecell
mean = np.sum(data[:, 3:6], axis=0)/data.shape[0]
print(mean)
if np.any(np.abs(mean) > 0.01):
    data[:, 3:6] -= mean

# %%codecell
preds = np.empty((30000, 6))
variances = np.empty((30000, 6, 6))
ks = np.empty((30000, 6, 6))
for i in range(30000):
    out = kalman(x, P, u, data[i], F, B, Q, H, R, dt)
    x = out[0]
    P = out[1]
    preds[i, :] = out[0]
    variances[i, :] = out[1]
    ks[i, :] = out[2]
plt.plot(data[:, 0], 'y,', preds[:, 0], 'g,')
