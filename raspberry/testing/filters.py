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

# %%codecell
with open('./raspberry/testing/200409_02.log', 'r') as file:
    lines = file.readlines()
data = np.array([[float(d)
                  for d in l.replace('\n', '').split(': ')[1].split(', ')]
                 for l in lines[11::]])
accData = data[:, 6:9]


def strided_app(a, L, S):  # Window len = L, Stride len/stepsize = S
    nrows = ((a.size-L)//S)+1
    n = a.strides[0]
    return np.lib.stride_tricks.as_strided(a,
                                           shape=(nrows, L), strides=(S*n, n))


# %%codecell
mask = np.all(np.abs(np.diff(accData, axis=0, append=0)) > 0.01, axis=1)
pad = 1000
p = np.round(pad*np.mean(strided_app(np.pad(mask,
                                            (pad//2, (pad-1)//2), 'constant'),
                                     pad, 1), axis=1)).astype(np.bool)
print('Sum: ', mask.sum(),
      p.sum(), 'Median: ',
      np.median(np.abs(accData[~mask])),
      'Mean: ', np.mean(np.abs(accData[~mask])))
accData = accData[~p]
plt.plot(accData)

# %%codecell
mask = np.all(np.abs(np.diff(accData, axis=0, append=0)) > 0.01, axis=1)
pad = 1000
p = np.round(pad *
             np.mean(strided_app(np.pad(mask,
                                        (pad//2, (pad-1)//2),
                                        'constant'),
                                 pad, 1), axis=1)).astype(np.bool)
accData = accData[~p]
cuts = np.where(np.any(np.abs(np.diff(accData, axis=0)) > 0.1, axis=1))[0]
cuts = cuts[np.abs(np.diff(cuts, append=0)) > 1000]
splits = np.split(accData, cuts)
min_d = np.min([d.shape[0] for d in splits])
splits = np.stack([arr[-min_d:] for arr in splits])
# ind = np.array([25000, 29000, 51500, 58000, 77000,
#                 85000, 105000, 111500, 127500, 133000])
# splits = np.split(accData, ind)[::2]
# accData = np.concatenate(splits)
# plt.plot(accData)
plt.plot(np.concatenate(splits))
ix = np.argmax(np.min(np.abs(splits), axis=1), axis=1)
sign = np.atleast_2d(np.sign(splits[np.arange(6), min_d//2, ix])).T
y = np.zeros_like(splits)
y[np.arange(6), :, ix] = sign
data = np.concatenate(splits)
y = np.concatenate(y)
data = np.pad(data, ((0, 0), (0, 1)), 'constant', constant_values=1)
X = np.linalg.inv(data.T.dot(data)).dot(data.T).dot(y)
X
