import numpy as np
import matplotlib.pyplot as plt
from raspberry.testing.analyze import calibrateAcc
from raspberry.testing.kalman import kalman
from raspberry.testing.madgwick import madgwick_IMU
from raspberry.testing.quaternion import Quaternion

np.set_printoptions(precision=3, linewidth=100, suppress=True)
%load_ext autoreload
%autoreload 2

# %%codecell
dt = 0.002
x0 = np.array([0., 0., 0., 0., 0., 0.])
P0 = np.array([[90., 0., 0., 0., 0., 0.],
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
Q = B@B.T * 20
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
# For only raw acc data
kalmanin = np.array([np.degrees(np.arctan2(data[:, 1],
                                           np.sqrt(np.square(data[:, 0]) +
                                                   np.square(data[:, 2])))),
                     np.degrees(np.arctan2(-data[:, 0],
                                           np.sqrt(np.square(data[:, 2]) +
                                                   np.square(data[:, 1])))),
                     np.zeros((data.shape[0])),
                     data[:, 3],
                     data[:, 4],
                     data[:, 5]]).T

# %%codecell
SEq = Quaternion(1.00, 0.00, 0.00, 0.00)
for n in range(500):
    ad = accData[n]
    gd = gyroData[n]
    norm = np.sqrt(np.sum(np.square(ad)))
    ad /= norm
    f_1 = 2 * SEq.x * SEq.z - 2 * SEq.w * SEq.y - ad[0]
    f_2 = 2 * SEq.w * SEq.x + 2 * SEq.y * SEq.z - ad[1]
    f_3 = 1 - 2 * SEq.x * SEq.x - 2 * SEq.y * SEq.y - ad[2]
    J_11or24 = 2 * SEq.y
    J_12or23 = 2 * SEq.z
    J_13or22 = 2 * SEq.w
    J_14or21 = 2 * SEq.x
    J_32 = 2 * J_14or21
    J_33 = 2 * J_11or24
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2
    norm = np.sqrt(SEqHatDot_1**2 + SEqHatDot_2**2 +
                   SEqHatDot_3**2 + SEqHatDot_4**2)
    SEqHatDot_1 /= norm
    SEqHatDot_2 /= norm
    SEqHatDot_3 /= norm
    SEqHatDot_4 /= norm
    SEqDot_omega_1 = -0.5*SEq.x * gd[0] - 0.5*SEq.y * gd[1] - 0.5*SEq.z * gd[2]
    SEqDot_omega_2 = 0.5*SEq.w * gd[0] + 0.5*SEq.y * gd[2] - 0.5*SEq.z * gd[1]
    SEqDot_omega_3 = 0.5*SEq.w * gd[1] - 0.5*SEq.x * gd[2] + 0.5*SEq.z * gd[0]
    SEqDot_omega_4 = 0.5*SEq.w * gd[2] + 0.5*SEq.x * gd[1] - 0.5*SEq.y * gd[0]
    SEq = Quaternion((SEqDot_omega_1 - (beta * SEqHatDot_1))*dt,
                     (SEqDot_omega_2 - (beta * SEqHatDot_2))*dt,
                     (SEqDot_omega_3 - (beta * SEqHatDot_3))*dt,
                     (SEqDot_omega_4 - (beta * SEqHatDot_4))*dt) * SEq
    SEq = SEq.normalize()
    yaw = np.degrees(np.arctan2(2*SEq.x*SEq.y-2*SEq.w*SEq.z,
                                2*SEq.w**2+2*SEq.x**2-1))
    roll = np.degrees(-np.arcsin(2*SEq.x*SEq.z + 2*SEq.w*SEq.y))
    pitch = np.degrees(np.arctan2(2*SEq.y*SEq.z - 2*SEq.w*SEq.x,
                                  2*SEq.w**2 + 2*SEq.z**2 - 1))
    pred_m[n, :] = [pitch, roll, yaw]

# %%codecell
X = calibrateAcc('./raspberry/testing/200409_02.log')
with open('./raspberry/testing/200412.log', 'r') as f:
    lines = f.readlines()
data = np.array([[float(d)
                  for d in l.replace('\n', '').split(': ')[1].split(', ')]
                 for l in lines[11::] if 'ERROR' not in l])
angles = data[:, 3:6]
accData = data[:, 6:9]
accData = np.pad(accData, ((0, 0), (0, 1)),
                 mode='constant', constant_values=1) @ X
gyroData = data[:, 9:12]
mean = np.median(gyroData, axis=0)
mean
if np.any(np.abs(mean) > 0.01):
    gyroData -= mean
kalmanin = np.concatenate((angles, gyroData), axis=1)
SEq = Quaternion(1.00, 0.00, 0.00, 0.00)
pred_k = np.empty_like(angles)
pred_m = np.empty_like(angles)
fs = np.empty((angles.shape[0]), dtype=np.float)
samples = 50000
x = x0
P = P0
for i in range(samples):
    # Kalman
    out = kalman(x, P, u, kalmanin[i], F, B, Q, H, R, dt)
    x = out[0]
    P = out[1]
    pred_k[i, :] = x[:3]

    # Madgwick
    beta = 0.01 if i > 200 else 1
    accD = accData[i, :]
    SEq_dot = 0.5 * SEq * Quaternion(0, np.deg2rad(gyroData[i, :]))
    f = SEq.conj()*Quaternion(0, 0, 0, 1)*SEq - Quaternion(0, accD)
    fs[i] = f.norm()
    pred_m[i, :] = [*SEq.toEuler()]
    grad = Quaternion(-2*accD[1]*SEq.x + 2*accD[0]*SEq.y +
                      4*SEq.w*(SEq.x**2 + SEq.y**2),
                      -2*accD[1]*SEq.w - 2*accD[0]*SEq.z +
                      4*SEq.x*(accD[2] + SEq.y**2 + SEq.x**2),
                      2*accD[0]*SEq.w - 2*accD[1]*SEq.z +
                      4*SEq.y*(accD[2] + SEq.x**2 + SEq.y**2),
                      -2*accD[0]*SEq.x - 2*accD[1]*SEq.y +
                      4*SEq.z*(SEq.x**2 + SEq.y**2))
    # grad = Quaternion(4*SEq.w*(SEq.x**2 + SEq.y**2) -
    #                   2*SEq.y*accD[0] + 2*SEq.x*accD[1],
    #                   2*(SEq.z*accD[0] + SEq.w*accD[1] +
    #                   2*SEq.x*(SEq.x**2 + SEq.y**2 - accD[2])),
    #                   -2*SEq.w*accD[0] + 2*SEq.z*accD[1] +
    #                   4*SEq.y*(SEq.x**2 + SEq.y**2 - accD[2]),
    #                   2*(2*SEq.z*(SEq.x**2 + SEq.y**2) +
    #                   SEq.x*accD[0] + SEq.y*accD[1]))
    # SEq_dot_est = SEq_dot - beta*grad.normalize()
    # SEq = ((SEq_dot_est*dt).exp()*SEq).normalize()
    SEq = (SEq + (SEq_dot - beta * grad.normalize()) * dt).normalize()
init = 500
axis = 0
plt.plot(angles[init:samples, axis], 'y,',
         pred_k[init:samples, axis], 'g,',
         pred_m[init:samples, axis], 'r,')
plt.show()
plt.plot(pred_m[40:samples, :3])
plt.show()
plt.plot(data[40:samples, 3:5])
plt.show()
plt.plot(accData[:samples])
plt.show()
plt.plot(data[:samples, 1])
