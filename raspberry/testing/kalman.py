import numpy as np
import matplotlib.pyplot as plt


def kalman(x, P, u, z, F, B, Q, H, R, dt):
    # Predict
    x = F.dot(x) + B.dot(u)
    P = F.dot(P).dot(F.T) + Q
    diag = np.einsum('ii->i', P)
    save = diag.copy()
    P[...] = 0
    diag[...] = save
    # Update
    y = z - H.dot(x)
    S = H.dot(P).dot(H.T) + R
    K = P.dot(H.T).dot(np.linalg.inv(S))
    x = x + K.dot(y)
    P = (np.identity(x.size) - K.dot(H)).dot(P)
    return (x, P, K)


# %%codecell
dt = 1
x = np.array([[0, 10]]).T
P = np.array([[0., 0.],
              [0., 1.]])
u = np.array([[1.]])
F = np.array([[1., dt],
              [0., 1.]])
B = np.array([[(dt**2)/2],
              [dt]])
Q = B@B.T * 2
H = np.array([[1., 0.],
              [0., 1.]])
R = np.array([[10., 0.],
              [0., 3.]])

# %%codecell
n = 20
preds = np.empty((n, 2, 1))
variances = np.empty((n, 2, 2))
ks = np.empty((n, 2, 2))
data = (np.random.normal(scale=np.sqrt([10, 3]), size=(n, 2, 1))
        + np.pad((10*dt*np.arange(1, n+1)
                  + 0.5*np.square(dt*np.arange(1, n+1))).reshape(n, 1, 1),
                 ((0, 0), (0, 1), (0, 0)), mode='constant', constant_values=0)
        + np.pad(10 + dt*np.arange(1, n+1).reshape(n, 1, 1),
                 ((0, 0), (1, 0), (0, 0)), mode='constant', constant_values=0))
for i in range(n):
    x = F.dot(x) + B.dot(u)
    P = (F.dot(P).dot(F.T) + Q)
    diag = np.einsum('ii->i', P)
    save = diag.copy()
    P[...] = 0
    diag[...] = save
    y = data[i] - H.dot(x)
    S = H.dot(P).dot(H.T) + R
    K = P.dot(H.T).dot(np.linalg.inv(S))
    x = x + K.dot(y)
    P = (np.identity(x.size) - K.dot(H)).dot(P)
    preds[i, :] = x
    variances[i, :] = P
    ks[i, :] = K
plt.plot(data[:, 0], 'y', preds[:, 0], 'g')
plt.show()
plt.plot(variances[:, 0, 0])
plt.show()
plt.plot(ks[:, 0, 0])

# %%codecell
n = 20
preds = np.empty((n, 2, 1))
variances = np.empty((n, 2, 2))
ks = np.empty((n, 2, 2))
data = (np.random.normal(scale=np.sqrt([10, 3]), size=(n, 2, 1))
        + np.pad((10*dt*np.arange(1, n+1)
                  + 0.5*np.square(dt*np.arange(1, n+1))).reshape(n, 1, 1),
                 ((0, 0), (0, 1), (0, 0)), mode='constant', constant_values=0)
        + np.pad(10 + dt*np.arange(1, n+1).reshape(n, 1, 1),
                 ((0, 0), (1, 0), (0, 0)), mode='constant', constant_values=0))
for i in range(n):
    out = kalman(x, P, u, data[i], F, B, Q, H, R, dt)
    x = out[0]
    P = out[1]
    preds[i, :] = out[0]
    variances[i, :] = out[1]
    ks[i, :] = out[2]
preds
plt.plot(data[:, 0], 'y', preds[:, 0], 'g')
plt.show()
plt.plot(variances[:, 0, 0])
plt.show()
plt.plot(ks[:, 0, 0])
