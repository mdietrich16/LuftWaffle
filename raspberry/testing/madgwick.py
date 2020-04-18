from raspberry.testing.quaternion import Quaternion
print('Loaded Madgwick!')

deg2rad = 0.017453292519943295


def madgwick_IMU(SEq, accData, gyroData, beta, dt):
    w = Quaternion(0, gyroData)
    SEq_dot = 0.5 * SEq * (w * deg2rad)
    # For g = (0, 0, 1)
    # grad = Quaternion(-2*accData[1]*SEq.x + 2*accData[0]*SEq.y +
    #                   4*SEq.w*(SEq.x**2 + SEq.y**2),
    #                   -2*accData[1]*SEq.w - 2*accData[0]*SEq.z +
    #                   4*SEq.x*(accData[2] + 2*SEq.y**2 + SEq.x**2),
    #                   2*accData[0]*SEq.w - 2*accData[1]*SEq.z +
    #                   4*SEq.y*(accData[2] + 2*SEq.x**2 + SEq.y**2),
    #                   -2*accData[0]*SEq.x - 2*accData[1]*SEq.y +
    #                   4*SEq.z*(SEq.x**2 + SEq.y**2))
    # For g = (0, 0, -1)
    grad = Quaternion(4*SEq.w*(SEq.x**2 + SEq.y**2) -
                      2*SEq.y*accData[0] + 2*SEq.x*accData[1],
                      2*(SEq.z*accData[0] + SEq.w*accData[1] +
                      2*SEq.x*(SEq.x**2 + SEq.y**2 - accData[2])),
                      -2*SEq.w*accData[0] + 2*SEq.z*accData[1] +
                      4*SEq.y*(SEq.x**2 + SEq.y**2 - accData[2]),
                      2*(2*SEq.z*(SEq.x**2 + SEq.y**2) +
                      SEq.x*accData[0] + SEq.y*accData[1]))
    # mu = alpha * SEq_dot.norm() * dt
    # SEq_a = SEq - mu*grad.normalize()
    # gamma = beta/(mu/dt + beta)
    SEq = (SEq + (SEq_dot - beta * grad.normalize()) * dt).normalize()
    return SEq


def madgwick_MARG(SEq, accData, gyroData, magData, dt):
    pass
