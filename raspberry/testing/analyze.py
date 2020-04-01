# %%codecell
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

# %%codecell
# Constants from raspberry program
INTERVAL_MSEC = 2
INTERVAL_SEC = INTERVAL_MSEC/1000
FREQ = 1/INTERVAL_SEC


# %%codecell
def DFT(x, polar=False):
    N = x.size
    cc = 0
    cs = 0
    X = np.empty((int(N/2+1), 2))
    for i in range(int(N/2+1)):
        cc = np.sum([x[k] * np.cos(2*np.pi*k*i/N) for k in range(N)])
        cs = np.sum([x[k] * np.sin(2*np.pi*k*i/N) for k in range(N)])
        X[i, 0] = cc
#       np.sqrt(cc*cc + cs*cs)
        X[i, 1] = cs
#       np.arctan2(cs, cc)
    return X if not polar else rect_to_polar(X)


def FFT(x, polar=False):
    def transform(x):
        x = np.array(x, dtype=np.complex)
        N = x.size
        if not (np.log2(N) % 1 == 0):
            raise IndexError('The length of the input must be a power of 2')
        if N == 1:
            return x
        else:
            g = transform(x[::2])
            h = transform(x[1::2])
            c = np.empty((N), dtype=np.complex)
            for k in range(0, int(N/2)):
                c[k] = g[k] + h[k] * np.exp(-2*np.pi*1j*k/N)
                c[int(k+N/2)] = g[k] - h[k] * np.exp(-2*np.pi*1j*k/N)
        return c
    c = transform(x)
    fft = np.array([c.real, c.imag])
    if polar:
        fft = rect_to_polar(fft)
    return fft


def rect_to_polar(X):
    Y = np.empty_like(X)
    Y[:, 0] = np.sqrt(np.square(X[:, 0]) + np.square(X[:, 1]))
    Y[:, 1] = np.arctan2(X[:, 1], X[:, 0])
    return Y


def polar_to_rect(X):
    Y = np.empty_like(X)
    Y[:, 0] = X[:, 0] * np.cos(X[:, 1])
    Y[:, 1] = X[:, 0] * np.sin(X[:, 1])
    return Y


def IDFT(X, polar=False):
    N = (X.shape[0]-1)*2
    x = np.empty(N)
    if polar:
        for n in range(N):
            x[n] = np.sum([2./N * X[k, 0] * np.cos(2*np.pi*n*k/N-X[k, 1])
                          for k in range(X.shape[0])])
    else:
        X = np.copy(X)
        X *= 2./N
        X[0, 0] = X[0, 0]/2
        X[-1, 0] = X[-1, 0]/2
        for n in range(N):
            x[n] = np.sum([X[k, 0] * np.cos(2*np.pi*n*k/N)
                          + X[k, 1] * np.sin(2*np.pi*n*k/N)
                           for k in range(X.shape[0])])
    return x


# %%codecell
def analyze(func, data, plot=False):
    out = func(data)
    fftd = DFT(data, True)[:, 0]
    ffto = DFT(out, True)[:, 0]
    N = data.size
    if plot:
        plt.rcParams.update({'figure.autolayout': True})
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(nrows=2,
                                                     ncols=2,
                                                     figsize=(8, 8))
        ax1.plot(np.linspace(0, N*INTERVAL_SEC, N), data, 'k')
        ax1.set_xlabel(r'Time $t$')
        ax1.set_ylabel('Signal value')
        ax1.set_title('Data')

        ax2.plot(np.linspace(0, np.pi, fftd.size), fftd, 'k')
        ax2.set_xlabel(r'Frequency $\omega$(radians)')
        ax2.set_ylabel('Amplitude')
        ax2.set_title('Discrete Fourier Transform of data')

        def rad(x, pos):
            frac = x/np.pi
            if x == 0:
                return '0'
            if frac % 1 == 0:
                coeff = str(int(frac)) if frac != 1 else ''
            elif 1 % frac == 0:
                coeff = r'$\frac{1}{' + str(int(1/frac)) + '}$'
            else:
                coeff = (r'$\frac{' +
                         str(int(frac/(1 % frac))) +
                         r'}{' +
                         str(int(1 / (1 % frac))) +
                         r'}$')
            return r'{}{}'.format(coeff, r'$\pi$')

        ax2.xaxis.set_major_formatter(FuncFormatter(rad))
        ax2.set_xticks(np.linspace(0, np.pi, 5))

        ax3.plot(np.linspace(0, N*INTERVAL_SEC, N), out, 'k')
        ax3.set_xlabel(r'Time $t$')
        ax3.set_ylabel('Signal value')
        ax3.set_title('Filtered Data')

        ax4.plot(np.linspace(0, np.pi, ffto.size), ffto, 'k')
        ax4.set_xlabel(r'Frequency $\omega$(radians)')
        ax4.set_ylabel('Amplitude')
        ax4.set_title('Discrete Fourier Transform')
        ax4.xaxis.set_major_formatter(FuncFormatter(rad))
        ax4.set_xticks(np.linspace(0, np.pi, 5))

        plt.tight_layout()
        plt.show()
    return (out, fftd, ffto)


# %%codecell
def f(x, window=21):
    N = x.size
    p = (window-1)//2
    padx = np.pad(x, (p, p), 'constant', constant_values=0)
    y = np.zeros(N)
    y[0] = np.sum([padx[i+p] for i in range(p)])/window
    for n in range(1, N):
        y[n] = y[n-1] + (padx[n+2*p] - padx[n])/window
    return y


def g(x):
    def sinc(N=64, freq=2):
        p = N//2
        i = np.arange(-p, p)
        sinc = np.sin(freq*i)/(i+1e-20)
        sinc[p] = 1.
        return sinc
    return np.convolve(x, sinc(), 'same')


# %%codecell
with open('./raspberry/testing/200326.log', 'r') as file:
    lines = file.readlines()
data = np.array([[float(d)
                  for d in l.replace('\n', '').split(': ')[1].split(', ')]
                for l in lines[10::]]).T
# fft_data = [DFT(data[3], True), DFT(data[4], True), DFT(data[5], True)]
# fft_data[0]
fft_data = DFT(data[3, 0:500], True)
fft_data.shape
fft_data[0]
out, fftd, ffto = analyze(g, data[3, 0:500], True)

# %%codecell
data = np.sin(np.linspace(0, 2*np.pi, 512))  # np.repeat([1], 512)
out, fftd, ffto = analyze(g, data, True)

np.max(fftd)/np.sum(fftd)
ffto
a = FFT(data, False).T
b = DFT(data, False)
b.shape
a.shape
plt.plot(a[:, 0])
plt.plot(b[:, 0])

# %%codecellwith open('./raspberry/testing/200326.log', 'r') as file:
with open('./raspberry/testing/200401.log', 'r') as file:
    lines = file.readlines()
data = np.array([[float(d)
                  for d in l.replace('\n', '').split(': ')[1].split(', ')]
                for l in lines[10::]]).T
data = data[:3, :]
plt.plot(data[0], 'b', data[1], 'g', data[2], 'y',
         np.sqrt(np.sum(np.square(data), axis=0)), 'r')
