import numpy as np

from scipy import signal

import matplotlib.pyplot as plt

samplerate = 6000
nyquist = samplerate / 2
cutoff = 100

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)

b, a = signal.iirfilter(2, cutoff, btype='lowpass', ftype='butter', fs=samplerate)
print(b, a)

w, h = signal.freqz(b, a, fs=samplerate)

ax.semilogx(w, 20 * np.log10(np.maximum(abs(h), 1e-5)))
ax.set_xlabel('Frequency [Hz]')
ax.set_ylabel('Amplitude [dB]')
ax.axis((10, 10000, -100, 10))
ax.grid(which='both', axis='both')
plt.show()