import numpy as np
import matplotlib.pyplot as plt
import scipy
class IIRFilter:
    def __init__(self, b, a):
        self.a = a
        self.b = b
        assert(len(a) == len(b))
        self.size = len(a)
        self.input = [0] * self.size
        self.output = [0] * self.size

    def run(self, x):
        output = 0

        for i in range(self.size - 1)[::-1]:
            self.input[i+1] = self.input[i]
        self.input[0] = x


        for i in range(self.size):
            output += self.input[i] * self.b[i]


        for i in range(self.size - 1):
            output -= self.output[i] * self.a[i + 1]

        for i in range(self.size - 1)[::-1]:
            self.output[i+1] = self.output[i]

        output /= self.a[0]

        self.output[0] = output

        return output

fltr = IIRFilter([0.00255054, 0.00510107, 0.00255054], [ 1.  ,       -1.85214649,  0.86234863])

samplerate = 6000
freq = 200
ts = np.arange(0, 500e-3, 1 / samplerate)
inp = np.sin(2 * np.pi * freq * ts)

out = []
for x in inp:
    out.append(fltr.run(x))

out = scipy.signal.lfilter(fltr.b, fltr.a, inp)

plt.plot(ts, inp, label='input')
plt.plot(ts, out, label='output')
plt.show()
