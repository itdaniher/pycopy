# NeoPixel driver for MicroPython on ESP32 with the neopixel_rmt driver
# MIT license;
# 
# Based on original neopixel driver by
# Copyright (c) 2016 Damien P. George
#
# With stolen bits and pieces by nevercast
# https://gist.github.com/nevercast/9c48505cc6c5687af59bcb4a22062795

from neopixel_rmt import write

class NeoPixel:
    def __init__(self, pin, n):
        self.n = n
        self.bpp = 3
        self.pin = pin
        self.buf = bytearray(n * 3)

    def __setitem__(self, index, val):
        if isinstance(index, int):
            offset = index * self.bpp
            for i in range(self.bpp):
                self.buf[offset + i] = val[i]
        elif isinstance(index, slice):
            start = 0 if index.start is None else index.start
            stop = self.n if index.stop is None else index.stop
            step = 1 if index.step is None else index.step
            # Assume that if the first colors element is an int, then its not a sequence
            # Otherwise always assume its a sequence of colors
            if isinstance(val[0], int):
                # pixels[:] = (r,g,b)
                for myindex in range(start, stop, step):
                    (self.buf[myindex * self.bpp],
                     self.buf[myindex * self.bpp + 1],
                     self.buf[myindex * self.bpp + 2]) = val[0], val[1], val[2]
            else:
                # pixels[:] = [(r,g,b), ...]
                # Assume its a sequence, make it a list so we know the length
                if not isinstance(val, list):
                    val = list(val)
                val_length = len(val)
                for myindex in range(start, stop, step):
                    color = val[(myindex - start) % val_length]
                    (self.buf[myindex * self.bpp],
                     self.buf[myindex * self.bpp + 1],
                     self.buf[myindex * self.bpp + 2]) = color[0], color[1], color[2]

    def __getitem__(self, index):
        offset = index * self.bpp
        return tuple(self.buf[offset + i]
                     for i in range(self.bpp))

    def __len__(self):
        return self.n

    def fill(self, color):
        for i in range(self.n):
            self[i] = color

    def write(self):
        write(self.buf)