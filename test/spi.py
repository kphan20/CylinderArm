import time
import spidev

spi = spidev.SpiDev()

spi.open(1, 0)

spi.max_speed_hz = 100000
spi.mode = 0

toggle = False

try:
    while True:
        if toggle:
            msg = [0x01]
        else:
            msg = [0x00]
        toggle = not toggle

        print(spi.xfer2(msg))
        time.sleep(.5)
finally:
    spi.close()
