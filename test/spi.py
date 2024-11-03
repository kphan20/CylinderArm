import time
import spidev
import subprocess

spi = spidev.SpiDev()

spi.open(1, 0)

spi.max_speed_hz = 100000
spi.mode = 0

toggle = False

try:
    result = subprocess.run(['bash', 'reset_spi.sh'], check=True, capture_output=True, text=True)
    print(result.stdout)
except subprocess.CalledProcessError as e:
    print(e.stderr)

try:
    while True:
        if toggle:
            msg = [0x80]
        else:
            msg = [0x00]
        toggle = not toggle

        print(spi.xfer2(msg))
        time.sleep(.1)
finally:
    spi.close()
