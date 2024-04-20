import math
import matplotlib.pyplot as plt

bruh = []
for i in range(90):
    bruh.append(int(math.cos(i * math.pi / 180) * 2**15))
    print(f"{int(math.sin(i * math.pi / 180) * 2**15)},", end="")
    if i % 20 == 0 and i > 0:
        print()

plt.plot(list(range(90)), bruh)
# plt.plot(list(range(90)), [math.cos(i * math.pi / 180) * 2**15 for i in range(90)])
plt.show()
