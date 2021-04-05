from smbus2 import SMBus
from mlx90614 import MLX90614
import time

bus = SMBus(1)

while True:
    sensor = MLX90614(bus, address=0x5A)
    print("Amb Temp: ", sensor.get_ambient())
    print("Obj Temp: ", sensor.get_object_1())
    time.sleep(0.1)
bus.close()