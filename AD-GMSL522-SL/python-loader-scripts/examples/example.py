import sys
sys.path.append('..')

from gmsl_lib import config_loader
from smbus2 import SMBus

I2C_BUS_NR = 1
IN_FILES = [
    'gmsl_scripts_examples/MAX96724-MAX96717-V-INT-rgb888-portA.cpp',
    'gmsl_scripts_examples/MAX96724_TPG.csv',
]

bus = SMBus(I2C_BUS_NR)

c = config_loader.GMSLConfig(bus, IN_FILES)
c.load()

bus.close()
print('Done configuring GMSL')
