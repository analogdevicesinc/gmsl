from gmsl_lib import script_parser
from smbus2 import SMBus, i2c_msg
from time import sleep

class GMSLConfig:
    def __init__(self, i2c_bus, scripts):
        self.p = script_parser.Parser()
        self.bus = i2c_bus
        self.script_list = scripts

    def load(self, verify=False):
        for file in self.script_list:
            cmd_list = self.p.parse(file)
            for cmd in cmd_list:
                chip_addr = cmd[1] // 2 # GMSL chip address
                try:
                    if cmd[0] == 'W1':
                        self.bus.write_byte_data(chip_addr, cmd[2], cmd[3])
                    elif cmd[0] == 'W2':
                        self.bus.write_i2c_block_data(chip_addr, cmd[2] >> 8, [cmd[2], cmd[3]])
                    elif cmd[0] == 'RMW':
                        msg = i2c_msg.write(chip_addr, cmd[2].to_bytes(2, 'big'))
                        self.bus.i2c_rdwr(msg)
                        to_write = self.bus.read_byte(chip_addr)
                        to_write &= ~cmd[4]
                        to_write |= cmd[3]
                        self.bus.write_i2c_block_data(chip_addr, cmd[2] >> 8, [cmd[2], to_write])
                    elif cmd[0] == 'delay':
                        sleep(cmd[1] / 1000)
                    else:
                        print("Unknown command!")
                        exit()
                except Exception as e:
                    print("Failed I2C transfer! \nCMD:", cmd[0], "CHIP_ADDR:", hex(cmd[1]), "\nREG:", hex(cmd[2]), "\nDATA:", hex(cmd[3]))
                    exit()
                if verify and cmd[0] != 'delay':
                    to_write = cmd[3]
                    if cmd[0] == 'W1':
                        msg = i2c_msg.write(chip_addr, cmd[2].to_bytes(1, 'big'))
                    elif cmd[0] == 'W2':
                        msg = i2c_msg.write(chip_addr, cmd[2].to_bytes(2, 'big'))
                    self.bus.i2c_rdwr(msg)
                    read = self.bus.read_byte(chip_addr)
                    if cmd[0] == 'RMW':
                        read &= cmd[4]
                    if  to_write != read:
                        print("Wrong!!! Device:", hex(cmd[1]), "Register:", hex(cmd[2]), "Should be:", hex(to_write), "   but was read:", hex(read))

