#!/usr/bin/env python

import sys
import logging
import serial
import argparse
import time
import pandas as ps
from datetime import datetime
from config import PS3005D_PORT

# FORMAT = '%(levelname)-5s %(message)s'

# logging.basicConfig(format=FORMAT)
# logger = logging.getLogger(__name__)
# logger.level = logging.DEBUG

device = None
data = []

PS3005D_DEFAULT_BAUDRATE = 9600

class PS3005D():
    def __init__(self, port, baudrate=PS3005D_DEFAULT_BAUDRATE):
        self.port = port
        self.ser = serial.Serial(port, baudrate, timeout=1)

    def send(self, msg):
        # msg += '\r\n'
        # logger.debug('Sending: {0}'.format(msg))
        self.ser.write(bytes(msg, 'ascii'))

    def receive(self, timeout=2000):
        end_time = time.time()+(timeout/1000)
        payload=""
        while(end_time-time.time() > 0):
            c = self.ser.readline()
            payload = c.decode("utf-8")
            if c:
                break
        return payload


    def get_id(self):
        self.send('*IDN?')
        payload=self.receive()
        return payload

    def turn_on(self):
        self.send('OUT1')
        logging.info('Power ON')

    def turn_off(self):
        self.send('OUT0')    
        logging.info('Power OFF')

    def set_voltage(self, voltage):
        tx_str = "VSET1:%0.2f" % (voltage)
        self.send(tx_str)
        # self.send('VSET1:{0}'.format(voltage))
        logging.info("Voltage set to %0.2fV" % (voltage))

    def set_current(self, current):
        tx_str = "ISET1:%0.2f" % (current)
        self.send(tx_str)
        # self.send('ISET1:{0}'.format(current))
        logging.info("Current set to %0.3fA" % (current))


    def enable_ovp(self):
        self.send('OVP1')
        logging.info('Enabled OVP')

    def disable_ovp(self):
        self.send('OVP0')
        logging.info('Disabled OVP')


    def enable_ocp(self):
        self.send('OCP1')
        logging.info('Enabled OCP')


    def disable_ocp(self):
        self.send('OCP0')
        logging.info('Disabled OCP')


    def get_load_voltage(self):
        self.send('VOUT1?')
        payload=self.receive()
        return float(payload)

    def get_load_current(self):
        self.send('IOUT1?')
        payload=self.receive()
        try:
            return float(payload)
        except:
            return -1.0


def log(ps3005d, voltage,current,frequency):
    ps3005d.set_voltage(voltage)
    time.sleep(0.2)
    ps3005d.set_current(current)
    time.sleep(0.2)
    ps3005d.enable_ovp()
    time.sleep(0.2)
    ps3005d.enable_ocp()
    time.sleep(0.2)
    ps3005d.turn_on()
    time.sleep(0.2)
    timestamps = []
    voltages = []
    currents = []
    try:
        while True:
            v = ps3005d.get_load_voltage()
            i = ps3005d.get_load_current()
            timestamp = datetime.now()
            timestamps.append(timestamp)
            voltages.append(v)
            currents.append(i)
            time.sleep(frequency/1000)
    except KeyboardInterrupt:
        ps3005d.turn_off()

    df = ps.DataFrame(data={'voltage':voltages,'current':currents},index=timestamps)
    return df

def main():
    logger.info('Starting PS3005D interface')
    parser = argparse.ArgumentParser(description='PS3005D')
    parser.add_argument('port',type=str, default=PS3005D_PORT)
    parser.add_argument('cmd',type=str)
    parser.add_argument('args',nargs='*',type=float,default=[])
    parser.add_argument('--baud',dest='baud',type=int,default=PS3005D_DEFAULT_BAUDRATE)
    parser.add_argument('--log',dest='log',type=str,default='log.csv')
    args = parser.parse_args()

    try:
        ps3005D = PS3005D(args.port, int(args.baud))
    except serial.SerialException:
        logger.error('Could not connect to device on {0}'.format(args.port))
        return 1

    if args.cmd == 'id':
        device_id=ps3005D.get_id()
        print('Device ID: {0}'.format(device_id))
    
    elif args.cmd == 'on':
        ps3005D.turn_on()
    
    elif args.cmd == 'off':
        ps3005D.turn_off()
    
    elif args.cmd == 'enable_ovp':
        ps3005D.enable_ovp()
    
    elif args.cmd == 'enable_ocp':
        ps3005D.enable_ocp()
    
    elif args.cmd == 'enable_ovp':
        ps3005D.disable_ovp()
    
    elif args.cmd == 'disable_ocp':
        ps3005D.disable_ocp()
    
    elif args.cmd == 'load_voltage':
        voltage = ps3005D.get_load_voltage()
        print('Load Voltage: {0}'.format(voltage))
    
    elif args.cmd == 'load_current':
        current = ps3005D.get_load_current()
        print('Load Voltage: {0}'.format(current))
    
    elif args.cmd == 'voltage':
        if not len(args.args) == 1:
            print('`voltage` command requires voltage argument')
            return 2
        voltage = args.args[0]
        ps3005D.set_voltage(voltage)
    
    elif args.cmd == 'current':
        if not len(args.args) == 1:
            print('`current` command requires current argument')
            return 2
        current = args.args[0]
        ps3005D.set_current(current)

    elif args.cmd == 'log':
        log_voltage = args.args[0]
        log_current = args.args[1]

        if len(args.args) == 2:
            log_frequency = 1000

        logger.info('Logging {0}V, {1}A every {2}ms'.format(log_voltage,log_current,log_frequency))

        data = log(ps3005D, log_voltage,log_current,log_frequency)
            
        data.to_csv(args.log,index_label='timestamp')
        logger.info('Saved {0} records to {1}'.format(len(data),args.log))


if __name__ == '__main__':
    sys.exit(main())