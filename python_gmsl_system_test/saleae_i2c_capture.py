#!/usr/bin/env python

from saleae import automation

import numpy as np
import os
import os.path
from threading import Thread, Event
from queue import Queue
import time

import logging

class GmslCaptureManager(Thread):
    def __init__(self, capture_time=1):
        super(GmslCaptureManager, self).__init__()
        self._trigger = Queue(1)
        self._done = Event()
        self._exit = Event()
        self.start()
        
    def run(self):
        
        i2c_capture = GmslI2cCapture()
        
        while(1):
            if not self._done.isSet():
                #Wait for trigger event with 0.1s timeout
                try:
                    filepath = self._trigger.get(timeout=0.1)
                    i2c_capture.run_capture(filepath)
                    self._done.set()
                except:
                    pass
            else:
                time.sleep(0.1)
                                    
            if self._exit.is_set():
                return
        
    def trigger_capture(self, filepath):
        if self._done.is_set():
            logging.warning("Capture done pending, can't run another capture")
            return False
        try:
            self._trigger.put_nowait(filepath)
        except:
            logging.warning("Trigger queue is full")
            return False
        return True
    
    def capture_done(self, clear_flag=True):
        done = self._done.is_set()
        if done and clear_flag:
            self._done.clear()
        return done
        
    def request_exit(self):
        self._exit.set()
        self.join()

class GmslI2cCapture:
    def __init__(self, capture_time=0.2, trigger_type=automation.DigitalTriggerType.FALLING):
        self.capture_time=capture_time
        self.trigger_type=trigger_type
        self.debug_enable = False

    def debug(self, *args, **kwargs):
        if self.debug_enable:
            print(*args, **kwargs)
        

    def run_capture(self, filepath):
        try:
            with automation.Manager.connect(port=10430) as manager:

                # Configure the capturing device to record on digital channels 0, 1, 2, and 3,
                # with a sampling rate of 10 MSa/s, and a logic level of 3.3V.
                # The settings chosen here will depend on your device's capabilities and what
                # you can configure in the Logic 2 UI.
                device_configuration = automation.LogicDeviceConfiguration(
                    enabled_digital_channels=[0, 1, 2, 3],
                    enabled_analog_channels=[],
                    digital_sample_rate=12_000_000
                )

                # Record 5 seconds of data before stopping the capture
                capture_configuration = automation.CaptureConfiguration(
                    # capture_mode=automation.TimedCaptureMode(duration_seconds=5.0)
                    capture_mode=automation.DigitalTriggerCaptureMode(
                        trigger_type=self.trigger_type,
                        trigger_channel_index=0,
                        after_trigger_seconds=self.capture_time
                        )
                )
                
                # Start a capture - the capture will be automatically closed when leaving the `with` block
                # Note: The serial number 'F4241' is for the Logic Pro 16 demo device.
                #       To use a real device, you can:
                #         1. Omit the `device_id` argument. Logic 2 will choose the first real (non-simulated) device.
                #         2. Use the serial number for your device. See the "Finding the Serial Number
                #            of a Device" section for information on finding your device's serial number.
                with manager.start_capture(
            #            device_id='',
                        device_configuration=device_configuration,
                        capture_configuration=capture_configuration) as capture:

                    # Wait until the capture has finished
                    # This will take about 5 seconds because we are using a timed capture mode
                    capture.wait()                 

                    self._analyze_capture(capture, filepath)
                    
                    # Finally, save the capture to a file
                    if not filepath.endswith('.sal'):
                        filepath += '.sal'
                    capture.save_capture(filepath=filepath)
                    
        except:
            self.debug("Error: Logic2 probably not running")
            
            
    def _analyze_capture(self, capture, filepath):
        local = capture.add_analyzer(name='I2C', label="I2C_local", settings={"SCL": 0, "SDA": 1})
        remote = capture.add_analyzer(name='I2C', label="I2C_remote", settings={"SCL": 2, "SDA": 3})
        
        #export capture to csv file
        csv_filepath = filepath.replace(".sal", ".csv")
        capture.export_data_table(csv_filepath, [local, remote])        
        
            
    def run_analyzers(self, filepath):
        try:
            with automation.Manager.connect(port=10430) as manager:
                if not filepath.endswith('.sal'):
                    filepath += '.sal'
                with manager.load_capture_file(filepath) as capture:
                    self._analyze_capture(capture, filepath)
                    
        except:
            self.debug("Error: Logic2 probably not running")


if __name__ == "__main__":
    print("DO NOT RUN THIS AS MAIN")