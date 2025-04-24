#!/usr/bin/python3
# -*- coding: ascii -*-

import smbus2 as smbus
import time
from enum import IntEnum
if __name__!="__main__":
    from .imubase import *
else:
    from imubase import *

class AK09918(IMUBase):
    ADDRESS = [0x0c,0x0c]
    ID_REG_VALUE=[(0x00,0x48),(0x01,0x0c)]

    @staticmethod
    def has_accelerometer():
        return False

    @staticmethod
    def has_gyro():
        return False

    @staticmethod
    def has_magnetometer():
        return True

    def __init__(self):
        self.is_initialised = False
        self.regs = AK09918.Regs


    class Regs(IntEnum):
        # AK09918 register definitions
        WIA1 = 0x00    # Company ID
        WIA2 = 0x01    # Device ID
        RSV1 = 0x02    # Reserved 1
        RSV2 = 0x03    # Reserved 2
        ST1 = 0x10     # DataStatus 1
        HXL = 0x11     # X-axis data 
        HXH = 0x12
        HYL = 0x13     # Y-axis data
        HYH = 0x14
        HZL = 0x15     # Z-axis data
        HZH = 0x16
        TMPS = 0x17    # Dummy
        ST2 = 0x18     # Datastatus 2
        CNTL1 = 0x30   # Dummy
        CNTL2 = 0x31   # Control settings
        CNTL3 = 0x32   # Control settings

        SRST_BIT = 0x01  # Soft Reset
        HOFL_BIT = 0x08  # Sensor Over Flow
        DOR_BIT = 0x02   # Data Over Run
        DRDY_BIT = 0x01  # Data Ready

        MAG_DATA = 0x11 # start of data

    # AK8963 magnetometer included on chip
    # NB magnetometer data is little endian so 
    # needs swapping on read (this is done by
    # the AK09918 chip, which means at the end
    # all readings are in the same format)

    def _startup(self):
        self.start_i2c(big_endian=False)
        if self.read(AK09918.Regs.WIA2) != 0x0c:
            print("WOO")
            raise IOError("AK09918 returned incorrect device ID:",self.read(AK09918.Regs.WHOAMI))

        # turn on
        self.write(AK09918.Regs.CNTL2, 0x01)
        time.sleep(0.01)


        # read at 100hz
        self.write(AK09918.Regs.CNTL2, 0x08)

        self.is_initialised=True

    
    def _wait_for_data_ready(self):
        while True:
            # for some reason reading both ST1 and ST2 is
            # required to trigger update of status regs
            st1=self.read(AK09918.Regs.ST1)
            st2=self.read(AK09918.Regs.ST2)
            if (st1&1)!=0:
                break


    def get_magnetometer(self):
        """Get magnetometer values (in gauss)
        """
        if not self.is_initialised:
            self._startup()
        self._wait_for_data_ready()
        return self.read_16_bit_values(AK09918.Regs.MAG_DATA, 3,multiplier=0.15)


if __name__ == "__main__":
    s=AK09918()
    format_all=":".join(["{:-7.2f}"]*3)



    while True:
        print(format_all.format(*s.get_magnetometer()))
        time.sleep(0.01)
else:
    IMUBase.register_sensor_type(AK09918)
