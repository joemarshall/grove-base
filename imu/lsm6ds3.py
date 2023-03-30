#!/usr/bin/python3
# -*- coding: ascii -*-

import time,sys,struct
from enum import IntEnum
if __name__!="__main__":
    from .imubase import *
else:
    from imubase import *

X_AXIS=0
Y_AXIS=1
Z_AXIS=2

import smbus2 as smbus

class LSM6DS3(IMUBase):
    ADDRESS = 0x6a
    ID_REG_VALUE=(0x0f,0x69)

    @staticmethod
    def has_accelerometer():
        return True

    @staticmethod
    def has_gyro():
        return True

    def __init__(self):
        self.is_initialised=False

    class Regs(IntEnum):
        # LSM6DS3 Register definitions 
        ACCEL_X_LOW = 0x28
        GYRO_X_LOW = 0x22
        CTRL1_XL = 0x10
        CTRL2_G = 0x11


    def _startup(self):
        self.start_i2c(big_endian=False)

        self.write(LSM6DS3.Regs.CTRL1_XL,0b01011000) # 4G, 200hz
        self.write(LSM6DS3.Regs.CTRL2_G,0b01011000)      # gyro scale -+1000DPS 200hz
        self.ACCEL_SCALE=4.0
        self.GYRO_SCALE=1000.0
        self.is_initialised=True


    def get_accel(self):
        """Get accelerometer values (in multiples of g)        
        """
        if not self.is_initialised:
            self._startup()
        multiplier=self.ACCEL_SCALE/(2.**15.) 
        return self.read_16_bit_values(LSM6DS3.Regs.ACCEL_X_LOW,3,multiplier)

    def get_gyro(self):
        """Get magnetometer values. 
        """
        if not self.is_initialised:
            self._startup()
        multiplier=self.GYRO_SCALE/(2.**15.) 
        return self.read_16_bit_values(LSM6DS3.Regs.GYRO_X_LOW,3,multiplier)
        

IMUBase.register_sensor_type(LSM6DS3)

if __name__=="__main__":
    s=LSM6DS3()
    format_acc=":".join(["{:-7.2f}"]*3)
    format_mag=":".join(["{:-7.2f}"]*3)
    while True:
        print(format_acc.format(*s.get_accel()),format_mag.format(*s.get_magnetometer()))
        time.sleep(0.01)
