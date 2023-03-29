#!/usr/bin/python3
# -*- coding: ascii -*-

import smbus2 as smbus
import time
from enum import IntEnum
if __name__!="__main__":
    from .imubase import *
else:
    class IMUBase:
        pass
import struct

X_AXIS = 0
Y_AXIS = 1
Z_AXIS = 2


class MPU9250(IMUBase):

    @staticmethod
    def has_accelerometer():
        return True

    @staticmethod
    def has_gyro():
        return True

    @staticmethod
    def has_magnetometer():
        return True

    def __init__(self):
        self.is_initialised = False
        self.regs = MPU9250.Regs

    ADDRESS = 0x68

    class Regs(IntEnum):
        # MPU9250 register definitions
        ACC_DATA = 0x3B  # big endian!
        GYRO_DATA = 0x43
        MAG_DATA = 0x49
        USER_CONTROL = 0x6a
        ACC_CONFIG = 0x1C
        ACC_CONFIG2 = 0x1D
        GYRO_CONFIG = 0x1B
        GYRO_CONFIG2 = 0x1A

        I2C_MASTER_CONTROL = 0x24
        I2C_SLAVE0_ADDR = 0x25
        I2C_SLAVE0_REG = 0x26
        I2C_SLAVE0_CTRL = 0x27

        I2C_SLAVE0_DATA_OUT = 0x63

        SAMPLE_RATE_DIVIDER = 0x19

        POWERMANAGEMENT1 = 0x6b
        POWERMANAGEMENT2 = 0x6c
        WHOAMI = 0x75

    # AK8963 magnetometer included on chip
    # NB magnetometer data is little endian so 
    # needs swapping on read (this is done by
    # the MPU9250 chip, which means at the end
    # all readings are in the same format)
    class AK8963Regs(IntEnum):
        MAG_DATA = 0x03  
        CTL = 0x0A
        FUSE_X = 0x10
        FUSE_Y = 0x11
        FUSE_Z = 0x12

    class AccelRange(IntEnum):
        RANGE_2G = 0x00
        RANGE_4G = 0x08
        RANGE_8G = 0x10
        RANGE_16G = 0x18

    class GyroRange(IntEnum):
        RANGE_250DPS = 0x00
        RANGE_500DPS = 0x08
        RANGE_1000DPS = 0x10
        RANGE_2000DPS = 0x18

    class LowPassFilter(IntEnum):
        LP_184 = 0x01
        LP_92 = 0x02
        LP_41 = 0x03
        LP_20 = 0x04
        LP_10 = 0x05
        LP_5 = 0x06

    def _startup(self):
        self.bus = smbus.SMBus(1)
        if self.read(MPU9250.Regs.WHOAMI) != 0x71:
            print("WOO")
            raise IOError("MPU9250 returned incorrect device ID:",self.read(MPU9250.Regs.WHOAMI))

        # set clock to PLL / internal if PLL is not available yet
        self.write(MPU9250.Regs.POWERMANAGEMENT1, 0x01)
        # enable everything (accel and gyro)
        self.write(MPU9250.Regs.POWERMANAGEMENT2, 0x0)
        # enable I2C master for magnetometer
        self.write(MPU9250.Regs.USER_CONTROL, 0x20)
        # 400hz i2c on secondary bus
        self.write(MPU9250.Regs.I2C_MASTER_CONTROL, 0x0d)
        self.write(MPU9250.Regs.SAMPLE_RATE_DIVIDER,
                   0x0)  # sample rate divider = 1


        self.write(MPU9250.Regs.ACC_CONFIG, MPU9250.AccelRange.RANGE_4G)
        self.acc_scale=4.0

        self.write(MPU9250.Regs.GYRO_CONFIG, MPU9250.GyroRange.RANGE_1000DPS)
        self.gyro_scale=1000.0

        self.write(MPU9250.Regs.ACC_CONFIG2, MPU9250.LowPassFilter.LP_184)
        self.write(MPU9250.Regs.GYRO_CONFIG2, MPU9250.LowPassFilter.LP_184)

        # configure magnetometer
        # address of magnetometer and read flag
        self.write(MPU9250.Regs.I2C_SLAVE0_ADDR, 0x0c | 0x80)

        self.write_ak8963(MPU9250.AK8963Regs.CTL, 0)  # power down between any mode changes
        time.sleep(0.01)
        # fuse rom access mode - gets calibration data
        self.write_ak8963(MPU9250.AK8963Regs.CTL, 0b11111)
        time.sleep(0.01)
        mag_scales=self.read_ak8963(MPU9250.AK8963Regs.FUSE_X, 3)
        self.mag_multipliers=[((k - 128.0)/256.0 + 1.0)
                               *(4912.0/32760.0) for k in mag_scales]
        self.write_ak8963(MPU9250.AK8963Regs.CTL, 0)  # power down
        time.sleep(0.01)
        # 16 bit continous read mode 100hz
        self.write_ak8963(MPU9250.AK8963Regs.CTL, 0b10110)
        time.sleep(0.01)
        self.is_initialised=True
        self.set_ak8963_mag_read()

    # configure mpu9250 to continuously read mag data from ak8963 
    def set_ak8963_mag_read(self):        
        self.write(MPU9250.Regs.I2C_SLAVE0_REG,
            MPU9250.AK8963Regs.MAG_DATA)
        # 7 bytes read from data address, swap endian 
        # n.b. we only use 6 bytes, but need to read status byte
        # or else the AK8963 won't update the value
        self.write(MPU9250.Regs.I2C_SLAVE0_CTRL, 0b11010111)
        self.write(MPU9250.Regs.I2C_SLAVE0_ADDR, 0x0c|0x80) # read from device 0x0c


    def write_ak8963(self,address,data):
        self.write(MPU9250.Regs.I2C_SLAVE0_REG, address)
        self.write(MPU9250.Regs.I2C_SLAVE0_DATA_OUT,data)
        self.write(MPU9250.Regs.I2C_SLAVE0_ADDR, 0x0c) # write to device 0x0c
        self.write(MPU9250.Regs.I2C_SLAVE0_CTRL,0b10000001)# 1 byte, enabled
        time.sleep(0.01)
        read_data=self.read_ak8963(address,1)
        if read_data!=[data]:
            print("Failed write to ak8963:",address,data,read_data)

    def read_ak8963(self,address,count):
        assert(count<16)
        self.write(MPU9250.Regs.I2C_SLAVE0_ADDR, 0x0c|0x80) # read from device 0x0c
        self.write(MPU9250.Regs.I2C_SLAVE0_REG, address)
        self.write(MPU9250.Regs.I2C_SLAVE0_CTRL, count | 0b11000000)
        time.sleep(0.01)
        data=self.bus.read_i2c_block_data(MPU9250.ADDRESS,MPU9250.Regs.MAG_DATA,count)
        return data



    def write(self, address,data ):
        self.bus.write_byte_data(MPU9250.ADDRESS, address, data)

    def read(self, address):
        return self.bus.read_byte_data(MPU9250.ADDRESS, address)

    def read_bytes(self, address, length):
        return self.bus.read_i2c_block_data(MPU9250.ADDRESS, address, length)

    def read_16_bit_words(self, address, length):
        byte_data=self.read_bytes(address, length*2)
        format=">"+("h"*length)
        word_data=struct.unpack(format, bytearray(byte_data))
        return word_data

    def get_accel(self):
        """Get accelerometer values (in multiples of g)
        """
        if not self.is_initialised:
            self._startup()
        multiplier=self.acc_scale/32768
        data=self.read_16_bit_words(MPU9250.Regs.ACC_DATA, 3)
        return tuple(k*multiplier for k in data)

    def get_gyro(self):
        """Get gyro values (in multiples of dps)
        """
        if not self.is_initialised:
            self._startup()
        multiplier=self.gyro_scale/32768
        data=self.read_16_bit_words(MPU9250.Regs.GYRO_DATA, 3)
        return tuple(k*multiplier for k in data)

    def get_magnetometer(self):
        """Get magnetometer values (in gauss)
        """
        if not self.is_initialised:
            self._startup()
        data=self.read_16_bit_words(MPU9250.Regs.MAG_DATA, 3)
        return tuple(k*multiplier for k, multiplier in zip(data, self.mag_multipliers))




if __name__ == "__main__":
    s=MPU9250()
    format_acc=":".join(["{:-7.2f}"]*3)
    format_mag=":".join(["{:-7.2f}"]*3)
    format_gyro=":".join(["{:-7.1f}"]*3)
    format_all="  ".join([format_acc,format_mag,format_gyro])



    while True:
        print(format_all.format(*s.get_accel(),*s.get_magnetometer(),*s.get_gyro()))
        time.sleep(0.01)
else:
    IMUBase.register_sensor_type(MPU9250.ADDRESS, MPU9250)
