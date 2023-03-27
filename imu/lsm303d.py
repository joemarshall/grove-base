#!/usr/bin/python3
# -*- coding: ascii -*-

import time,sys,struct
from enum import IntEnum
from .imubase import *

X_AXIS=0
Y_AXIS=1
Z_AXIS=2

import smbus2 as smbus

class LSM303D(IMUBase):

    @staticmethod
    def has_accelerometer():
        return True

    @staticmethod
    def has_magnetometer():
        return True

    def __init__(self):
        self.is_initialised=False

    class Regs(IntEnum):
        # LSM303D Register definitions 
        CTRL_REG1_A=0x20
        CTRL_REG2_A=0x21
        CTRL_REG3_A=0x22
        CTRL_REG4_A=0x23
        CTRL_REG5_A=0x24
        HP_FILTER_RESET_A=0x25
        REFERENCE_A=0x26
        STATUS_REG_A=0x27
        INT1_CFG_A=0x30
        INT1_SOURCE_A=0x31
        INT1_THS_A=0x32
        INT1_DURATION_A=0x33
        CRA_REG_M=0x00
        CRB_REG_M=0x01#refer to the Table 58 of the datasheet of LSM303DLM
        MAG_SCALE_1_3=0x20#full-scale is +/-1.3Gauss
        MAG_SCALE_1_9=0x40#+/-1.9Gauss
        MAG_SCALE_2_5=0x60#+/-2.5Gauss
        MAG_SCALE_4_0=0x80#+/-4.0Gauss
        MAG_SCALE_4_7=0xa0#+/-4.7Gauss
        MAG_SCALE_5_6=0xc0#+/-5.6Gauss
        MAG_SCALE_8_1=0xe0#+/-8.1Gauss
        MR_REG_M=0x02
        SR_REG_M=0x09
        IRA_REG_M=0x0A
        IRB_REG_M=0x0B
        IRC_REG_M=0x0C
        SIX_AXIS_ACCEL_ADDR=0x1e
        STATUS_REG_M    = 0x07
        OUT_X_L_M       = 0x08
        OUT_X_H_M       = 0x09
        OUT_Y_L_M       = 0x0A
        OUT_Y_H_M       = 0x0B
        OUT_Z_L_M       = 0x0C
        OUT_Z_H_M       = 0x0D    
        OUT_X_L_A       = 0x28
        OUT_X_H_A       = 0x29
        OUT_Y_L_A       = 0x2A
        OUT_Y_H_A       = 0x2B
        OUT_Z_L_A       = 0x2C
        OUT_Z_H_A       = 0x2D
        CTRL_REG0       = 0x1F
        CTRL_REG1       = 0x20
        CTRL_REG2       = 0x21
        CTRL_REG3       = 0x22
        CTRL_REG4       = 0x23
        CTRL_REG5       = 0x24
        CTRL_REG6       = 0x25
        CTRL_REG7       = 0x26
        MAG_SCALE_2 	= 0x00 #full-scale is +/-2Gauss
        MAG_SCALE_4 	= 0x20 #+/-4Gauss
        MAG_SCALE_8 	= 0x40 #+/-8Gauss
        MAG_SCALE_12 	= 0x60 #+/-12Gauss



    def _startup(self):
        self.bus=smbus.SMBus(1)
        self.write(0x7f, LSM303D.Regs.CTRL_REG1)               # ODR=200hz, all accel axes on, block data read
        self.write(0x8, LSM303D.Regs.CTRL_REG2)      # scale -+4g
        self.ACCEL_SCALE=4
        self.MAG_SCALE=4
        self.write(0x00, LSM303D.Regs.CTRL_REG3)           # no interrupt 1
        self.write(0x00, LSM303D.Regs.CTRL_REG4)           # no interrupt 2
        self.write(0x74, LSM303D.Regs.CTRL_REG5)             # high quality, read 100hz
        self.write(0x28, LSM303D.Regs.CTRL_REG6) # 100hz
        self.write(0x0, LSM303D.Regs.CTRL_REG7)             # continuous conversion, not low power
        self.is_initialised=True


    def write(self,data, address):
        self.bus.write_byte_data(LSM303D.Regs.SIX_AXIS_ACCEL_ADDR,address,data)

    def read(self, address):
        return self.bus.read_byte_data(LSM303D.Regs.SIX_AXIS_ACCEL_ADDR,address)

    def get_accel(self):
        """Get accelerometer values (in multiples of g)        
        """
        if not self.is_initialised:
            self._startup()
        x= self.read_signed_16_bit(LSM303D.Regs.OUT_X_L_A,LSM303D.Regs.OUT_X_H_A)
        y= self.read_signed_16_bit(LSM303D.Regs.OUT_Y_L_A,LSM303D.Regs.OUT_Y_H_A)
        z= self.read_signed_16_bit(LSM303D.Regs.OUT_Z_L_A,LSM303D.Regs.OUT_Z_H_A)        
        multiplier=self.ACCEL_SCALE/(2.**15.) 
        return (x*multiplier,y*multiplier,z*multiplier)


    def get_magnetometer(self):
        """Get magnetometer values. 
        """
        if not self.is_initialised:
            self._startup()
        x= self.read_signed_16_bit(LSM303D.Regs.OUT_X_L_M,LSM303D.Regs.OUT_X_H_M)
        y= self.read_signed_16_bit(LSM303D.Regs.OUT_Y_L_M,LSM303D.Regs.OUT_Y_H_M)
        z= self.read_signed_16_bit(LSM303D.Regs.OUT_Z_L_M,LSM303D.Regs.OUT_Z_H_M)
        multiplier=self.MAG_SCALE/(2.**15.) 
        return (x*multiplier,y*multiplier,z*multiplier)
        
    def read_signed_16_bit(self,arg1,arg2):
        bytes=struct.pack("BB",self.read(arg1),self.read(arg2))
        return struct.unpack("<h",bytes)[0]

IMUBase.register_sensor_type(0x1e,LSM303D)

if __name__=="__main__":
    s=LSM303D()
    while True:
        print((s.get_accel(),s.get_magnetometer()))
        time.sleep(0.01)
