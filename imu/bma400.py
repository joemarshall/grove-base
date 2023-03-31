#!/usr/bin/python3
# -*- coding: ascii -*-

import time
from enum import IntEnum
if __name__!="__main__":
    from .imubase import *
else:
    import imubase

X_AXIS=0
Y_AXIS=1
Z_AXIS=2

import smbus2 as smbus

class BMA400(IMUBase):
    ADDRESS=0x15
    ID_REG_VALUE=(0x0,0x90)


    @staticmethod
    def has_accelerometer():
        return True


    def __init__(self):
        self.is_initialised=False


    class Regs(IntEnum):
        # BMA400 register definitions
        BMA400_CHIPID                   =0x00
        BMA400_ERR_REG                  =0x02
        BMA400_STATUS                   =0x03

        BMA400_ACC_X_LSB                =0x04
        BMA400_ACC_X_MSB                =0x05
        BMA400_ACC_Y_LSB                =0x06
        BMA400_ACC_Y_MSB                =0x07
        BMA400_ACC_Z_LSB                =0x08
        BMA400_ACC_Z_MSB                =0x09

        BMA400_SENSOR_TIME_0            =0x0A
        BMA400_SENSOR_TIME_1            =0x0B
        BMA400_SENSOR_TIME_2            =0x0C

        BMA400_EVENT                    =0x0D

        BMA400_INT_STAT0                =0x0E
        BMA400_INT_STAT1                =0x0F
        BMA400_INT_STAT2                =0x10

        BMA400_TEMP_DATA                =0x11

        BMA400_FIFO_LENGTH_0            =0x12
        BMA400_FIFO_LENGTH_1            =0x13
        BMA400_FIFO_DATA                =0x14

        BMA400_STEP_CNT_0               =0x15
        BMA400_STEP_CNT_1               =0x16
        BMA400_STEP_CNT_2               =0x17
        BMA400_STEP_STAT                =0x18

        BMA400_ACC_CONFIG_0             =0x19
        BMA400_ACC_CONFIG_1             =0x1A
        BMA400_ACC_CONFIG_2             =0x1B

        BMA400_INT_CONFIG_0             =0x1F
        BMA400_INT_CONFIG_1             =0x20
        BMA400_INT_1_MAP                =0x21
        BMA400_INT_2_MAP                =0x22
        BMA400_INT_1_2_MAP              =0x23
        BMA400_INT_1_2_CTRL             =0x24

        BMA400_FIFO_CONFIG_0            =0x26
        BMA400_FIFO_CONFIG_1            =0x27
        BMA400_FIFO_CONFIG_2            =0x28
        BMA400_FIFO_PWR_CONFIG          =0x29

        BMA400_AUTO_LOW_POW_0           =0x2A
        BMA400_AUTO_LOW_POW_1           =0x2B
        BMA400_AUTO_WAKE_UP_0           =0x2C
        BMA400_AUTO_WAKE_UP_1           =0x2D
        BMA400_WAKE_UP_INT_CONFIG_0     =0x2F
        BMA400_WAKE_UP_INT_CONFIG_1     =0x30
        BMA400_WAKE_UP_INT_CONFIG_2     =0x31
        BMA400_WAKE_UP_INT_CONFIG_3     =0x32
        BMA400_WAKE_UP_INT_CONFIG_4     =0x33

        BMA400_ORIENTCH_CONFIG_0        =0x35
        BMA400_ORIENTCH_CONFIG_1        =0x36
        BMA400_ORIENTCH_CONFIG_2        =0x37
        BMA400_ORIENTCH_CONFIG_3        =0x38
        BMA400_ORIENTCH_CONFIG_4        =0x39
        BMA400_ORIENTCH_CONFIG_5        =0x3A
        BMA400_ORIENTCH_CONFIG_6        =0x3B
        BMA400_ORIENTCH_CONFIG_7        =0x3C
        BMA400_ORIENTCH_CONFIG_8        =0x3D
        BMA400_ORIENTCH_CONFIG_9        =0x3E

        BMA400_GEN_1_INT_CONFIG_0       =0x3F
        BMA400_GEN_1_INT_CONFIG_1       =0x40
        BMA400_GEN_1_INT_CONFIG_2       =0x41
        BMA400_GEN_1_INT_CONFIG_3       =0x42
        BMA400_GEN_1_INT_CONFIG_3_1     =0x43
        BMA400_GEN_1_INT_CONFIG_4       =0x44
        BMA400_GEN_1_INT_CONFIG_5       =0x45
        BMA400_GEN_1_INT_CONFIG_6       =0x46
        BMA400_GEN_1_INT_CONFIG_7       =0x47
        BMA400_GEN_1_INT_CONFIG_8       =0x48
        BMA400_GEN_1_INT_CONFIG_9       =0x49

        BMA400_GEN_2_INT_CONFIG_0       =0x4A
        BMA400_GEN_2_INT_CONFIG_1       =0x4B
        BMA400_GEN_2_INT_CONFIG_2       =0x4C
        BMA400_GEN_2_INT_CONFIG_3       =0x4D
        BMA400_GEN_2_INT_CONFIG_3_1     =0x4E
        BMA400_GEN_2_INT_CONFIG_4       =0x4F
        BMA400_GEN_2_INT_CONFIG_5       =0x50
        BMA400_GEN_2_INT_CONFIG_6       =0x51
        BMA400_GEN_2_INT_CONFIG_7       =0x52
        BMA400_GEN_2_INT_CONFIG_8       =0x53
        BMA400_GEN_2_INT_CONFIG_9       =0x54

        BMA400_ACT_CH_CONFIG_0          =0x55
        BMA400_ACT_CH_CONFIG_1          =0x56

        BMA400_TAP_CONFIG_0             =0x57
        BMA400_TAP_CONFIG_1             =0x58

        BMA400_STEP_COUNTER_CONFIG_0    =0x59
        BMA400_STEP_COUNTER_CONFIG_1    =0x5A
        BMA400_STEP_COUNTER_CONFIG_2    =0x5B
        BMA400_STEP_COUNTER_CONFIG_3    =0x5C
        BMA400_STEP_COUNTER_CONFIG_4    =0x5D
        BMA400_STEP_COUNTER_CONFIG_5    =0x5E
        BMA400_STEP_COUNTER_CONFIG_6    =0x5F
        BMA400_STEP_COUNTER_CONFIG_7    =0x60
        BMA400_STEP_COUNTER_CONFIG_8    =0x61
        BMA400_STEP_COUNTER_CONFIG_9    =0x62
        BMA400_STEP_COUNTER_CONFIG_10   =0x63
        BMA400_STEP_COUNTER_CONFIG_11   =0x64
        BMA400_STEP_COUNTER_CONFIG_12   =0x65
        BMA400_STEP_COUNTER_CONFIG_13   =0x66
        BMA400_STEP_COUNTER_CONFIG_14   =0x67
        BMA400_STEP_COUNTER_CONFIG_15   =0x68
        BMA400_STEP_COUNTER_CONFIG_16   =0x69
        BMA400_STEP_COUNTER_CONFIG_17   =0x6A
        BMA400_STEP_COUNTER_CONFIG_18   =0x6B
        BMA400_STEP_COUNTER_CONFIG_19   =0x6C
        BMA400_STEP_COUNTER_CONFIG_20   =0x6D
        BMA400_STEP_COUNTER_CONFIG_21   =0x6E
        BMA400_STEP_COUNTER_CONFIG_22   =0x6F
        BMA400_STEP_COUNTER_CONFIG_23   =0x70
        BMA400_STEP_COUNTER_CONFIG_24   =0x71

        BMA400_IF_CONF                  =0x7C
        BMA400_SELF_TEST                =0x7D
        BMA400_CMD                      =0x7E

    class OversamplingRate(IntEnum):
        OSR_LOWEST=0x00
        OSR_LOW= 0x01
        OSR_HIGH=0x03
        OSR_HIGHEST=0x03
    class FilterType(IntEnum):
        ACC_FILT1 = 0x00
        ACC_FILT2 = 0x01
        ACC_FILT_LP = 0x02
        ACC_FILT11  = 0X03

    class DataRate(IntEnum):
        ODR_12 = 0x00
        ODR_25 = 0x06
        ODR_50 = 0x07
        ODR_100 = 0x08
        ODR_200 = 0x09
        ODR_400 = 0x0A
        ODR_800 = 0x0B

    class Range(IntEnum):
        RANGE_2G = 0x00
        RANGE_4G = 0x01
        RANGE_8G = 0x02
        RANGE_16G = 0x03

    class PowerMode(IntEnum):
        SLEEP = 0x00
        LOW_POWER = 0x01
        NORMAL = 0x02

    def _startup(self):
        self.start_i2c(big_endian=False)
        self.bus=smbus.SMBus(1)
        self.setPoweMode(BMA400.PowerMode.NORMAL)
        self.setFullScaleRange(BMA400.Range.RANGE_4G)
        self.setOutputDataRate(BMA400.DataRate.ODR_100)
        self.setOSR(BMA400.OversamplingRate.OSR_LOWEST)
        self.setFilter(BMA400.FilterType.ACC_FILT1)
        # the following is interrupt stuff - I don't think it
        # is needed unless you're using interrupt pin
#        self.enableGen1()
#        self.setRouteGen1()
#        self.configIntPin()
#        self.setLatch()
        self.is_initialised=True

    def setLatch(self):
        data=self.read(BMA400.Regs.BMA400_INT_CONFIG_1)
        data=data&0b01111111
        data=data|0x80
        self.write(BMA400.Regs.BMA400_INT_CONFIG_1, data)

    def configIntPin(self):
        data=self.read(BMA400.Regs.BMA400_INT_1_2_CTRL)
        data=data&0b11011101
        data=data|0b00100010
        self.write(BMA400.Regs.BMA400_INT_1_2_CTRL,data)

    def setRouteGen1(self):
        data=self.read(BMA400.Regs.BMA400_INT_1_MAP)
        data=data&0b11111011
        data=data|0b00000100
        self.write(BMA400.Regs.BMA400_INT_1_MAP,data)

    def enableGen1(self):
        data=self.read(BMA400.Regs.BMA400_INT_CONFIG_0)
        data=data&0b11111011
        data=data|0b00000100
        self.write(BMA400.Regs.BMA400_INT_CONFIG_0,data)

    def setOSR(self,oversampling):
        data=self.read(BMA400.Regs.BMA400_ACC_CONFIG_1)
        data=data&0b11001111
        data=data|(oversampling<<4)
        self.write(BMA400.Regs.BMA400_ACC_CONFIG_1,data)

    def setPoweMode(self,mode):
        data = self.read(BMA400.Regs.BMA400_ACC_CONFIG_0)
        data = (data & 0xfc) | mode
        self.write(BMA400.Regs.BMA400_ACC_CONFIG_0, data)

    def setFullScaleRange(self,range):
        scales={BMA400.Range.RANGE_2G:2,BMA400.Range.RANGE_4G:4,BMA400.Range.RANGE_8G:8,BMA400.Range.RANGE_16G:16}
        self.acc_scale=scales[range]

        data = self.read(BMA400.Regs.BMA400_ACC_CONFIG_1)
        data = (data & 0x3f) | (range << 6)
        self.write(BMA400.Regs.BMA400_ACC_CONFIG_1, data)

    def setOutputDataRate(self,odr):
        data = self.read(BMA400.Regs.BMA400_ACC_CONFIG_1)
        data = (data & 0xf0) | odr
        self.write(BMA400.Regs.BMA400_ACC_CONFIG_1, data)       
    
    def setFilter(self,filter):
        data= self.read(BMA400.Regs.BMA400_ACC_CONFIG_2)
        data= data & 0b11110011
        data= data| (filter<<2)
        self.write(BMA400.Regs.BMA400_ACC_CONFIG_2,data)

    def get_accel(self):
        """Get accelerometer values (in multiples of g)        
        """
        if not self.is_initialised:
            self._startup()
        multiplier=self.acc_scale/2048 
        return self.read_signed_12_bit(BMA400.Regs.BMA400_ACC_X_LSB,3,multiplier)

if __name__=="__main__":
    s=BMA400()
    while True:
        print(s.get_accel())
        time.sleep(0.01)
else:
    IMUBase.register_sensor_type(BMA400)
