#!/usr/bin/python3
# -*- coding: ascii -*-

import smbus2 as smbus
import time
from enum import IntEnum
if __name__!="__main__":
    from .imubase import *
else:
    from imubase import *

X_AXIS = 0
Y_AXIS = 1
Z_AXIS = 2


class ICM20600(IMUBase):
    ADDRESS = 0x69
    ID_REG_VALUE=(0x75,0x11)

    @staticmethod
    def has_accelerometer():
        return True

    @staticmethod
    def has_gyro():
        return True

    @staticmethod
    def has_magnetometer():
        return False

    def __init__(self):
        self.is_initialised = False
        self.regs = ICM20600.Regs


    class Regs(IntEnum):
        # ICM20600 register definitions
        XG_OFFS_TC_H = 0x04
        XG_OFFS_TC_L = 0x05
        YG_OFFS_TC_H = 0x07
        YG_OFFS_TC_L = 0x08
        ZG_OFFS_TC_H = 0x0a
        ZG_OFFS_TC_L = 0x0b
        SELF_TEST_X_ACCEL = 0x0d
        SELF_TEST_Y_ACCEL = 0x0e
        SELF_TEST_Z_ACCEL = 0x0f
        XG_OFFS_USRH = 0x13
        XG_OFFS_USRL = 0x14
        YG_OFFS_USRH = 0x15
        YG_OFFS_USRL = 0x16
        ZG_OFFS_USRH = 0x17
        ZG_OFFS_USRL = 0x18
        SMPLRT_DIV = 0x19
        CONFIG = 0x1a
        GYRO_CONFIG = 0x1b
        ACCEL_CONFIG = 0x1c
        ACCEL_CONFIG2 = 0x1d
        GYRO_LP_MODE_CFG = 0x1e
        ACCEL_WOM_X_THR = 0x20
        ACCEL_WOM_Y_THR = 0x21
        ACCEL_WOM_Z_THR = 0x22
        FIFO_EN = 0x23
        FSYNC_INT = 0x36
        INT_PIN_CFG = 0x37
        INT_ENABLE = 0x38
        FIFO_WM_INT_STATUS = 0x39
        INT_STATUS = 0x3a
        ACCEL_XOUT_H = 0x3b
        ACCEL_XOUT_L = 0x3c
        ACCEL_YOUT_H = 0x3d
        ACCEL_YOUT_L = 0x3e
        ACCEL_ZOUT_H = 0x3f
        ACCEL_ZOUT_L = 0x40
        TEMP_OUT_H = 0x41
        TEMP_OUT_L = 0x42
        GYRO_XOUT_H = 0x43
        GYRO_XOUT_L = 0x44
        GYRO_YOUT_H = 0x45
        GYRO_YOUT_L = 0x46
        GYRO_ZOUT_H = 0x47
        GYRO_ZOUT_L = 0x48
        SELF_TEST_X_GYRO = 0x50
        SELF_TEST_Y_GYRO = 0x51
        SELF_TEST_Z_GYRO = 0x52
        FIFO_WM_TH1 = 0x60
        FIFO_WM_TH2 = 0x61
        SIGNAL_PATH_RESET = 0x68
        ACCEL_INTEL_CTRL = 0x69
        USER_CTRL = 0x6A
        PWR_MGMT_1 = 0x6b
        PWR_MGMT_2 = 0x6c
        I2C_IF = 0x70
        FIFO_COUNTH = 0x72
        FIFO_COUNTL = 0x73
        FIFO_R_W = 0x74
        WHO_AM_I = 0x75
        XA_OFFSET_H = 0x77
        XA_OFFSET_L = 0x78
        YA_OFFSET_H = 0x7a
        YA_OFFSET_L = 0x7b
        ZA_OFFSET_H = 0x7d
        ZA_OFFSET_L = 0x7e

        FIFO_EN_BIT = (1 << 6)
        FIFO_RST_BIT = (1 << 2)
        RESET_BIT = (1 << 0)
        DEVICE_RESET_BIT = (1 << 7)

        ACC_DATA = 0x3b
        GYRO_DATA = 0x43


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
        LP_218 = 0x00
        LP_184 = 0x01
        LP_99 = 0x02
        LP_44 = 0x03
        LP_21 = 0x04
        LP_10 = 0x05
        LP_5 = 0x06
        LP_420 = 0x07

    def _startup(self):
        self.start_i2c(big_endian=True)
        if self.read(ICM20600.Regs.WHO_AM_I) != 0x11:
            raise IOError("ICM20600 returned incorrect device ID:",self.read(ICM20600.Regs.WHO_AM_I))

        # set clock to PLL / internal if PLL is not available yet, and enable everything
        self.write(ICM20600.Regs.PWR_MGMT_1, 0x01)
        # enable everything (accel and gyro)
        self.write(ICM20600.Regs.PWR_MGMT_2, 0x0)
        # enable I2C master for magnetometer
#        self.write(ICM20600.Regs.USER_CONTROL, 0x20)
        # 400hz i2c on secondary bus
        # config - gyro lowpass filter at 176hz,    

        # reset
        temp = self.read(ICM20600.Regs.USER_CTRL)
        self.write(ICM20600.Regs.USER_CTRL, temp | ICM20600.Regs.RESET_BIT)


        self.write(ICM20600.Regs.CONFIG, 0x01)
        self.write(ICM20600.Regs.SMPLRT_DIV, 0x0)  # sample rate divider = 1

        # everything in the acc_config should be zero except for range
        self.write(ICM20600.Regs.ACCEL_CONFIG, ICM20600.AccelRange.RANGE_4G)
        self.acc_scale=4.0

        # everything in the gyro_config should be zero except for range
        self.write(ICM20600.Regs.GYRO_CONFIG, ICM20600.GyroRange.RANGE_1000DPS)
        self.gyro_scale=1000.0

        self.write(ICM20600.Regs.ACCEL_CONFIG2, ICM20600.LowPassFilter.LP_184)


        time.sleep(0.01)
        self.is_initialised=True

    def get_accel(self):
        """Get accelerometer values (in multiples of g)
        """
        if not self.is_initialised:
            self._startup()
        multiplier=self.acc_scale/32768
        return self.read_16_bit_values(ICM20600.Regs.ACC_DATA, 3,multiplier=multiplier)

    def get_gyro(self):
        """Get gyro values (in dps)
        """
        if not self.is_initialised:
            self._startup()
        multiplier=self.gyro_scale/32768
        return self.read_16_bit_values(ICM20600.Regs.GYRO_DATA, 3,multiplier=multiplier)


if __name__ == "__main__":
    s=ICM20600()
    format_acc=":".join(["{:-7.2f}"]*3)
    format_gyro=":".join(["{:-7.1f}"]*3)
    format_all="  ".join([format_acc,format_gyro])

    while True:
        print(format_all.format(*s.get_accel(),*s.get_gyro()))
        time.sleep(0.01)
else:
    IMUBase.register_sensor_type(ICM20600)
