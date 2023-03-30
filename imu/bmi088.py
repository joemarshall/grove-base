import smbus2 as smbus

import struct 
import time 
from enum import IntEnum
if __name__=="__main__":
    from imubase import *
else:
    from .imubase import *

class BMI088(IMUBase):
    ADDRESS = [0x19,0x69]
    ID_REG_VALUE=[(0x00,0x1E),(0x00,0x0f)]


    @staticmethod
    def has_accelerometer():
        return True

    @staticmethod
    def has_gyro():
        return True
    
    class Regs(IntEnum):
        BMI088_ACC_ADDRESS         =0x19

        BMI088_ACC_CHIP_ID         =0x00 
        BMI088_ACC_ERR_REG         =0x02
        BMI088_ACC_STATUS          =0x03

        BMI088_ACC_X_LSB           =0x12
        BMI088_ACC_X_MSB           =0x13
        BMI088_ACC_Y_LSB           =0x14
        BMI088_ACC_Y_MSB           =0x15
        BMI088_ACC_Z_LSB           =0x16
        BMI088_ACC_Z_MSB           =0x17

        BMI088_ACC_SENSOR_TIME_0   =0x18
        BMI088_ACC_SENSOR_TIME_1   =0x19
        BMI088_ACC_SENSOR_TIME_2   =0x1A

        BMI088_ACC_INT_STAT_1      =0x1D

        BMI088_ACC_TEMP_MSB        =0x22
        BMI088_ACC_TEMP_LSB        =0x23

        BMI088_ACC_CONF            =0x40
        BMI088_ACC_RANGE           =0x41

        BMI088_ACC_INT1_IO_CTRL    =0x53
        BMI088_ACC_INT2_IO_CTRL    =0x54
        BMI088_ACC_INT_MAP_DATA    =0x58

        BMI088_ACC_SELF_TEST       =0x6D

        BMI088_ACC_PWR_CONF        =0x7C
        BMI088_ACC_PWR_CTRl        =0x7D

        BMI088_ACC_SOFT_RESET      =0x7E

        BMI088_GYRO_ADDRESS            =0x69

        BMI088_GYRO_CHIP_ID            =0x00 

        BMI088_GYRO_RATE_X_LSB         =0x02
        BMI088_GYRO_RATE_X_MSB         =0x03
        BMI088_GYRO_RATE_Y_LSB         =0x04
        BMI088_GYRO_RATE_Y_MSB         =0x05
        BMI088_GYRO_RATE_Z_LSB         =0x06
        BMI088_GYRO_RATE_Z_MSB         =0x07

        BMI088_GYRO_INT_STAT_1         =0x0A

        BMI088_GYRO_RANGE              =0x0F
        BMI088_GYRO_BAND_WIDTH         =0x10

        BMI088_GYRO_LPM_1              =0x11

        BMI088_GYRO_SOFT_RESET         =0x14

        BMI088_GYRO_INT_CTRL           =0x15
        BMI088_GYRO_INT3_INT4_IO_CONF  =0x16
        BMI088_GYRO_INT3_INT4_IO_MAP   =0x18

        BMI088_GYRO_SELF_TEST          =0x3C

        # accel range
        RANGE_3G = 0x00 
        RANGE_6G = 0x01 
        RANGE_12G = 0x02 
        RANGE_24G = 0x03 

        #output data rate
        ODR_12 = 0x05
        ODR_25 = 0x06 
        ODR_50 = 0x07 
        ODR_100 = 0x08 
        ODR_200 = 0x09 
        ODR_400 = 0x0A 
        ODR_800 = 0x0B 
        ODR_1600 = 0x0C 

        # gyro range
        RANGE_2000 = 0x00
        RANGE_1000 = 0x01 
        RANGE_500 = 0x02 
        RANGE_250 = 0x03 
        RANGE_125 = 0x04 

        #gyro out data rate
        ODR_2000_BW_532 = 0x00 
        ODR_2000_BW_230 = 0x01 
        ODR_1000_BW_116 = 0x02 
        ODR_400_BW_47 = 0x03 
        ODR_200_BW_23 = 0x04 
        ODR_100_BW_12 = 0x05 
        ODR_200_BW_64 = 0x06 
        ODR_100_BW_32 = 0x07 

        #gyro power state
        GYRO_NORMAL = 0x00 
        GYRO_SUSPEND = 0x80 
        GYRO_DEEP_SUSPEND = 0x20 


        #accelerometer active or not
        ACC_ACTIVE = 0x00
        ACC_SUSPEND = 0x03 

    ADDR_IDX_ACCEL=0
    ADDR_IDX_GYRO=1

    def __init__(self):
        self.initialised=False


    def _startup(self,):
        self.start_i2c(big_endian=False)
        
        self.set_accel_scale_range(BMI088.Regs.RANGE_6G)
        self.set_accel_output_data_rate(BMI088.Regs.ODR_100)
        self.set_accel_power_mode(BMI088.Regs.ACC_ACTIVE)
        
        self.set_gyro_scale_range(BMI088.Regs.RANGE_1000)
        self.set_gyro_output_data_rate(BMI088.Regs.ODR_400_BW_47)
        self.set_gyro_power_mode(BMI088.Regs.GYRO_NORMAL)
        self.initialised=True

    def set_accel_power_mode(self,mode):
        if mode == BMI088.Regs.ACC_ACTIVE:
            self.write( BMI088.Regs.BMI088_ACC_PWR_CTRl, 0x04,address_index=BMI088.ADDR_IDX_ACCEL)
            self.write( BMI088.Regs.BMI088_ACC_PWR_CONF, 0x00,address_index=BMI088.ADDR_IDX_ACCEL)
        elif mode == BMI088.Regs.ACC_SUSPEND:
            self.write( BMI088.Regs.BMI088_ACC_PWR_CONF, 0x03,address_index=BMI088.ADDR_IDX_ACCEL)
            self.write( BMI088.Regs.BMI088_ACC_PWR_CTRl, 0x00,address_index=BMI088.ADDR_IDX_ACCEL)

    def set_gyro_power_mode(self,mode):
        if mode == BMI088.Regs.GYRO_NORMAL:
            self.write(BMI088.Regs.BMI088_GYRO_LPM_1, BMI088.Regs.GYRO_NORMAL,address_index=BMI088.ADDR_IDX_GYRO)
        elif mode == BMI088.Regs.GYRO_SUSPEND:
            self.write( BMI088.Regs.BMI088_GYRO_LPM_1, BMI088.Regs.GYRO_SUSPEND,address_index=BMI088.ADDR_IDX_GYRO)
        elif mode == BMI088.Regs.GYRO_DEEP_SUSPEND:
            self.write(BMI088.Regs.BMI088_GYRO_LPM_1, BMI088.Regs.GYRO_DEEP_SUSPEND,address_index=BMI088.ADDR_IDX_GYRO)

    def set_accel_scale_range(self,range):
        if range == BMI088.Regs.RANGE_3G:
            self._accRange = 3.0
        elif range == BMI088.Regs.RANGE_6G:
            self._accRange = 6.0
        elif range == BMI088.Regs.RANGE_12G:
            self._accRange = 12.0
        elif range == BMI088.Regs.RANGE_24G:
            self._accRange = 24.0
        self.write(BMI088.Regs.BMI088_ACC_RANGE, range,address_index=BMI088.ADDR_IDX_ACCEL)

    def set_accel_output_data_rate(self,odr):    
        data = self.read(BMI088.Regs.BMI088_ACC_CONF,address_index=BMI088.ADDR_IDX_ACCEL);
        data = data & 0xf0;
        data = data | odr;
        
        self.write(BMI088.Regs.BMI088_ACC_CONF, data,address_index=BMI088.ADDR_IDX_ACCEL)

    def set_gyro_scale_range(self,range):
        if range == BMI088.Regs.RANGE_2000:
            self._gyroRange = 2000
        elif range == BMI088.Regs.RANGE_1000:
            self._gyroRange = 1000
        elif range == BMI088.Regs.RANGE_500:
            self._gyroRange = 500
        elif range == BMI088.Regs.RANGE_250:
            self._gyroRange = 250
        elif range == BMI088.Regs.RANGE_125:
            self._gyroRange = 125
        self.write(BMI088.Regs.BMI088_GYRO_RANGE, range,address_index=BMI088.ADDR_IDX_GYRO)

    def set_gyro_output_data_rate(self,odr):
        self.write(BMI088.Regs.BMI088_GYRO_BAND_WIDTH, odr,address_index=BMI088.ADDR_IDX_GYRO)
    
    def get_accel(self):
        """Get accelerometer values (in multiples of g)        
        """
        if not self.initialised:
            self._startup()
        return self.read_16_bit_values(BMI088.Regs.BMI088_ACC_X_LSB,3,multiplier=self._accRange / 32768,address_index=BMI088.ADDR_IDX_ACCEL)

    def get_gyro(self):
        """Get gyro values (in degrees per second)        
        """
        if not self.initialised:
            self._startup()
        return self.read_16_bit_values(BMI088.Regs.BMI088_GYRO_RATE_X_LSB,3,multiplier=self._gyroRange / 32768,address_index=BMI088.ADDR_IDX_GYRO)

if __name__=="__main__":
    b=BMI088()
    while True:
        print("%8.3f,%8.3f,%8.3f"%b.get_gyro())
        print("%8.3f,%8.3f,%8.3f"%b.get_accel())
        time.sleep(0.01)
else:
    IMUBase.register_sensor_type(BMI088)


