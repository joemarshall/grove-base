import smbus2 as smbus
from math import *
import struct

class IMUBase:
    imu_classes={}
    @staticmethod
    def register_sensor_type(sensor_class):
        if type(sensor_class.ADDRESS)==list:
            IMUBase.imu_classes[tuple(sensor_class.ADDRESS)]=sensor_class
        else:
            IMUBase.imu_classes[sensor_class.ADDRESS]=sensor_class

    @staticmethod
    def has_accelerometer():
        return False

    @staticmethod
    def has_magnetometer():
        return False

    @staticmethod
    def has_gyro():
        return False

    @staticmethod
    def scan_imus():
        imu_list=[]
        bus=smbus.SMBus(1)
        for addr,cls in IMUBase.imu_classes.items():
            try:
                found_device=False
                if hasattr(addr, '__iter__'):
                    for subaddr,(reg,value) in zip(cls.ADDRESS,cls.ID_REG_VALUE):
                        if bus.read_byte_data(subaddr,reg)==value:
                            found_device=True
                        else:
                            print(f"Wrong ID for {cls}")
                            continue
                else:
                    reg,value=cls.ID_REG_VALUE
                    if bus.read_byte_data(addr,reg)==value:
                        found_device=True
                    else:
                        print(f"Wrong ID for {cls}")
                if found_device:
                    imu_list.append(IMUBase.imu_classes[addr])
            except IOError:
                pass
        return imu_list
        
    def get_rotation_matrix(mag=None,accel=None):
        """ Returns a 3x3 matrix of how the device is rotated, based on magnetometer and accelerometer values

        Args:
            mag: Magnetometer values from getMag()
            accel: Accelerometer values from getAccel()
            
            If either argument is None, it will call getMag / getAccel
        
        Returns:
            3x3 tuple rotation matrix, put this into getOrientation(matrix) to get yaw, pitch, roll values
            or None if there isn't enough information (device is in freefall)
        """
        if self.has_magnetometer()==False or self.has_accelerometer()==False:
            raise RuntimeError("Can't get rotation matrix from device without magnetometer and gyroscope")
        if mag==None:
            mag=self.get_magnetometer()
        if accel==None:
            accel=self.get_accel()
        Ax = accel[0]
        Ay = accel[1]
        Az = accel[2]
        Ex = mag[0]
        Ey = mag[1]
        Ez = mag[2]
        Hx = Ey*Az - Ez*Ay
        Hy = Ez*Ax - Ex*Az
        Hz = Ex*Ay - Ey*Ax        
        normH = sqrt(Hx*Hx + Hy*Hy + Hz*Hz)
        if normH < 0.1:
            # in freefall or something
            return None
        invH = 1.0 / normH
        Hx *= invH
        Hy *= invH
        Hz *= invH
        invA = 1.0 / sqrt(Ax*Ax + Ay*Ay + Az*Az)
        Ax *= invA;
        Ay *= invA;
        Az *= invA;
        Mx = Ay*Hz - Az*Hy;
        My = Az*Hx - Ax*Hz;
        Mz = Ax*Hy - Ay*Hx;
        return ((Hx,Hy,Hz),(Mx,My,Mz),(Ax,Ay,Az))
        
    def get_orientation(self,matrix=None,errorValue=(0,0,0)):
        """ Get orientation values (Yaw, pitch, roll) from rotation matrix
        
        Args: 
            matrix: Rotation matrix, from getRotationMatrix(mag,accel)
                    if Matrix is None, then it will call the relevant getAccel functions itself
                    
            errorValue: If the rotation matrix can't be found (if it is in freefall, this value is returned. 
                        By default this is set to be just a zero value, if you want to distinguish error events
                        then set this to some other value (e.g. None)
            
        Returns:
            (yaw, pitch, roll) tuple
        """
        if matrix==None:
            matrix=self.get_rotation_matrix()
        if matrix==None:
            return errorValue        
        yaw=atan2(matrix[0][1], matrix[1][1])
        pitch=asin(-matrix[2][1])
        roll=atan2(-matrix[2][0], matrix[2][2])
        return yaw,pitch,roll
    
    def start_i2c(self,big_endian):
        # address can be either a single address or a list of them
        # for devices with multiple i2c addresses
        self.bus=smbus.SMBus(1)
        if type(self.ADDRESS)==int:
            self.address=[self.ADDRESS]
        else:
            self.address=self.ADDRESS
        self.big_endian=big_endian

    def write(self,register,data,*,address_index=0):
        self.bus.write_byte_data(self.address[address_index],register,data)
        
    def read(self,register,*,address_index=0):
        return self.bus.read_byte_data(self.address[address_index],register)
    
    def read_byte_values(self,register,count,*,address_index=0):
        return self.bus.read_i2c_block_data(self.address[address_index],register,count)
    
    def read_16_bit_values(self,register,count,multiplier,*,address_index=0):
#        raw_data=bytearray(self.bus.read_i2c_block_data(self.address[address_index],register,count*2))
        raw_data=bytearray([self.read(a,address_index=address_index) for a in range(register,register+count*2)])
        format_str="h"*count
        if self.big_endian:
            format_str=">"+format_str
        else:
            format_str="<"+format_str
        retval=struct.unpack(format_str,raw_data)
        if type(multiplier)==float or type(multiplier)==int:
            return tuple([x * multiplier for x in retval])
        elif hasattr(multiplier, '__iter__'):
            return tuple([x * mult  for x,mult in zip(retval,multiplier)])
        elif multiplier==None:
            return retval
        else:
            raise RuntimeError(f"Bad multiplier:{multiplier}")

    def read_12_bit_values(self,register,count,multiplier,*,address_index=0):
        raw_data=bytearray(self.bus.read_i2c_block_data(self.address[address_index],register,count*2))
        format_str="H"*count
        if self.big_endian:
            format_str=">"+format_str
        else:
            format_str="<"+format_str            
        retval=struct.unpack(format_str,raw_data)
        retval=[x if x<2048 else x-4096 for x in retval]
        if type(multiplier)==float or type(multiplier)==int:
            return tuple([x * multiplier for x in retval])
        elif hasattr(multiplier, '__iter__'):
            return tuple([x * mult  for x,mult in zip(retval,multiplier)])
        elif multiplier==None:
            return retval
        else:
            raise RuntimeError(f"Bad multiplier:{multiplier}")
            



