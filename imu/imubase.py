import smbus2 as smbus
from math import *

class IMUBase:
    imu_classes={}
    @staticmethod
    def register_sensor_type(i2c_address,sensor_class):
        IMUBase.imu_classes[i2c_address]=sensor_class

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
        for addr in IMUBase.imu_classes.keys():            
            try:
                bus.read_byte(addr)
                imu_list.append(IMUBase.imu_classes[addr])
            except IOError:
                pass
        
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
        
