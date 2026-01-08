"""This module allows you to get data from sensors, and also to replay sensor data from pre-recorded csv files."""

from math import sqrt
import io, csv
import grovepi
import imu
import smbus2 as smbus
from contextlib import contextmanager
import re
import time


def __getattr__(name):
    if not name in globals():
        print(
            f"Can't find sensors.{name}. Don't forget that you need to call sensors.set_pins before any sensor calls"
        )
    return globals()[name]


def set_pins(sensor_pin_mapping: dict):
    """Set the mapping between grove board pins and sensor objects. On the web sensor platform this does nothing.

    Call this with a mapping like:
    {
        "ultrasonic":3, # ultrasonic sensor on digital pin 3
        "gyro":0, # gyroscope on i2c port
        "sound":1 # sound sensor on analog pin 1
    }

    If you have two of any sensor, then you can add a number to the end of the name, like
    "ultrasonic1":4 # ultrasonic sensor 1 on digital pin 4

    Once this is called, sensor objects will be available as e.g. sensors.ultrasonic, sensors.gyro. For
    multiple of the same sensor, you can access them by the full name as e.g. sensors.ultrasonic1

    Args:
        sensor_pin_mapping (dict): Dict mapping from sensor name to pin. Sensor names are:
            "light,temperature_analog,sound,rotary_angle" - sensors connected to an analog pin
            "gyro,accel,magnetometer,nfc" - sensors connected to an i2c port
            "pir,button,touch,dht,ultrasonic" - sensors connected to a digital port.

    Raises:
        RuntimeError: Raised if an unknown sensor type is used.
    """
    _PIN_MAP = sensor_pin_mapping
    # pin indices is used to record all the sensor numbers given to things
    # this is so that if multiple different IMUs are attached,
    # we will use the first device for the lowest sensorNum, and the next
    # device for the following one.
    # This means that either {"accel0":0,"accel2":0} or
    # {"accel":0,"accel2":0} will both correctly
    # give the first two accelerometers attached
    pin_indices = {}
    for sensorFullname, pin in sensor_pin_mapping.items():
        sensorName = sensorFullname.lower()
        sensorName, sensorNum = re.match(r"(\D+)(\d*)", sensorName).groups()
        sensorNum = 0 if sensorNum == "" else int(sensorNum)
        if sensorName not in pin_indices:
            pin_indices[sensorName] = []
        pin_indices[sensorName].append(sensorNum)
        pin_indices[sensorName] = sorted(pin_indices[sensorName])

    for sensorFullname, pin in sensor_pin_mapping.items():
        sensorName = sensorFullname.lower()
        sensorName, sensorNum = re.match(r"(\D+)(\d*)", sensorName).groups()
        sensorNum = 0 if sensorNum == "" else int(sensorNum)
        new_sensor = None
        if (
            sensorName == "light"
            or sensorName == "temperature_analog"
            or sensorName == "sound"
            or sensorName == "rotary_angle"
        ):
            new_sensor = AnalogPinSensor(pin)
        elif sensorName == "gyro":
            # ignore the pin, must be an i2c port
            new_sensor = GyroSensor(pin_indices[sensorName].index(sensorNum))
        elif sensorName == "accel":
            # ignore the pin, i2c sensor
            new_sensor = AccelSensor(pin_indices[sensorName].index(sensorNum))
        elif sensorName == "magnetometer":
            # ignore the pin, i2c sensor
            new_sensor = MagnetometerSensor(pin_indices[sensorName].index(sensorNum))
        elif sensorName == "thermal":
            # ignore the pin, i2c sensor
            new_sensor = ThermalCameraSensor(pin_indices[sensorName].index(sensorNum))
        elif sensorName == "gesture":
            # ignore the pin, i2c sensor
            new_sensor = GestureSensor(pin_indices[sensorName].index(sensorNum))
        elif sensorName == "dht":
            new_sensor = DHTSensor(pin)
        elif (
            sensorName == "pir"
            or sensorName == "button"
            or sensorName == "touch"
            or sensorName == "tilt"
        ):
            new_sensor = DigitalPinSensor(pin)
        elif sensorName == "ultrasonic":
            new_sensor = UltrasonicSensor(pin)
        elif sensorName == "nfc":
            # NFC reader on i2c port
            new_sensor = NFCReader()
        else:
            raise RuntimeError("Unknown sensor type:", sensorName)
        if new_sensor != None:
            globals()[sensorFullname] = new_sensor


_LAST_SAMPLE_TIME = None
_SHOWN_DELAY_WARNING = False


def delay_sample_time(delay):
    """Sleep until *delay* seconds after the last time this was called.
    This allows you to steadily sample at a given rate even if sampling
    from your sensors takes some time.
    """
    global _LAST_SAMPLE_TIME, _SHOWN_DELAY_WARNING
    curtime = time.time()
    if _LAST_SAMPLE_TIME is not None:
        if _LAST_SAMPLE_TIME + delay > curtime:
            _LAST_SAMPLE_TIME += delay
            time.sleep(_LAST_SAMPLE_TIME - curtime)
            return
        else:
            if not _SHOWN_DELAY_WARNING:
                print(f"Warning, can't sample fast enough for delay {delay}")
                _SHOWN_DELAY_WARNING = True
    _LAST_SAMPLE_TIME = time.time()


def _does_i2c_device_exist(addr):
    retVal = False
    bus = smbus.SMBus(1)
    try:
        bus.read_byte(addr)
        retVal = True
    except IOError:
        retVal = False
    bus.close()
    return retVal


# mapping from sensor to pin
_SENSOR_PIN_MAP = {}


class NFCReader:
    def __init__(self):
        if not _does_i2c_device_exist(0x24):
            raise RuntimeError("Can't find NFC reader on I2C port")
        import pn532

        self.nfc = pn532.PN532()
        self.nfc.begin()
        self.nfc.SAM_configuration()

    def get_level(self):
        return self.nfc.read_passive_target(timeout_sec=0.0000000001)


class AnalogPinSensor:
    def __init__(self, pin):
        self.pin = pin

    def get_level(self):
        """return the level of this analog pin (from 0 to 1023)"""
        return grovepi.analogRead(self.pin)


class DigitalPinSensor:
    def __init__(self, pin):
        self.pin = pin

    def get_level(self):
        """return the level of this digital pin (0 or 1)"""
        return grovepi.digitalRead(self.pin)


class DHTSensor:
    def __init__(self, pin):
        self.pin = pin

    def get_level(self):
        """return the temperature (in C) and humidity as a tuple"""
        return grovepi.dht(self.pin, 0)


class ThermalCameraSensor:
    """Thermal camera sensor - currently only supports mlx90640 32x24 sensor"""

    def __init__(self, num):
        from i2csensors import mlx90640

        self.sensor = mlx90640.MLX90640()
        self.sensor.refresh_rate = mlx90640.RefreshRate.REFRESH_8_HZ
        self.frame_data = [0] * 768
        self.frame_reshaped = [[0] * 32 for x in range(24)]
        self.width = 32
        self.height = 24

    def get_temperature_array(self):
        self.sensor.get_frame(self.frame_data)
        for x in range(0, 24):
            self.frame_reshaped[x] = self.frame_data[32 * x : 32 * (x + 1)]
        return self.frame_reshaped


class GestureSensor:
    """Grove gesture sensor (paj7620) or similar. Detects gestures in front of it
    and/or cursor position of a waving hand"""

    def __init__(self, num):
        from i2csensors import paj7620
 
        self.sensor = paj7620.PAJ7620()
        GestureSensor.Gesture = paj7620.PAJ7620.Gesture

    def get_cursor_pos(self):
        """returns x,y, or None if no cursor"""
        if self.sensor.get_cursor_visible():
            return (self.sensor.get_cursor_x(), self.sensor.get_cursor_y())
        else:
            return None

    def get_detected_gesture(self):
        """return the detected gesture, or None if no gesture"""
        # n.b. gestures are defined in paj7620.py
        return self.sensor.get_gesture()

    def enable_gestures(self, enabled):
        """If gestures are turned off, we only report a cursor"""
        if enabled:
            self.sensor.gesture_mode()
        else:
            self.sensor.cursor_mode()


class UltrasonicSensor:
    """Ultrasonic sensor

    This sensor detects the distance to the nearest object in front of its beam. Because this is done
    with ultrasonic pulses, the time to get a sample from this sensor is equal to roughly the time sound
    takes to travel to and from the object. The maximum read distance is about 5 metres, which means that
    a call to get_level can take up to 50-100ms to run, during which your code stalls.

    If you want to dice with danger, this code also supports the begin_read and end_read methods. begin_read
    sends out a pulse on the sensor. end_read checks if the response has come back yet, and returns None if
    it hasn't, or the value otherwise. In between calls to begin_read and end_read, your code can do anything
    *except* read from other sensors connected to the digital or analog pins on the grovepi board. Note, this
    means it is okay to read from sensors connected to the i2c ports, such as the accelerometer, gyro,
    magnetometer boards etc.

    """

    def __init__(self, pin):
        self.pin = pin

    def get_level(self):
        """Return the sensed distance in centimetres"""
        return grovepi.ultrasonicRead(self.pin)

    def begin_read(self):
        """Begin an ultrasonic read. Until you call the matching end_read, *DO NOT* get values from
        any other sensors attached to the digital or analog pins of grovepi, or else *BAD THINGS* will
        happen. There is *NO SAFETY CODE* stopping you doing bad things here. You may read accelerometers,
        gyroscopes etc. to your hearts content though. This lets you combine ultrasonic pulses with high
        speed accelerometer readings."""
        grovepi.ultrasonicReadBegin(self.pin)

    def end_read(self):
        """Get the value of an ultrasonic read. Only call this after a begin_read call. If there is no
        response yet, it will return None, otherwise it returns the distance in centimetres where the ultrasonic
        pulse bounced off an object. This always returns immediately and does not delay.
        """
        value = grovepi.ultrasonicReadFinish(self.pin)
        if value >= 0:
            return value
        else:
            return None


class AccelSensor:
    """Accelerometer sensor

    This allows you to get the acceleration of a device in metres per second squared, along three axes, X, Y and Z, which for a phone
    are typically X,Y axes side to side and top to bottom on the screen, Z coming out of the screen. Be aware that in addition
    to any motion of the phone, the accelerometer will pick up a constant $9.8 \\frac{m/s}^2$ acceleration due to gravity.
    """

    def __init__(self, num):
        count = num if num is not None else 0
        for x in imu.IMUBase.scan_imus():
            if x.has_accelerometer():
                count -= 1
                if count < 0:
                    self.imu_class = x()
                    return
        raise IOError(f"Please connect accelerometer board {num+1}")

    def get_xyz(self):
        """Get the acceleration of the device

        This is returned in terms of x,y and z axes

        Returns
        -------
        x: float
            x axis acceleration in m/s^2
        y: float
            y axis acceleration in m/s^2
        z: float
            z axis acceleration in m/s^2
        """
        return self.imu_class.get_accel()

    def get_magnitude(self):
        """Get the magnitude of device acceleration.

        If the device is still, this will be 1G (about 9.8 m/s^2)

        Returns
        -------
        mag: float
            magnitude of device acceleration (i.e. sqrt(x^2+y^2+z^2))
        """
        x, y, z = self.get_xyz()
        return sqrt((x * x) + (y * y) + (z * z))


class MagnetometerSensor:
    """Magnetometer sensor

    This allows you to get the magnetic field affecting a device along three axes, X, Y and Z, which for a phone
    are typically X,Y axes side to side and top to bottom on the screen, Z coming out of the screen.
    """

    def __init__(self, num):
        count = num if num is not None else 0
        for x in imu.IMUBase.scan_imus():
            if x.has_magnetometer():
                count -= 1
                if count < 0:
                    self.imu_class = x()
                    return
        raise IOError(f"Please connect magnetometer board {num+1}")

    def get_xyz(self):
        """Get the magnetic field strength from the device

        This is returned in terms of x,y and z axes

        Returns
        -------
        x: float
            x axis magnetic field strength
        y: float
            y axis magnetic field strength
        z: float
            z axis magnetic field strength
        """
        return self.imu_class.get_magnetometer()

    def get_magnitude(self):
        """Get the magnitude of magnetic field strength

        Returns
        -------
        mag: float
            magnitude of device acceleration (i.e. sqrt(x^2+y^2+z^2))
        """
        x, y, z = self.get_xyz()
        return sqrt((x * x) + (y * y) + (z * z))


class GyroSensor:
    """Gyroscope sensor

    This allows you to get the rotation of a device in radians per second, around three axes, X, Y and Z, which for a phone
    are typically X,Y axes side to side and top to bottom on the screen, Z coming out of the screen.
    """

    def __init__(self, num):
        count = num if num is not None else 0
        for x in imu.IMUBase.scan_imus():
            if x.has_gyro():
                count -= 1
                if count < 0:
                    self.imu_class = x()
                    return
        raise IOError(f"Please connect gyro board {num+1}")

    def get_xyz(self):
        """Get the rotation of the device

        This is returned in terms of x,y and z axes

        Returns
        -------
        x: float
            x axis rotation in radians/s
        y: float
            y axis rotation in radians/s
        z: float
            z axis rotation in radians/s
        """
        return self.imu_class.get_gyro()

    def get_magnitude(self):
        """Get the magnitude of device rotation

        If the device is still, this will be 0

        Returns
        -------
        mag: float
            magnitude of device rotation (i.e. sqrt(x^2+y^2+z^2))
        """
        x, y, z = self.get_xyz()
        return sqrt((x * x) + (y * y) + (z * z))


class replayer:
    r"""Replay pre-recorded sensor data from CSV files

    This class supports loading of CSV files into your code and replaying them. The actual CSV loading logic is done for you
    when your script is started, you just need to check if there is any replay data and use it if so. For example you might
    do this with a conditional if statement like this:

    \`\`\`python
    if sensors.replayer.has_replay():
        this_time,x,y,z,sound = sensors.replayer.get_level("time","x","y","z","sound")
    else:
        this_time=time.time()-start_time
        x,y,z=sensors.accel.get_xyz()
        sound=sensors.sound.get_level()
    \`\`\`

    """

    _pos = 0
    _start_time = None
    _replay_lines = None
    _replay_columns = None
    _filename = None

    # do nothing context manager, which forces interrupts to stop
    @staticmethod
    @contextmanager
    def run_fast():
        try:
            yield 0
        finally:
            return

    @staticmethod
    def reset():
        """Restart the replay of data"""
        replayer._startTime = None
        replayer._pos = 0

    @staticmethod
    def columns():
        """Return the mapping of columns in the current CSV file

        Returns
        -------
        columns: map
            list of column:index pairs
        """
        return replayer._replay_columns

    # parse text csv string
    @staticmethod
    def _on_lines(lines, filename):
        def make_numbers(x):
            retval = []
            for y in x:
                try:
                    retval.append(float(y))
                except ValueError:
                    retval.append(y)
            return retval

        if not lines or len(lines) == 0:
            replayer._replay_lines = None
            replayer._replay_columns = None
            replayer._filename = None
            return
        replayer._filename = filename
        print("ON LINES:", replayer._filename)
        f = io.StringIO(lines)
        r = csv.reader(f)
        replayer._replay_columns = r.__next__()
        # make lookup for columns
        replayer._replay_columns = {
            str(x): y for y, x in enumerate(replayer._replay_columns)
        }
        # only get rows with the correct amount of data
        replayer._replay_lines = [
            make_numbers(x) for x in r if len(x) == len(replayer._replay_columns)
        ]

        replayer.reset()

    @staticmethod
    def init_replay(filename):
        """Load replay data from file. The replay data should be a CSV file which has a column for each sensor you are recording from."""
        with open(filename) as f:
            lines = f.read()
            replayer._on_lines(lines, filename)

    @staticmethod
    def get_replay_name():
        """Return the name of the currently loaded replay file
        This is useful for example if you want to do different
        tests for different types of input data
        """
        print("GET FNAME:", replayer._filename)
        return replayer._filename

    @staticmethod
    def has_replay():
        """Find out if there is replay data

        Returns True if there is a replay CSV file set up, false otherwise.

        Returns
        -------
        has_csv: bool
            True iff there is a replay CSV file.

        """
        return replayer._replay_lines != None

    @staticmethod
    def finished():
        """Has replay finished yet?

        Returns True if there are no more lines left in the CSV file

        Returns
        -------
        finished_csv: bool
            True iff the CSV file is finished

        """
        if not replayer._replay_lines:
            return True
        return replayer._pos >= len(replayer._replay_lines) - 1

    @staticmethod
    def get_level(*col_names):
        r"""Get a sample worth of sensor levels from the CSV file

        This returns selected columns from a line in the CSV file and then moves onto the next line. This means that
        if you want to read multiple columns, you have to do it in one call.

        Parameters
        ----------
        *col_names : tuple
            Pass the list of column names that you want to read, e.g.
            \`sensors.replayer.get_level("time","sound","light")\`

        Returns
        -------
        columns: tuple
            The value of each of the requested columns
        """
        if replayer._replay_lines and len(replayer._replay_lines) > replayer._pos:
            ret_val = replayer._replay_lines[replayer._pos]
            if replayer._pos < len(replayer._replay_lines) - 1:
                replayer._pos += 1
        else:
            ret_val = 0 * len(replayer._replay_columns)
        if col_names:
            # look up the columns and return in order
            return [ret_val[replayer._replay_columns[x]] for x in col_names]
        else:
            return ret_val


if __name__ == "__main__":
    import time

    set_pins({"ultrasonic": 4})
    while True:
        print(ultrasonic.get_level())
        time.sleep(0.0)
