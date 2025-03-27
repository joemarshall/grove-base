# support for the same grovepi interface on the grove HAT
# (updated seed studio shield for PI + pi zero)

import RPi.GPIO
import time
import threading

from concurrent.futures import ThreadPoolExecutor

RPi.GPIO.setwarnings(False)
RPi.GPIO.setmode(RPi.GPIO.BCM)

DIGITAL_PINS={5,16,18,22,24,26}
DIGITAL_PIN_DIRECTIONS={}
ANALOG_PINS={0,2,4,6}

import smbus2 as smbus
bus = smbus.SMBus(1)

class BadPinException(Exception):
    pass

SHIELD_TYPE_PIZERO_HAT=0x05
SHIELD_TYPE_PI_HAT=0x04

SHIELD_TYPE = 0
ADC_ADDRESS= 0x04

def _read_shield_type():
    global SHIELD_TYPE,ADC_ADDRESS
    shield_type=0
    try:
        shield_type = bus.read_word_data(0x04, 0)
    except IOError:
        shield_type=0
    if shield_type not in [SHIELD_TYPE_PIZERO_HAT,SHIELD_TYPE_PI_HAT]:
        try:
            shield_type = bus.read_word_data(0x08,0)
            if shield_type  in [SHIELD_TYPE_PIZERO_HAT,SHIELD_TYPE_PI_HAT]:
                ADC_ADDRESS=0x08
            else:
                shield_type=0
        except IOError:
            shield_type=0  
    SHIELD_TYPE=shield_type        
    return shield_type

_read_shield_type()

# analog read via the onboard ADC
def analogRead(pin):
    if pin not in ANALOG_PINS:
        raise BadPinException(f"Grove base doesn't support analog read on pin {pin}")
    return (bus.read_word_data(ADC_ADDRESS,0x10+pin))>>2

# digital read via raspberry pi GPIO
def digitalRead(pin):
    if pin not in DIGITAL_PINS:
        raise BadPinException(f"Grove base doesn't support digital read on pin {pin}")
    pinMode(pin,"INPUT")
    return RPi.GPIO.input(pin)

# digital write via raspberry pi GPIO
def digitalWrite(pin,output):
    if pin not in DIGITAL_PINS:
        raise BadPinException(f"Grove base doesn't support digital write on pin {pin}")
    pinMode(pin,"OUTPUT")
    RPi.GPIO.output(pin, output)


# set pin mode of digital pin only
def pinMode(pin,mode):
    if pin not in DIGITAL_PINS:
        raise BadPinException(f"Grove base doesn't support digital pin {pin}")
    newMode = RPi.GPIO.IN
    if mode.lower()=="OUTPUT":
        newMode=RPi.GPIO.OUT
    if pin not in DIGITAL_PIN_DIRECTIONS or DIGITAL_PIN_DIRECTIONS[pin]!=newMode:
        RPi.GPIO.setup(pin, newMode)

_ULTRASONIC_READS={}
_ULTRASONIC_FINISHED_EVENT = threading.Event()
_TIMEOUT1 = 1000
_TIMEOUT2 = 10000
_ULTRASONIC_TIMEOUT_NS_PER_COUNT=None

_ULTRASONIC_READ_EXECUTOR=ThreadPoolExecutor ()

# ultrasonic read via raspberry pi GPIO
def ultrasonicRead(pin):
    if pin not in DIGITAL_PINS:
        raise BadPinException(f"Grove base doesn't support digital pin {pin}")
    ultrasonicReadBegin(pin)
    return ultrasonicReadFinish(pin)

def _ultrasound_thread(pin):
    global _ULTRASONIC_TIMEOUT_NS_PER_COUNT
    t1=time.monotonic_ns()
    RPi.GPIO.setup(pin,RPi.GPIO.OUT)
    RPi.GPIO.output(pin,0)
    time.sleep(0.000002) #2us
    RPi.GPIO.output(pin,1)
    time.sleep(0.0001) #10us
    RPi.GPIO.output(pin,0)
    RPi.GPIO.setup(pin,RPi.GPIO.IN)   
    count1 = 0
    while count1 < _TIMEOUT1 and RPi.GPIO.input(pin)==0:
        count1 += 1
    if count1 >= _TIMEOUT1:
        return (None,1)
    t2=time.monotonic_ns()
    count2=0
    while count2 < _TIMEOUT2 and RPi.GPIO.input(pin)==1:
        count2 += 1
    if count2 >= _TIMEOUT2:
        return (None,2)
    t3=time.monotonic_ns()
    ns_per_count=((count1/(t2-t1)),(count2/(t3-t2)))
    if _ULTRASONIC_TIMEOUT_NS_PER_COUNT==None:
        _ULTRASONIC_TIMEOUT_NS_PER_COUNT=ns_per_count
    else:
        _ULTRASONIC_TIMEOUT_NS_PER_COUNT=(ns_per_count[0]*0.1 + _ULTRASONIC_TIMEOUT_NS_PER_COUNT[0]*0.9,ns_per_count[1]*0.1 + _ULTRASONIC_TIMEOUT_NS_PER_COUNT[1]*0.9)
    return (t2,t3)

def ultrasonicReadAdjustTimeouts(pin,distanceMax=150):
    global _TIMEOUT2
    if _ULTRASONIC_TIMEOUT_NS_PER_COUNT==None:
        for c in range(10):
            ultrasonicRead(pin)
    _TIMEOUT2 = distanceMax*29*2000*_ULTRASONIC_TIMEOUT_NS_PER_COUNT[1]

# ultrasonic read via raspberry pi GPIO
def ultrasonicReadBegin(pin):
    global _ULTRASONIC_READS,_ULTRASONIC_READ_EXECUTOR
    if pin not in DIGITAL_PINS:
        raise BadPinException(f"Grove base doesn't support digital pin {pin}")
    if pin in _ULTRASONIC_READS and not _ULTRASONIC_READS[pin].done():
        _ULTRASONIC_READS[pin].cancel()
    _ULTRASONIC_READS[pin]=_ULTRASONIC_READ_EXECUTOR.submit(_ultrasound_thread,pin)
    


def ultrasonicReadFinish(pin):
    global _ULTRASONIC_READS
    if pin not in DIGITAL_PINS:
        raise BadPinException(f"Grove base doesn't support digital pin {pin}")
    if pin not in _ULTRASONIC_READS:
        raise BadPinException(f"Ultrasonic read finish without begin")
    t1,t2=_ULTRASONIC_READS[pin].result()
    if t1==None:
        return 600
    else:
        dt = int((t2 - t1) )

        distance = int((t2 - t1)   / 29 // 2000)
        return distance
        

def dht(pin,version):
    if pin not in DIGITAL_PINS:
        raise BadPinException(f"Grove base doesn't support DHT on pin {pin}")
    raise RuntimeError("DHT not implemented yet")

def version():
    return "1.4.4"

if __name__=="__main__":
    import time
    print(f"Board type: {SHIELD_TYPE} ADC:{ADC_ADDRESS}")
    ot=0
    while True:
        t=time.monotonic()
#        time.sleep(1)
 #       print("DR16:",digitalRead(16))
 #       print("AR0:",analogRead(0))
        print("UR5:",ultrasonicRead(5),t-ot)
#        print(_ULTRASONIC_TIMEOUT_NS_PER_COUNT)
        ot=t
        ultrasonicReadAdjustTimeouts(5,150)
        time.sleep(.00001)
