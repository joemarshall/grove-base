# support for the same grovepi interface on the grove HAT
# (updated seed studio shield for PI + pi zero)

import time
import threading

from concurrent.futures import ThreadPoolExecutor
from gpiodirect import FastDHT11
import gpiod
from gpiod.line import Direction,Value,Bias,Edge

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
ADC_ADDRESS= 0x08

def _read_shield_type():
    global SHIELD_TYPE,ADC_ADDRESS
    shield_type=0
    try:
        shield_type = bus.read_word_data(0x08, 0)
    except IOError:
        shield_type=0
    if shield_type not in [SHIELD_TYPE_PIZERO_HAT,SHIELD_TYPE_PI_HAT]:
        try:
            shield_type = bus.read_word_data(0x04,0)
            if shield_type  in [SHIELD_TYPE_PIZERO_HAT,SHIELD_TYPE_PI_HAT]:
                ADC_ADDRESS=0x04
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
    with gpiod.request_lines("/dev/gpiochip4",
        config={
            pin: gpiod.LineSettings(
                direction=Direction.INPUT
            )
        },
    ) as req:
        return 1 if req.get_value(pin) == gpiod.line.Value.ACTIVE else 0

# digital write via raspberry pi GPIO
def digitalWrite(pin,output):
    if pin not in DIGITAL_PINS:
        raise BadPinException(f"Grove base doesn't support digital write on pin {pin}")
    with gpiod.request_lines("/dev/gpiochip4",
        config={
            pin: gpiod.LineSettings(
                direction=Direction.OUTPUT,
                output_value=Value.ACTIVE
            )
        },
    ) as req:
        pass

# set pin mode of digital pin only
def pinMode(pin,mode):
    if pin not in DIGITAL_PINS:
        raise BadPinException(f"Grove base doesn't support digital pin {pin}")
    if mode.lower()=="OUTPUT":
        digitalWrite(pin,0)
    else:
        digitalRead(pin)

_ULTRASOUND_MAX_TIME=0.2
_ULTRASOUND_MAX_DISTANCE=500
_ULTRASOUND_MAX_VALUE=500

# ultrasonic read via raspberry pi GPIO
def ultrasonicRead(pin,fp_distance=False):
    distance=_ULTRASOUND_MAX_VALUE
    if pin not in DIGITAL_PINS:
        raise BadPinException(f"Grove base doesn't support digital pin {pin}")
    chip = gpiod.Chip("/dev/gpiochip4")
    with chip.request_lines(
        config={
                pin: gpiod.LineSettings(
                    direction=Direction.OUTPUT, output_value=Value.INACTIVE
                )
            },
        event_buffer_size=4,
            consumer="us",
        ) as req:
            chip.watch_line_info(pin)
            time.sleep(0.000002) #2us
            req.set_value(pin,Value.ACTIVE)
            time.sleep(0.000005) #5us
#            req.set_value(pin,Value.INACTIVE)
            req.reconfigure_lines({pin:gpiod.LineSettings(
                    direction=Direction.INPUT,bias=Bias.PULL_DOWN,edge_detection=Edge.FALLING)})
            events=[]
            if req.wait_edge_events(_ULTRASOUND_MAX_TIME):
                events=req.read_edge_events()
                if chip.wait_info_event(0):
                    transition_time = chip.read_info_event().timestamp_ns
                    event_time = events[0].timestamp_ns
                    dt=(event_time-transition_time)
                    if fp_distance:
                        distance = (dt   / 29 / 2000)
                    else:
                        distance = int(dt   / 29 // 2000)

            chip.unwatch_line_info(pin)
    if distance>_ULTRASOUND_MAX_DISTANCE:
        distance=_ULTRASOUND_MAX_VALUE
    return distance
    
def ultrasonicReadSetMaxDistance(distanceMax=150,maxOutputValue=500):
    global _ULTRASOUND_MAX_TIME,_ULTRASOUND_MAX_DISTANCE,_ULTRASOUND_MAX_VALUE
    _ULTRASOUND_MAX_TIME=(1.5*distanceMax * 29 *2000)*1e-9
    _ULTRASOUND_MAX_DISTANCE=distanceMax
    _ULTRASOUND_MAX_VALUE=maxOutputValue

dht_handler = None

def dht(pin,version=12):
    global dht_handler
    if pin not in DIGITAL_PINS:
        raise BadPinException(f"Grove base doesn't support DHT on pin {pin}")
    if dht_handler==None:
        dht_handler=FastDHT11()
    return dht_handler.read(pin)

def version():
    return "1.4.4"

if __name__=="__main__":
    import time
    print(f"Board type: {SHIELD_TYPE} ADC:{ADC_ADDRESS}")
    ot=0
    ultrasonicReadSetMaxDistance(15)    
    while True:
#        time.sleep(1)
 #       print("DR16:",digitalRead(16))
 #       print("AR0:",analogRead(0))

        t=time.monotonic()
        print("UR16:",ultrasonicRead(16,fp_distance=True),time.monotonic()-t)
 #       print("DHT5",dht(5))
#        print(_ULTRASONIC_TIMEOUT_NS_PER_COUNT)
        ot=t
#        time.sleep(.5)
