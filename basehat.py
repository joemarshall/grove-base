# support for the same grovepi interface on the grove HAT
# (updated seed studio shield for PI + pi zero)

import RPi.GPIO

DIGITAL_PINS={5,16,18,22,24,26}
DIGITAL_PIN_DIRECTIONS={}
ANALOG_PINS={0,2,4,6}

import smbus2 as smbus

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
        shield_type = self.bus.read_word_data(0x04, 0)
    except IOError:
        shield_type=0
    if shield_type not in [SHIELD_TYPE_PIZERO_HAT,SHIELD_TYPE_PI_HAT]:
        try:
            shield_type = self.bus.read_word_data(0x08,0)
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
    self.bus.read_word_data(ADC_ADDRESS,0x10+pin)

# digital read via raspberry pi GPIO
def digitalRead(pin):
    if pin not in DIGITAL_PINS:
        raise BadPinException(f"Grove base doesn't support digital read on pin {pin}")
    pinMode(pin,"INPUT")

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
        RPi.GPIO.setup(self.pin, newMode)

# ultrasonic read via raspberry pi GPIO
def ultrasonicRead(pin):
    if pin not in DIGITAL_PINS:
        raise BadPinException(f"Grove base doesn't support digital pin {pin}")

# ultrasonic read via raspberry pi GPIO
def ultrasonicReadBegin(pin):
    if pin not in DIGITAL_PINS:
        raise BadPinException(f"Grove base doesn't support digital pin {pin}")
    raise RuntimeError("Ultrasonic read not implemented yet")

def ultrasonicReadFinish(pin):
    if pin not in DIGITAL_PINS:
        raise BadPinException(f"Grove base doesn't support digital pin {pin}")

def dht(pin,version):
    if pin not in DIGITAL_PINS:
        raise BadPinException(f"Grove base doesn't support DHT on pin {pin}")
    raise RuntimeError("DHT not implemented yet")

def version():
    return "1.4.4"

if __name__=="__main__":
    import time
    print(f"Board type: {SHIELD_TYPE} ADC:{ADC_ADDRESS}")
    while True:
        time.sleep(1)
        print("DR16:",digitalRead(16))
        print("AR0:",analogRead(0))
