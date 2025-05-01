import grovepi

def on(pin):
    """Turn an LED connected to digital pin number *pin* on"""
    grovepi.pin_mode(pin,"OUTPUT")
    grovepi.digital_write(pin,1)
    
def off(pin):
    """Turn an LED connected to digital pin number *pin* off"""
    grovepi.pin_mode(pin,"OUTPUT")
    grovepi.digital_write(pin,0)
