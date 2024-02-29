import grovepi

def on(pin):
    """Turn an LED connected to digital pin number *pin* on"""
    grovepi.pinMode(pin,"OUTPUT")
    grovepi.digitalWrite(pin,1)
    
def off(pin):
    """Turn an LED connected to digital pin number *pin* off"""
    grovepi.pinMode(pin,"OUTPUT")
    grovepi.digitalWrite(pin,0)
