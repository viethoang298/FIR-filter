#import numpy as np
class FIR_filter:
    def __init__( self, coefficients):
        self.coeffFIR = coefficients
        self.nTaps = len(coefficients)
        self.ringbuffer = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.ringBufferOffset = 0

    def dofilter( self, inputValue ):
        # Store new value at the offset 
        self.ringbuffer[self.ringBufferOffset] = inputValue
        
        # Set offset variables
        offset = self.ringBufferOffset
        coeffOffset = 0
        
        # Initialize output to zero
        output = 0
        
        # Multiply values with coefficients until it reaches the beginning of the ring buffer
        while( offset >= 0 ):
            # Calculate tap value and add it to a sum
            output += self.ringbuffer[offset] * self.coeffFIR[coeffOffset] 
            # Move offsets 
            offset -= 1
            coeffOffset += 1
    
        # Set the offset to end of the array 
        offset = self.nTaps - 1
        
        # Multiply coefficients until it reaches the start of the ring buffer
        while( self.ringBufferOffset < offset ):
            # Calculate tap value and add it to a sum
            output += self.ringbuffer[offset] * self.coeffFIR[coeffOffset] 
            # Move offsets 
            offset -= 1
            coeffOffset += 1
           
        # Check if the next inputValue would be placed outside of the boundary of ring buffer and prevent this by wraping to the index of first element
        if( (offset + 1) >= self.nTaps ):
            self.ringBufferOffset = 0
        # The next offset value doesn't exceed the boundary
        else:
            self.ringBufferOffset += 1
            
        return output
    
    
    def ResetFilter( self ):
        # Reset the current offset and clear ringbuffer 
        self.ringBufferOffset = 0
        self.ringbuffer = np.zeros(self.nTaps)
        
#dac_ pcf8951
AOUTFLG = 0b01000000
AINPRG0 = 0b00000000
AINPRG1 = 0b00010000
AINPRG2 = 0b00100000
AINPRG3 = 0b00110000
AUTOINC = 0b00000100
ACHNNL0 = 0b00000000
ACHNNL1 = 0b00000001
ACHNNL2 = 0b00000010
ACHNNL3 = 0b00000011


class PCF8591:
    def __init__(self, i2c, addr=0x48, enable_out=True, in_program=AINPRG0):
        self.i2c = i2c
        self.addr = addr
        self._aout = self.set_out(enable_out)
        self._ainprg = self.set_program(in_program)
        self._last_ctl = self._make_control()

    def _make_control(self, auto_increment=False, channel=ACHNNL0):
        return 0 | self._aout | self._ainprg | (AUTOINC if auto_increment else 0) | channel

    def _write_control(self, control):
        if control != self._last_ctl:
            self.i2c.writeto(self.addr, bytes([control]))
            self.i2c.readfrom(self.addr, 1)
            self._last_ctl = control

    def _read_raw(self):
        return self.i2c.readfrom(self.addr, 4)

    def set_out(self, enable_out):
        self._aout = AOUTFLG if enable_out else 0
        return self._aout

    def set_program(self, in_program):
        self._ainprg = in_program
        return self._ainprg

    def read(self, channel=-1):
        if channel == -1:
            self.set_out(True)
            self._write_control(self._make_control(auto_increment=True))
            return self._read_raw()
        else:
            self._write_control(self._make_control(channel=channel))
            return int(self._read_raw()[0])

    def write(self, value):
        self.set_out(True)
        control = self._make_control()
        self._last_ctl = control
        self.i2c.writeto(self.addr, bytes([control, value]))
        
from machine import Pin
import utime
import ustruct
import sys

###############################################################################
# Settings
x = machine.Pin(23, Pin.OUT, value = 1)
# Initialize I2C with pins
i2c = machine.I2C(0,
                  scl=machine.Pin(9),
                  sda=machine.Pin(8),
                  freq=400000)

###############################################################################
'''


sampling frequency: 100 Hz

* 0 Hz - 10 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 2.3133909166191504 dB

* 20 Hz - 50 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -45.01618674525617 dB

'''
coeffs = [
  -0.01259277478717816,
  -0.02704833486706803,
  -0.031157016036431583,
  -0.003351666747179282,
  0.06651710329324828,
  0.1635643048779222,
  0.249729473226146,
  0.2842779082622769,
  0.249729473226146,
  0.1635643048779222,
  0.06651710329324828,
  -0.003351666747179282,
  -0.031157016036431583,
  -0.02704833486706803,
  -0.01259277478717816
]
length = len(coeffs)

# Initialise
fir = FIR_filter(coeffs)

adc_dac = PCF8591(i2c, 0x48 ,True, AINPRG0)

while True:
    vin = adc_dac.read(ACHNNL0) # channel 0
    output_fil = fir.dofilter(vin)
    output = round(output_fil)
    if output > 255:
        output = 255
    if output < 0:
        output = 0
    adc_dac.write(output)
    print( vin)
    utime.sleep(0.0000001)

    