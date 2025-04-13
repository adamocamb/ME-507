"""
@author: Adam O'Camb and Jake Chess

Class to control our motor using PWM. It uses the X-Nucleo-IHM04A1 motor driver
for the nucleo board. Our motor was plugged into port A. This sets up PWM for
Motor A, then sends a percentage of full speed to each motor terminal.
"""

import pyb

class motor:
    #Initialize PWM for motor A on the X-Nucleo-IHM04A1 driver
    def __init__(self):
        #Enables Motor A by pulling up CPU pin A10. If using motor B, pull up
        #CPU pin C1 instead
        pyb.Pin('A10', pyb.Pin.OUT_PP, pyb.Pin.PULL_UP).high()
        #Defines CPU pin B4 as b4
        b4 = pyb.Pin('B4')
        #Defines CPU pin B5 as b5
        b5 = pyb.Pin('B5')
        #Sets TIM3 frequency on B4
        tim_b4 = pyb.Timer(3, freq=1000)
        #Sets TIM3 frequency on B5
        tim_b5 = pyb.Timer(3, freq=1000)
        #Sends PWM to B4, uses TIM3_CH1 
        ch_b4 = tim_b4.channel(1, pyb.Timer.PWM, pin=b4)
        #Sends PWM to B5, uses TIM3_CH2
        ch_b5 = tim_b5.channel(2, pyb.Timer.PWM, pin=b5)
        
        self.ch_b4 = ch_b4
        self.ch_b5 = ch_b5
    #Sends PWM signal to motor terminals as a percentage of max speed. Input
    #desired motor speed percentage.
    def motor_speed(self, speed):
        #Sets max speed percentage as 0% to B4
        self.ch_b4.pulse_width_percent(0)
        #Sets desired motor speed percentage to B5
        self.ch_b5.pulse_width_percent(speed)