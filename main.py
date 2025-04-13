"""
Created on Thu Dec  8 15:54:30 2016

@author: Adam O'Camb and Jake Chess

This program is made to control the position of a motor using feedback from a
LSM303DLHC accelerometer on an IMU. We used a Nucleo STM32F411RE board with a 
X-Nucleo-IHM04A1 motor driver. All the modules are included as classes in
this file because our Nucleo Board cannot store any files and can only run one
at a time using pyboard.py. Although we used a Nucleo Board, this program
should run on any board with an STM32F411 with minimal modification. 

Current status: Logic seems like it works, but 'correction' returns 0's. Also,
'angle' varies 0-90 tilted each way. When the motor driver is run by itself the
all the angles are 0-180. All classes do work separately.
"""

#------------------------------------------------------------------------------

#Import needed MicroPython modules
import pyb
import math

"""
@author: davek
https://forum.pololu.com/t/lsm303d-raspberry-pi-driver/7698

Ported to MicroPython and LSM303DLHC by Adam O'Camb and Jake Chess

Driver for the LSM303DLHC accelerometer. It's loosely based on davek's driver
for the LSM303D accelerometer, but modified for the LSM303DLHC using
MicroPython. Using an I2C bus, it configures the accelerometer, then reads
acceleration data and converts it to Euler angles.
"""

#Control register addresses -- from LSM303DLHC datasheet
CTRL_REG1_A = 0x20 #Initiates accelerometer, configures data rate and axes
CTRL_REG2_A = 0x21 #High pass filter configuration
CTRL_REG3_A = 0x22 #Interrupts
CTRL_REG4_A = 0x23 #Configure accelerometer data
CTRL_REG5_A = 0x24 #FIFO, latch interrupts, 4D detection
CTRL_REG6_A = 0x25 #Interrupts

#Registers holding twos-complemented MSB and LSB of accelerometer
#readings -- from LSM303DLHC datasheet
OUT_X_L_A = 0x28 # X-axis accelerometer reading, LSB and MSB
OUT_X_H_A = 0x29
OUT_Y_L_A = 0x2A # Y-axis accelerometer reading, LSB and MSB
OUT_Y_H_A = 0x2B
OUT_Z_L_A = 0x2C # Z-axis accelerometer reading, LSB and MSB
OUT_Z_H_A = 0x2D

class LSM303:
    #Initialize I2C, set bus address, and configure accelerometer settings
    def __init__(self):
        #Initialize I2C
        i2c = pyb.I2C(1, pyb.I2C.MASTER, baudrate=52000)
        #I2C bus address        
        ACCEL = 0x19

        #Turn on accelerometer, enable all axes, set 50 hz sampling
        i2c.mem_write(0b01000111, ACCEL, CTRL_REG1_A) 
        #Set +/- 2g full scale, high resolution output (16 bit)
        i2c.mem_write(0b00001000, ACCEL, CTRL_REG4_A) 
                                                    
        self.i2c = i2c
        self.ACCEL = ACCEL

    #Function to convert 2's compliment binary to integer, input binary number
    #and number of bits in number
    def twos_comp(val, bits):
        #if sign bit is set
        if (val & (1 << (bits - 1))) != 0: 
            #compute negative value
            val = val - (1 << bits)
        return val
                    
    #Read x acceleration, combine MSB and LSB, and convert to angle
    def x_accel(self):
        #Read MSB of x-axis
        temp_msb = int.from_bytes(self.i2c.mem_read(1, self.ACCEL, OUT_Y_H_A))
        #Read LSB of x-axis
        temp_lsb = int.from_bytes(self.i2c.mem_read(1, self.ACCEL, OUT_Y_L_A))
        #Combine LSB and MSB
        value = int((temp_msb << 8) | temp_lsb)
        #Convert 2's compliment binary to integer and store
        acc = LSM303.twos_comp(value, 16)
        
        return acc  
        
    #Read y acceleration, combine MSB and LSB, and convert to angle        
    def y_accel(self):
        #Read MSB of y-axis
        temp_msb = int.from_bytes(self.i2c.mem_read(1, self.ACCEL, OUT_X_H_A))
        #Read LSB of y-axis
        temp_lsb = int.from_bytes(self.i2c.mem_read(1, self.ACCEL, OUT_X_L_A))
        #Combine LSB and MSB       
        value = int((temp_msb << 8) | temp_lsb)
        #Convert 2's compliment binary to integer and store       
        acc = LSM303.twos_comp(value, 16)
        
        return acc
        
    #Read z acceleration, combine MSB and LSB, and convert to angle        
    def z_accel(self):
        #Read MSB of z-axis
        temp_msb = int.from_bytes(self.i2c.mem_read(1, self.ACCEL, OUT_Z_H_A))
        #Read LSB of z-axis        
        temp_lsb = int.from_bytes(self.i2c.mem_read(1, self.ACCEL, OUT_Z_L_A))
        #Combine LSB and MSB       
        value = int((temp_msb << 8) | temp_lsb)        
        #Convert 2's compliment binary to integer and store        
        acc = LSM303.twos_comp(value, 16)
        
        return acc
    
    #Function to convert raw accelerometer data to Euler angles    
    def get_angles(dummy=True):
        #Initialize LSM303 class
        lsm=LSM303()
        
        #Get acceleration raw data and convert to g's using conversion factor
        #from data sheet for +/- 2g full scale
        Rx=lsm.x_accel()/1000
        Ry=lsm.y_accel()/1000
        Rz=lsm.z_accel()/1000
        
        #Compute magnitude of acceleration vector
        R = math.sqrt(Rx**2+Ry**2+Rz**2)

        #Compute Euler angles using the acceleration vectors
        roll = math.degrees(math.acos(Rx/R))
        pitch = math.degrees(math.acos(Ry/R))
        yaw = math.degrees(math.acos(Rz/R))
        
        return ([roll, pitch, yaw])
        
#------------------------------------------------------------------------------

"""
@author: Adam O'Camb and Jake Chess

Class to control our motor using PWM. It uses the X-Nucleo-IHM04A1 motor driver
for the nucleo board. Our motor was plugged into port A. This sets up PWM for
Motor A, then sends a percentage of full speed to each motor terminal.
"""

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
        #Sets max speed percentage as 50% to B4
        self.ch_b4.pulse_width_percent(50)
        #Sets desired motor speed percentage to B5
        self.ch_b5.pulse_width_percent(speed)
        
#------------------------------------------------------------------------------

"""
@author: B3AU
https://github.com/B3AU/micropython/blob/master/PID.py

Modified by Adam O'Camb and Jake Chess

PID class that takes an Euler angles from the accelerometer and outputs a motor
speed as percentage of max speed. Input the current angle of the IMU as
feedback.
"""
        
class PID:
    #Get input angle, set PID gains and other initial values
    def __init__(self,input_fun, P=3., I=0.01, D=0.0):
        #PID gains
        self.Kp=P
        self.Ki=I
        self.Kd=D
        #Set initial values of the P, I, and D parts to 0
        self.I_value = 0.0
        self.P_value = 0.0
        self.D_value = 0.0
        #Limit the integral controller value to +/- 100
        self.I_max=100.0
        self.I_min=-100.0
        #Set the set point to where the IMU is flat (90 degrees)
        self.set_point=90.0
        #Set initial values to 0
        self.prev_value = 0.0
        self.output = 0.0
        #Set to IMU output [deg]
        self.input = input_fun 
        #Set time between updates (pyb.millis() returns the number of 
        #milliseconds since the board was last reset)
        self.last_update_time = pyb.millis()

    #Function to do PID control for given reference input and feedback
    def update(self):
        #If max time since last update has been reached, run the update code
        if pyb.millis()-self.last_update_time > 500:
            #The current angle is the input angle
            current_value = self.input
            #Calculate error
            self.error = self.set_point - current_value
            #Print current angle
            print ('Position: '+str(current_value))
            
            #Calculate P and D controller values
            self.P_value = self.Kp * self.error
            self.D_value = self.Kd * (current_value-self.prev_value)
            self.prev_value = current_value

            #Calculate the time elapsed since the last update
            lapsed_time = pyb.millis()-self.last_update_time
            lapsed_time/=1000. #convert to seconds
            self.last_update_time = pyb.millis()

            #Calculate I controller value
            self.I_value += (self.error * self.Ki)*lapsed_time
            #Limit I value so it doesn't accumulate too much error
            if self.I_value > self.I_max:
                self.I_value = self.I_max
            elif self.I_value < self.I_min:
                self.I_value = self.I_min
            #Calculate PID output 
            self.output = self.P_value + self.I_value - self.D_value
            #Limit the PID output to make it easy to convert to max speed
            #percentage for PWM control
            if self.output<-370:
                self.output = 370.0
            if self.output>370:
                self.output = 370.0
            #Convert PID output to a percentage of max motor speed. An output
            #of 0 equates to 50%, while max forward speed is 100% and max
            #backward speed is 0%                
            self.output = self.output/7.4
            self.output = self.output + 50.0
            #Print relevant data
            #print("Setpoint: "+str(self.set_point))
            print("P: "+str(self.P_value))
            print("I: "+str(self.I_value))
            print("Output: "+str(self.output))
            
            #Update the time since last update
            self.last_update_time=pyb.millis()
            
#------------------------------------------------------------------------------            
"""
@author: Adam O'Camb and Jake Chess

This is our main code that calls the classes above. It gets the current angle
of the IMU and sends it to the PID. Currently it only gets the pitch angle, but
it can easily be changed to get other angles as well. After that it sends the
output from the PID to the motor control class.
"""

#Init accelerometer and motor classes
lsm = LSM303()
m=motor()

#Function that grabs a current pitch angle
def update_angle():
    #Gets all 3 angles from the accelerometer class
    angles = lsm.get_angles()
    #Gets pitch from the output of 3 angles
    pitch = angles[2]
    
    return pitch

#Loop to run the PID control and send motor outputs every 50 microseconds
while True:
    #Calls the PID class and sends it the update_angle function as the input angle
    #pid = PID(update_angle())
    #Runs the update function from PID class to calculate a new PID output
    angle = update_angle()    
    PID(angle).update()
    #Sets the PID output to variable speed_correction
    correction = PID(angle).output
    #Sends the new speed to the motor class
    m.motor_speed(correction)
    print(correction)
    print(angle)
    #Delays 50 microseconds before running the loop again
    pyb.delay(50)