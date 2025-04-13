"""
@author: davek
https://forum.pololu.com/t/lsm303d-raspberry-pi-driver/7698

Ported to MicroPython and LSM303DLHC by Adam O'Camb and Jake Chess

Driver for the LSM303DLHC accelerometer. It's loosely based on davek's driver
for the LSM303D accelerometer, but modified for the LSM303DLHC using
MicroPython. Using an I2C bus, it configures the accelerometer, then reads
acceleration data and converts it to Euler angles.
"""

#Import needed MicroPython modules
import pyb
import math

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