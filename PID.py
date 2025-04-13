"""
@author: B3AU
https://github.com/B3AU/micropython/blob/master/PID.py

Modified by Adam O'Camb and Jake Chess

PID class that takes an Euler angles from the accelerometer and outputs a motor
speed as percentage of max speed. Input the current angle of the IMU as
feedback.
"""

import pyb
        
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
        self.input_fun = input_fun 
        #Set time between updates (pyb.millis() returns the number of 
        #milliseconds since the board was last reset)
        self.last_update_time = pyb.millis()

    #Function to do PID control for given reference input and feedback
    def update(self):
        #If max time since last update has been reached, run the update code
        if pyb.millis()-self.last_update_time > 500:
            #The current angle is the input angle
            current_value = self.input_fun()
            #Calculate error
            self.error = self.set_point - current_value
            #Print current angle and set point angle
            print ('Position '+str(current_value))
            print ('Set Point'+str(self.set_point))
            
            #Calculate P and D controller values
            self.P_value = self.Kp * self.error
            self.D_value = self.Kd * (current_value-self.prev_value)
            self.prev_value = self.current_value

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
            print("Setpoint: "+str(self.set_point))
            print("P: "+str(self.P_value))
            print("I: "+str(self.I_value))
            print("Output: "+str(self.output))
            
            #Update the time since last update
            self.last_update_time=pyb.millis()