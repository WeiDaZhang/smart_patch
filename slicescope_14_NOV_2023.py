#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed August 9 12:18:00 2023

@author: Dr. Vincent Lee, Dr. WeiDa Zhang
Bioptix Ltd.

"""

import serial 
import re

class Slicescope:

    # Class parameter

    # Instance method
    # Initialize instance with 'serial variable' in order to use serial.Serial functions
    def __init__(self,com_port):
        self.com_port = com_port
        self.ser = serial.Serial(port=self.com_port, baudrate=38400, bytesize=8, parity='N', stopbits=serial.STOPBITS_ONE, timeout=2)
        self.x = []
        self.y = []
        self.z = []
        self.x_origin = []
        self.y_origin = []
        self.z_origin = []
        self.x_max = []
        self.x_min = []
        self.y_max = []
        self.y_min = []
        self.z_max = []
        self.z_min = []
        self.x_delta = []
        self.y_delta = []
        self.z_delta = []

    def description(self):
        return "Slicescope"

    def coordinates(self):

        #Get xyz coordinates
        self.ser.write(b'P ? \r')

        byte_to_string = self.ser.readline().decode('utf-8')
        
        coords = [ int(s) for s in re.findall( r'[-+]?\d+' , byte_to_string) ]
    
        x_query = coords[0]
        y_query = coords[1]
        z_query = coords[2]
    
        return x_query,y_query,z_query 

    def scale(self):

        #X axis scale
        self.ser.write(b'SCALE X \r')
        x_scale = self.ser.readline().decode('utf-8')

        #Y axis scale
        self.ser.write(b'SCALE Y \r')
        y_scale = self.ser.readline().decode('utf-8')

        #Z axis scale
        self.ser.write(b'SCALE Z \r')
        z_scale = self.ser.readline().decode('utf-8')

        return x_scale,y_scale,z_scale

    def calibration(self):

        x_step = 100000_00
        y_step = 100000_00
        z_step = 100000_00

        #Slicescope
        # X axis calibration
        self.x,self.y,self.z = self.moveRelativeNoLimit(self.x,self.y,self.z,x_step,0,0)
        self.x_max = self.x
        self.x,self.y,self.z = self.moveRelativeNoLimit(self.x,self.y,self.z,-x_step,0,0)
        self.x_min = self.x

        self.x_delta = int(abs((self.x_max - self.x_min)/2))
        
        #Move slicescope back to center
        self.x,self.y,self.z = self.moveRelativeNoLimit(self.x,self.y,self.z,self.x_delta,0,0)
        self.x_origin = self.x

        # Y axis calibration
        self.x,self.y,self.z = self.moveRelativeNoLimit(self.x,self.y,self.z,0,y_step,0)
        self.y_max = self.y
        self.x,self.y,self.z = self.moveRelativeNoLimit(self.x,self.y,self.z,0,-y_step,0)
        self.y_min = self.y

        self.y_delta = int(abs((self.y_max - self.y_min)/2))

        #Move slicescope back to center
        self.x,self.y,self.z = self.moveRelativeNoLimit(self.x,self.y,self.z,0,self.y_delta,0)
        self.y_origin = self.y

        # Z axis calibration
        self.x,self.y,self.z = self.moveRelativeNoLimit(self.x,self.y,self.z,0,0,z_step)
        self.z_max = self.z
        self.x,self.y,self.z = self.moveRelativeNoLimit(self.x,self.y,self.z,0,0,-z_step)
        self.z_min = self.z

        self.z_delta = int(abs((self.z_max - self.z_min)/2))

        #Move slicescope back to center
        self.x,self.y,self.z = self.moveRelativeNoLimit(self.x,self.y,self.z,0,0,self.z_delta)
        self.z_origin = self.z

    def moveAbsolute(self,abs_x,abs_y,abs_z):   

    ###### Note: Be sure to check limit when using 40x objective. This must be -4000.00 minimum.

    #Move to absolute position (x,y,z coordinates)
    #Set xyz coord ABSXYZ: ABS # # #
    #Set xyz coord ABSXY: ABS # #
    #Set a coord ABSZ x: ABSZ #

    #Once the center of the coordinate system is established, only use ABS to move to specified point.
    #Know the maximum limits of coordinate system and put safeguards on this limit.
    #X,Y Limits +-25,000.00
    #Z Limits +- 12,500.00
    #Values are in hundredths of microns so 100.00 um is value = 10000

    #Limits of [X Y Z] = +-[25e5 25e5 12.5e5] 

        if abs_x > self.x_max:
            local_x = self.x_max
        
        elif abs_x < self.x_min:
            local_x = self.x_min

        else: 
            local_x = abs_x

        if local_x > 0:
            self.x_delta = int(abs(self.x_max - local_x))
        elif local_x < 0:
            self.x_delta = int(abs(self.x_min - local_x))
        else:
            self.x_delta = int(abs((self.x_max - self.x_min)/2))

        if abs_y > self.y_max:
            local_y = self.y_max
        
        elif abs_y < self.y_min:
            local_y = self.y_min

        else: 
            local_y = abs_y

        if local_y > 0:
            self.y_delta = int(abs(self.y_max - local_y))
        elif local_y < 0:
            self.y_delta = int(abs(self.y_min - local_y))
        else:
            self.y_delta = int(abs((self.y_max - self.y_min)/2))

        if abs_z > self.z_max:
            local_z = self.z_max
        
        elif abs_z < self.z_min:
            local_z = self.z_min

        else: 
            local_z = abs_z

        if local_z > 0:
            self.z_delta = int(abs(self.z_max - local_z))
        elif local_z < 0:
            self.z_delta = int(abs(self.z_min - local_z))
        else:
            self.z_delta = int(abs((self.z_max - self.z_min)/2))

        command = f"ABS {local_x} {local_y} {local_z}\r" #x,y,z coordinates 
        
        self.ser.write(command.encode('utf-8'))

        print(f"Moved Slicescope to ABS coords (X Y Z) = {local_x} {local_y} {local_z}")

        self.wait()
        
        return local_x,local_y,local_z

    def moveRelativeNoLimit(self,current_x,current_y,current_z,rel_x,rel_y,rel_z):
    #Move to NEW COORDINATES by new values relative to current coordinates

        command = f"REL {rel_x} {rel_y} {rel_z}\r"
        
        self.ser.write(command.encode('utf-8'))

        print(f"Moved Slicescope by REL coords = {rel_x} {rel_y} {rel_z}")
 
        self.wait()

        new_x,new_y,new_z = self.coordinates()

        print(f"New Slicescope coordinates = {new_x} {new_y} {new_z}")

        return new_x,new_y,new_z

    def moveRelative(self,current_x,current_y,current_z,rel_x,rel_y,rel_z):

    ###### Note: Be sure to check limit when using 40x objective. This must be -4000.00 minimum.
    #Make sure to pass in global slicescope_x,y,z coordinates to this function and return new local_x,y,z values for assignment as slicescope_x,y,z

    #Move to relative position (x,y,z coordinates)
    #Set xyz coord RELXYZ: REL # # #
    #Set xyz coord RELXY: REL # #
    #Set a coord RELZ x: RELZ #

    #Limits of [X Y Z] = +-[25e5 25e5 12.5e5]  overall. Need to know current position to calculate margin
    #for relative coordinate

    #Set a coord RELX x: RELX #
    #Need to know current position to calculate margin for relative coordinate

        new_x = current_x + rel_x    
    
        if new_x > self.x_max:
        
            new_rel_x = int(self.x_max - current_x)
            local_x = self.x_max

        elif new_x < self.x_min:
        
            new_rel_x = int(self.x_min - current_x)
            local_x = self.x_min

        else:
        
            new_rel_x = rel_x
            local_x = new_x

        print(f"Slicescope X coordinate is now = {local_x}")

        if local_x > 0:
            self.x_delta = int(abs(self.x_max - local_x))
        elif local_x < 0:
            self.x_delta = int(abs(self.x_min - local_x))
        else:
            self.x_delta = int(abs((self.x_max - self.x_min)/2))

    #Set a coord RELY x: RELY #

        new_y = current_y + rel_y   

        if new_y > self.y_max:
        
            new_rel_y = int(self.y_max - current_y)
            local_y = self.y_max

        elif new_y < self.y_min:
        
            new_rel_y = int(self.y_min - current_y)
            local_y = self.y_min

        else:
        
            new_rel_y = rel_y
            local_y = new_y

        print(f"Slicescope Y coordinate is now = {local_y}")

        if local_y > 0:
            self.y_delta = int(abs(self.y_max - local_y))
        elif local_y < 0:
            self.y_delta = int(abs(self.y_min - local_y))
        else:
            self.y_delta = int(abs((self.y_max - self.y_min)/2))

    #Set a coord RELZ x: RELZ #

        new_z = current_z + rel_z        

        if new_z > self.z_max:
        
            new_rel_z = int(self.z_max - current_z)
            local_z = self.z_max

        elif new_z < self.z_min:
        
            new_rel_z = int(self.z_min - current_z)
            local_z = self.z_min

        else:
        
            new_rel_z = rel_z
            local_z = new_z

        print(f"Slicescope Z coordinate is now = {local_z}")

        if local_z > 0:
            self.z_delta = int(abs(self.z_max - local_z))
        elif local_z < 0:
            self.z_delta = int(abs(self.z_min - local_z))
        else:
            self.z_delta = int(abs((self.z_max - self.z_min)/2))

    #Move to NEW COORDINATES by new values relative to current coordinates

        command = f"REL {new_rel_x} {new_rel_y} {new_rel_z}\r"
        
        self.ser.write(command.encode('utf-8'))
   
        print(f"Moved Slicescope by REL coords = {new_rel_x} {new_rel_y} {new_rel_z}")
        print(f"New Slicescope coordinates = {local_x} {local_y} {local_z}")

        self.wait()

        return local_x,local_y,local_z

    def moveUp(self,current_z,rel_z):

        ###### Note: Be sure to check limit when using 40x objective. This must be -4000.00 minimum.

        #Max is 12470.69 or 1.25e6
        #Min is -12470.00 or -1.25e6

        new_z = current_z + rel_z

        if rel_z > 0:
        
            if new_z > self.z_max:

                new_rel_z = int(self.z_max - current_z)
                local_z = self.z_max

            else:
            
                new_rel_z = rel_z
                local_z = new_z

            self.z_delta = int(abs(self.z_max - local_z))

            #Create string with command, then convert to byte to write to serial.
        
            command = f"OBJLIFT {new_rel_z}\r"  #Values in OBJLIFT can be negative.
        
            self.ser.write(command.encode('utf-8'))
        
            self.ser.write(b'OBJU\r')  #Moves like RELZ. 

            print(self.ser.readline().decode('utf-8'))
        
            print(f"Slicescope Z coordinate is now = {local_z}")

            self.wait()

        return local_z

    def stepOut(self,slicescope_z,step_out):

    ###### Note: Be sure to check limit when using 40x objective. This must be -4000.00 minimum.

    #Objective step in or out. Step in closer to sample. Step out away from sample.
    #Pass in global slicescope_z coordinate to this function and return new local_z value for assignment

        #Absolute value of step value
        step_out = abs(step_out)

        new_z = slicescope_z + step_out

        if new_z > self.z_max:

            new_rel_z = int(self.z_max - slicescope_z)
            local_z = self.z_max

        else:

            new_rel_z = step_out
            local_z = new_z

        self.z_delta = int(abs(self.z_max - local_z))

        command = f"SETSTEP {new_rel_z}\r"  #Set step value.

        self.ser.write(command.encode('utf-8'))

        self.ser.write(b'STEP\r')    #Step out. Moves up in Z direction by SETSTEP value
        print(f"Moved Slicescope OUT by STEP = {new_rel_z}")

        print(f"Slicescope Z coordinate is now = {local_z}")

        self.wait()

        return local_z

    def stepIn(self,slicescope_z,step_in):

    ###### Note: Be sure to check limit when using 40x objective. This must be -4000.00 minimum.

    #Objective step in or out. Step in closer to sample. Step out away from sample.
    #Pass in global slicescope_z coordinate to this function and return new local_z value for assignment

        #Absolute value of step value
        step_in = abs(step_in)

        new_z = slicescope_z - step_in

        if new_z < self.z_min:

            new_rel_z = int(slicescope_z - self.z_min)
            local_z = self.z_min

        else:

            new_rel_z = step_in
            local_z = new_z

        self.z_delta = int(abs(local_z - self.z_min))

        command = f"SETSTEP {new_rel_z}\r"  #Set step value.

        self.ser.write(command.encode('utf-8'))

        self.ser.write(b'STEP B\r')   #Step in. Moves down in Z direction by SETSTEP value
        print(f"Moved Slicescope IN by STEP = {new_rel_z}")

        print(f"Slicescope Z coordinate is now = {local_z}")

        self.wait()

        return local_z

    def stop(self):
        #Stop object
        self.ser.write(b'STOP S\r')
        print(f"Stopped {self.ser} = {self.ser.readline().decode('utf-8')}")

    def wait(self):
        #Wait for command to finish executing and instance stopped moving
        #Status (objective is stationary=0 or moving=1 or 2)
        self.ser.write(b'S\r')
        byte_to_string = self.ser.readline().decode('utf-8')   #Byte converted to String converted to Integer. This step is necessary for while loop condition
        status = int(re.search( r'[-+]?\d+' , byte_to_string).group()) #Search for numbers and convert MatchObject (whatever integer matches) as String.

        while(status != 0):
            self.ser.write(b'S\r')        
            byte_to_string = self.ser.readline().decode('utf-8')
            status = int(re.search( r'[-+]?\d+' , byte_to_string).group())

            if (status == 0):
                break

    def close(self):
        #Close connection
        self.ser.close()
        print(f"Connected to {self.ser} = {self.ser.is_open}")
