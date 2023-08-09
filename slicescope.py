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
        self.ser = serial.Serial(port=self.com_port, baudrate=9600, bytesize=8, parity='N', stopbits=serial.STOPBITS_ONE, timeout=2)
    
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

    def slicescope_scale(self):

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

        upper_limit_x = 1500000
        lower_limit_x = -1500000

        if abs_x > upper_limit_x:
            local_x = upper_limit_x
        
        elif abs_x < lower_limit_x:
            local_x = lower_limit_x

        else: 
            local_x = abs_x

        upper_limit_y = 1500000
        lower_limit_y = -1500000

        if abs_y > upper_limit_y:
            local_y = upper_limit_y
        
        elif abs_y < lower_limit_y:
            local_y = lower_limit_y

        else: 
            local_y = abs_y

        upper_limit_z = 1200000
        lower_limit_z = -1000000

        if abs_z > upper_limit_z:
            local_z = upper_limit_z
        
        elif abs_z < lower_limit_z:
            local_z = lower_limit_z

        else: 
            local_z = abs_z


        command = f"ABS {local_x} {local_y} {local_z}\r" #x,y,z coordinates 
        
        self.ser.write(command.encode('utf-8'))

        print(f"Moved Slicescope to ABS coords (X Y Z) = {local_x} {local_y} {local_z}")

        self.wait()
        
        return local_x,local_y,local_z

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

        upper_limit_x = 1500000
        lower_limit_x = -1500000

        new_x = current_x + rel_x    
    
        if new_x > upper_limit_x:
        
            new_rel_x = upper_limit_x - current_x
            local_x = upper_limit_x

        elif new_x < lower_limit_x:
        
            new_rel_x = lower_limit_x - current_x        
            local_x = lower_limit_x

        else:
        
            new_rel_x = rel_x
            local_x = new_x

        print(f"Slicescope X coordinate is now = {local_x}")


    #Set a coord RELY x: RELY #

        upper_limit_y = 1500000
        lower_limit_y = -1500000

        new_y = current_y + rel_y   

        if new_y > upper_limit_y:
        
            new_rel_y = upper_limit_y - current_y
            local_y = upper_limit_y

        elif new_y < lower_limit_y:
        
            new_rel_y = lower_limit_y - current_y        
            local_y = lower_limit_y

        else:
        
            new_rel_y = rel_y
            local_y = new_y

        print(f"Slicescope Y coordinate is now = {local_y}")


    #Set a coord RELZ x: RELZ #

        upper_limit_z = 1200000
        lower_limit_z = -1000000

        new_z = current_z + rel_z        

        if new_z > upper_limit_z:
        
            new_rel_z = upper_limit_z - current_z
            local_z = upper_limit_z

        elif new_z < lower_limit_z:
        
            new_rel_z = lower_limit_z - current_z        
            local_z = lower_limit_z

        else:
        
            new_rel_z = rel_z
            local_z = new_z

        print(f"Slicescope Z coordinate is now = {local_z}")


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
    
        upper_limit_z = 1200000

        new_z = current_z + rel_z

        if rel_z > 0:
        
            if new_z > upper_limit_z:

                new_rel_z = upper_limit_z - current_z
                local_z = upper_limit_z

            else:
            
                new_rel_z = rel_z
                local_z = new_z
        
            #Create string with command, then convert to byte to write to serial.
        
            command = f"OBJLIFT {new_rel_z}\r"  #Values in OBJLIFT can be negative.
        
            self.ser.write(command.encode('utf-8'))
        
            self.ser.write(b'OBJU\r')  #Moves like RELZ. 

            print(self.ser.readline().decode('utf-8'))
        
            print(f"Slicescope Z coordinate is now = {local_z}")

            self.wait()

        return local_z

    def stepOut(self,step_out):

    ###### Note: Be sure to check limit when using 40x objective. This must be -4000.00 minimum.

    #Objective step in or out. Step in closer to sample. Step out away from sample.
    #Pass in global slicescope_z coordinate to this function and return new local_z value for assignment

        command = f"SETSTEP {step_out}\r"  #Set step value.

        self.ser.write(command.encode('utf-8'))

        self.ser.write(b'STEP\r')    #Step out. Moves up in Z direction by SETSTEP value
        print(f"Moved Slicescope OUT by STEP = {step_out}")

        local_z = slicescope_z + step_out
    
        print(f"Slicescope Z coordinate is now = {local_z}")

        self.wait()

        return local_z

    def stepIn(self,step_in):

    ###### Note: Be sure to check limit when using 40x objective. This must be -4000.00 minimum.

    #Objective step in or out. Step in closer to sample. Step out away from sample.
    #Pass in global slicescope_z coordinate to this function and return new local_z value for assignment

        command = f"SETSTEP {step_in}\r"  #Set step value.

        self.ser.write(command.encode('utf-8'))

        self.ser.write(b'STEP B\r')   #Step in. Moves down in Z direction by SETSTEP value
        print(f"Moved Slicescope IN by STEP = {step_in}")

        local_z = slicescope_z - step_in
    
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
