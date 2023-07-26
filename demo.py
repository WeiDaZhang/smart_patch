#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon July 17 13:12:00 2023

@author: Dr. Vincent Lee, Dr. WeiDa Zhang
Bioptix Ltd.

"""

# Use # for single line comment
# Use ''' ''' or """ """ as bookends for multiple line comments. Must be indented!

import sys
import time
import serial 
import re

from tip_search_hist import hist_top_coord

import cv2 as cv
import numpy as np

from pyvcam import pvc
from pyvcam.camera import Camera
from pyvcam import constants

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


class Condenser:

    # Class parameter
    
    # Instance method
    # Initialize instance with 'serial variable' in order to use serial.Serial functions
    def __init__(self,com_port):
        self.com_port = com_port
        self.ser = serial.Serial(port='COM4', baudrate=9600, bytesize=8, parity='N', stopbits=serial.STOPBITS_ONE, timeout=2)

    def description(self):
        return "Condenser"

    def coordinates(self):
        #Get Z coordinate
        self.ser.write(b'PZ \r')

        byte_to_string = self.ser.readline().decode('utf-8')

        z_query = int(byte_to_string)

        return z_query

    def moveAbsolute(self,abs_z):

    #Move to absolute position in z coordinate
    #Set a coord ABSZ x: ABSZ #
    #Limits of Z = +-12.5e5 
    
        upper_limit_z = 1250000
        lower_limit_z = 1200000

        if abs_z > upper_limit_z:
            local_z = upper_limit_z
        
        elif abs_z < lower_limit_z:
            local_z = lower_limit_z
        
        else: 
            local_z = abs_z
    
        command = f"ABSZ {local_z}\r"  #z coordinates only. - is UP. + is DOWN.

        self.ser.write(command.encode('utf-8'))
    
        print(f"Moved Condenser to ABS coord = {local_z}")

        self.wait()

        return local_z


    def moveRelative(self,current_z,rel_z):
    #Move to relative position in z coordinate.
    #Make sure to pass in global condenser_z coordinate to this function and return new local_z value for assignment as condenser_z

        upper_limit_z = 1250000
        lower_limit_z = 1200000

        new_z = current_z + rel_z

    #Set a coord RELZ x: RELZ #
    #Need to know current position to calculate margin for relative coordinate

        if new_z > upper_limit_z:
        
            new_rel_z = upper_limit_z - current_z
            local_z = upper_limit_z

        elif new_z < lower_limit_z:
        
            new_rel_z = lower_limit_z - current_z        
            local_z = lower_limit_z

        else:
            new_rel_z = rel_z        
            local_z = new_z

        command = f"RELZ {new_rel_z}\r"

        self.ser.write(command.encode('utf-8'))

        print(f"Moved Condenser by REL coord = {new_rel_z}")
        print(f"Condenser coordinate is now = {local_z}")

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


class Patchstar:

    # Class parameter

    # Instance method
    # Initialize instance with 'serial variable' in order to use serial.Serial functions
    def __init__(self,com_port):
        self.com_port = com_port
        self.ser = serial.Serial(port=self.com_port, baudrate=38400, bytesize=8, parity='N', stopbits=serial.STOPBITS_ONE, timeout=2)
    
    def description(self):
        return "Micromanipulator"    

    def coordinates(self):

        #Get xyza coordinates
        self.ser.write(b'P ? \r')
    
        byte_to_string = self.ser.readline().decode('utf-8')

        coords = [ int(s) for s in re.findall( r'[-+]?\d+' , byte_to_string) ]

        x_query = coords[0]
        y_query = coords[1]
        z_query = coords[2]
        a_query = coords[3]

        return x_query,y_query,z_query,a_query 

    def probe_angle(self):

        self.ser.write(b'ANGLE \r')
    
        probe_angle = self.ser.readline().decode('utf-8')

        return probe_angle

    def probe_scale(self):

        #X axis scale
        self.ser.write(b'SCALE X \r')
        x_scale = self.ser.readline().decode('utf-8')

        #Y axis scale
        self.ser.write(b'SCALE Y \r')
        y_scale = self.ser.readline().decode('utf-8')

        #Z axis scale
        self.ser.write(b'SCALE Z \r')
        z_scale = self.ser.readline().decode('utf-8')

        #A axis scale
        self.ser.write(b'SCALE A \r')
        a_scale = self.ser.readline().decode('utf-8')

        return x_scale,y_scale,z_scale,a_scale

    def moveAbsolute(self,abs_x,abs_y,abs_z):  

    #Move to absolute position (x,y,z coordinates)
    #Set xyz coord ABSXYZ: ABS # # #
    #Set xyz coord ABSXY: ABS # #
    #Set a coord ABSZ x: ABSZ #

    #Once the center of the coordinate system is established, only use ABS to move to specified point.
    #Know the maximum limits of coordinate system and put safeguards on this limit.
    #X,Y,Z Limits +-10,000.00
    #Values are in hundredths of microns so 100.00 um is value = 10000

    #Limits of [X Y Z] = +-[10e5 10e5 10e5] 

        upper_limit_x = 1000000
        lower_limit_x = -1000000

        if abs_x > upper_limit_x:
            local_x = upper_limit_x
        
        elif abs_x < lower_limit_x:
            local_x = lower_limit_x

        else: 
            local_x = abs_x

        upper_limit_y = 1000000
        lower_limit_y = -1000000

        if abs_y > upper_limit_y:
            local_y = upper_limit_y
        
        elif abs_y < lower_limit_y:
            local_y = lower_limit_y

        else: 
            local_y = abs_y

        upper_limit_z = 1000000
        lower_limit_z = -1000000

        if abs_z > upper_limit_z:
            local_z = upper_limit_z
        
        elif abs_z < lower_limit_z:
            local_z = lower_limit_z

        else: 
            local_z = abs_z


        command = f"ABS {local_x} {local_y} {local_z}\r"  #x,y,z coordinates 
        
        self.ser.write(command.encode('utf-8'))

        print(f"Moved Micromanipulator to ABS coords = {local_x} {local_y} {local_z}")   #Read the message/acknowledgment from the command

        self.wait()

        return local_x,local_y,local_z

    def moveRelative(self,current_x,current_y,current_z,rel_x,rel_y,rel_z):

    #Make sure to pass in global patchstar_x,y,z coordinate to this function and return new local_x,y,z value for assignment as patchstar_x,y,z

    #Move to relative position (x,y,z coordinates)
    #Set xyz coord RELXYZ: REL # # #
    #Set xyz coord RELXY: REL # #
    #Set a coord RELZ x: RELZ #

    #Limits of [X Y Z] = +-[10e5 10e5 10e5]  overall. Need to know current position to calculate margin
    #for relative coordinate

    #Set a coord RELX x: RELX #
    #Need to know current position to calculate margin for relative coordinate

        upper_limit_x = 1000000
        lower_limit_x = -1000000

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

        print(f"Micromanipulator X coordinate is now = {local_x}")


    #Set a coord RELY x: RELY #

        upper_limit_y = 1000000
        lower_limit_y = -1000000

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

        print(f"Micromanipulator Y coordinate is now = {local_y}")


    #Set a coord RELZ x: RELZ #

        upper_limit_z = 1000000
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

        print(f"Micromanipulator Z coordinate is now = {local_z}")


    #Move to NEW COORDINATES by new values relative to current coordinates

        command = f"REL {new_rel_x} {new_rel_y} {new_rel_z}\r"
        
        self.ser.write(command.encode('utf-8'))

        print(f"Moved Micromanipulator by REL coords = {new_rel_x} {new_rel_y} {new_rel_z}")
        print(f"New Micromanipulator coordinates = {local_x} {local_y} {local_z}")

        self.wait()

        return local_x,local_y,local_z

    def approachAbsolute(self,abs_a): 

    #Move to absolute coordinate A=XZ
    #Set a coord ABSA x: ABSA #

    #Limit is +-

        upper_limit_a = 1000000
        lower_limit_a = -1000000

        if abs_a > upper_limit_a:
            local_a = upper_limit_a
        
        elif abs_a < lower_limit_a:
            local_a = lower_limit_a

        else: 
            local_a = abs_a


        command = f"ABSA {local_a}\r"  #A=XZ coordinates 

        self.ser.write(command.encode('utf-8'))

        print(f"Moved Probe to approach ABS = {local_a}")   #Read the message/acknowledgment from the command

        self.wait()

        return local_a

    def approachRelative(self,current_a,rel_a):  

    #Move to relative coordinate A=XZ
    #Set a coord RELA x: RELA #

    #Limit is +-12,000.00 or 12e5

        upper_limit_a = 1000000
        lower_limit_a = -1000000


        new_a = current_a + rel_a        

        if new_a > upper_limit_a:
        
            new_rel_a = upper_limit_a - current_a
            local_a = upper_limit_a

        elif new_a < lower_limit_a:
        
            new_rel_a = lower_limit_a - current_a        
            local_a = lower_limit_a

        else:
        
            new_rel_a = rel_a
            local_a = new_a

        command = f"RELA {new_rel_a}\r"  #A=XZ coordinates 

        self.ser.write(command.encode('utf-8'))

        print(f"Moved Probe to approach REL = {new_rel_a}")   #Read the message/acknowledgment from the command
        print(f"Coordinate A = {local_a}")   #Read the message/acknowledgment from the command

        self.wait()

        return local_a

    def moveUp(self,current_z,rel_z):

        #Max is 12470.69 or 1.25e6
        #Min is -12470.00 or -1.25e6

        upper_limit_z = 1000000

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

            print(f"Micromanipulator Z coordinate is now = {local_z}")

        self.wait()

        return local_z

    def stepOut(self,step_out):
    #Objective step in or out. Step in closer to sample. Step out away from sample.

        #Absolute value of step value
        step_out = abs(step_out)
    
        command = f"SETSTEP {step_out}\r"  #Set step value.

        self.ser.write(command.encode('utf-8'))

        self.ser.write(b'APPROACH\r')

        byte_to_string = self.ser.readline().decode('utf-8')
    
        status = int(re.search( r'[-+]?\d+' , byte_to_string).group()) #Search for numbers and convert MatchObject (whatever integer matches) as String.

        if (status == 1):
            #if APPROACH=1, then step can be used for A coordinate/direction
            self.ser.write(b'STEP\r')    #Step out. Moves up in A=XZ direction by SETSTEP value
            print(f"Moved Micromanipulator OUT by STEP = {step_out}")

            local_a = patchstar_a + step_out
    
            print(f"Micromanipulator A coordinate is now = {local_a}")

        else:

            #if APPROACH=0, then step can be used only for Z coordinate/direction 
            self.ser.write(b'STEP\r')    #Step out. Moves up in Z direction by SETSTEP value
            print(f"Moved Micromanipulator OUT by STEP = {step_out}")

            local_z = patchstar_z + step_out
    
            print(f"Micromanipulator Z coordinate is now = {local_z}")

        self.wait()

        return local_z, local_a

    def stepIn(self,step_in):
    #Objective step in or out. Step in closer to sample. Step out away from sample.

        #Absolute value of step value
        step_in = abs(step_in)
    
        command = f"SETSTEP {step_in}\r"  #Set step value.

        self.ser.write(command.encode('utf-8'))

        self.ser.write(b'APPROACH\r')

        byte_to_string = self.ser.readline().decode('utf-8')

        status = int(re.search( r'[-+]?\d+' , byte_to_string).group()) #Search for numbers and convert MatchObject (whatever integer matches) as String.

        if (status == 1):
            #if APPROACH=1, then step can be used for A coordinate/direction
            self.ser.write(b'STEP B\r')    #Step out. Moves up in A=XZ direction by SETSTEP value
            print(f"Moved Micromanipulator IN by STEP = {step_in}")

            local_a = patchstar_a - step_in
    
            print(f"Micromanipulator A coordinate is now = {local_a}")

        else:

            #if APPROACH=0, then step can be used only for Z coordinate/direction 
            self.ser.write(b'STEP B\r')    #Step out. Moves up in Z direction by SETSTEP value
            print(f"Moved Micromanipulator OUT by STEP = {step_in}")

            local_z = patchstar_z - step_in
    
            print(f"Micromanipulator Z coordinate is now = {local_z}")
        
        self.wait()

        return local_z, local_a

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


class PVCAM:

    # Class parameter

    # Instance method
    def __init__(self,slicescope_instance):
        #Connect to PVCAM
        pvc.init_pvcam()  # Initialize PVCAM 
    
        camera_names = Camera.get_available_camera_names()
        print(camera_names)
        #PMUSBCam00 is the only camera in this list
    
        #self.cam = next(Camera.detect_camera()) # Use generator to find first camera.
        self.cam = Camera.select_camera(camera_names[0])
        self.cam.open()  # Open the camera.

        self.slicescope_instance = slicescope_instance

    def description(self):
        return "Camera"

    def image(self):

        frame = self.cam.get_frame(exp_time=20)
        high = np.max(frame)
        frame_norm = np.uint8(frame/high * 255)
        #cv.imshow('Image', frame_norm)
        #print(f"row = {len(frame)}, col={len(frame[0])}")
        return frame_norm

    def image_stop(self):

        frame = self.cam.get_frame(exp_time=20)
        high = np.max(frame)
        frame_norm = np.uint8(frame/high * 255)

        cv.imshow('Image', frame_norm)
        cv.waitKey(0)

        return frame_norm

    def image_cross(self):

        frame = self.cam.get_frame(exp_time=20)

        high = np.max(frame)
        frame_norm = np.uint8(frame/high * 255)
        #cv.imshow('Image', frame_norm)
        #print(f"row = {len(frame)}, col={len(frame[0])}")

        frame_norm_cross = frame_norm.copy()
        frame_norm_cross[511][686]=0
        frame_norm_cross[511][687]=0
        frame_norm_cross[511][688]=0
        frame_norm_cross[511][689]=0
        frame_norm_cross[511][690]=0

        frame_norm_cross[512][686]=0
        frame_norm_cross[512][687]=0
        frame_norm_cross[512][688]=0
        frame_norm_cross[512][689]=0
        frame_norm_cross[512][690]=0
        
        frame_norm_cross[513][686]=0
        frame_norm_cross[513][687]=0
        frame_norm_cross[513][688]=0
        frame_norm_cross[513][689]=0
        frame_norm_cross[513][690]=0

        frame_norm_cross[510][687]=0
        frame_norm_cross[511][687]=0
        frame_norm_cross[512][687]=0
        frame_norm_cross[513][687]=0
        frame_norm_cross[514][687]=0

        frame_norm_cross[510][688]=0
        frame_norm_cross[511][688]=0
        frame_norm_cross[512][688]=0
        frame_norm_cross[513][688]=0
        frame_norm_cross[514][688]=0

        frame_norm_cross[510][689]=0
        frame_norm_cross[511][689]=0
        frame_norm_cross[512][689]=0
        frame_norm_cross[513][689]=0
        frame_norm_cross[514][689]=0
        #cv.imshow('Image', frame_norm_cross)
        #cv.waitKey(0)

        return frame_norm_cross


    def contour(self):

        input_image = self.image()

        # Find Canny edges
        edged = cv.Canny(input_image, 30, 200)
        #cv.waitKey(0)
  
        # Finding Contours
        # Use a copy of the image e.g. edged.copy()
        # since findContours alters the image
        contours, hierarchy = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
  
        point_list=np.empty(shape=[0,2])

        for contour in contours:
            for point in contour:
                #print(point)
                point_list=np.append(point_list,point,axis=0)
                #print(point_list)
                #cv.waitKey(0)


        print("______")

        #point_std = np.std(point_list,axis=0)
        #print(point_std)

        avg = np.average(point_list, axis=0)
        print(avg)

        if not np.isnan(np.average(point_list)):
            minx_idx = np.argmin(point_list[:, 0])
            print(f"min point inx{minx_idx}")
            minx_idx_coord = point_list[minx_idx]
            print(f"min point coord{minx_idx_coord}")
            print(minx_idx_coord)
            print(point_list)
            copy_input_image = input_image.copy()
            
            copy_input_image[int(minx_idx_coord[1])][int(minx_idx_coord[0])]=0
            tip_coord = np.array([[int(minx_idx_coord[1]), int(minx_idx_coord[0])]])

        # look for farest point --------------
        #delta_list = np.empty(shape=[0,1])
        #for point in point_list:
        #    delta = point-avg
        #    delta_list = np.append(delta_list,np.linalg.norm(delta))

        #print(sum(delta_list))
        #print(np.isnan(sum(delta_list)))
        #tip_coord = np.empty(shape=[0,2])

        # look for farest point --------------
        #if not np.isnan(np.average(delta_list)):
        #    delta_idx = np.argmax(delta_list)
        #    delta_idx_coord = point_list[delta_idx]
        #    print(delta_idx_coord)
        #    #print(point_list)
        #    copy_input_image = input_image.copy()
        #    copy_input_image[int(delta_idx_coord[1])][int(delta_idx_coord[0])]=0
        #    tip_coord = np.array([[int(delta_idx_coord[1]), int(delta_idx_coord[0])]])

        #for point in point_list:
            #print(point)
            #if point[0]>=1024:
            #    continue
            #copy_input_image[int(point[1])][int(point[0])]=0
            #copy_input_image[int(delta_idx_coord[1])][int(delta_idx_coord[0])]=0
            #cv.imshow('Copy Input Image',copy_input_image)
            #cv.waitKey(0)
            
        else:
            tip_coord = avg

        print("______")

        #cv.waitKey(0)

        #cv.imshow('Canny Edges After Contouring', edged)
        #cv.waitKey(0)
  
        print("Number of Contours found = " + str(len(contours)))
  
        # Draw all contours
        # -1 signifies drawing all contours
        ##cv.drawContours(input_image, contours, -1, (0, 255, 0), 3)
        #cv.imshow('Contours', input_image)
        #cv.waitKey(0)
        #cv.destroyAllWindows()
        return tip_coord

    def sequence(self,NUM_FRAMES):

        count = 0

        self.cam.start_seq(exp_time=20, num_frames=NUM_FRAMES)
        while count < NUM_FRAMES:
            frame, fps, frame_count = self.cam.poll_frame()  #cam.poll_frame() is to read in the frames from camera

            low = np.amin(frame['pixel_data'])  #alias for min
            high = np.amax(frame['pixel_data']) #alias for max
            average = np.average(frame['pixel_data'])

            print('Min:{}\tMax:{}\tAverage:{:.0f}\tFrame Rate: {:.1f}\tFrame Count: {:.0f}\n'.format(low, high, average, fps, frame_count))
            count = count + 1

            time.sleep(0.05)

        self.cam.finish()

        # Test basic sequence methods
        frames = self.cam.get_sequence(NUM_FRAMES)
        for frame in frames:
            low = np.amin(frame)
            high = np.amax(frame)
            average = np.average(frame)

            print('Min:{}\tMax:{}\tAverage:{:.0f}\tFrame Count: {:.0f}\n'.format(low, high, average, count))
            count = count + 1

        time_list = [i*10 for i in range(1, NUM_FRAMES+1)]
        frames = self.cam.get_vtm_sequence(time_list, constants.EXP_RES_ONE_MILLISEC, NUM_FRAMES)
        for frame in frames:
            low = np.amin(frame)
            high = np.amax(frame)
            average = np.average(frame)

            print('Min:{}\tMax:{}\tAverage:{:.0f}\tFrame Count: {:.0f}\n'.format(low, high, average, count))
            count = count + 1

    def video(self):

        self.cam.start_live(exp_time=20)

        count = 0
        total = 0
        t1 = time.time()
        start = time.time()
        width = 800
        height = int(self.cam.sensor_size[1] * width / self.cam.sensor_size[0])
        dim = (width, height)
        fps = 0

        while True:
            frame, fps, frame_count = self.cam.poll_frame()  #cam.poll_frame() is to read in the frames from camera
            frame['pixel_data'] = cv.resize(frame['pixel_data'], dim, interpolation = cv.INTER_AREA)

            high = np.max(frame['pixel_data'])

            cv.imshow('Live', frame['pixel_data']/high)

            if count == 10:
                t1 = time.time() - t1
                fps = 10/t1
                t1 = time.time()
                count = 0

            if cv.waitKey(10) == 27:  #Press ESC to end video feed
                break

            print('Frame Rate: {:.1f}\n'.format(fps))
            count = count + 1
            total = total + 1
        
        self.cam.finish()

    def calculate_focus_score(self):

        #blur = cv.GaussianBlur(self.image(),(3,3),0)
        blur = self.image()
        high = np.max(blur)
        norm = blur/high

        laplacian = cv.Laplacian(norm,cv.CV_64F)
        focus_score = laplacian.var()
        print(focus_score)
        return focus_score

    def autofocus(self,input_stepsize,max_distance):

        local_x,local_y,local_z = self.slicescope_instance.coordinates()

        local_x,local_y,local_z = self.slicescope_instance.moveRelative(local_x,local_y,local_z,0,0,max_distance/2)
        #Maintain minimum gap between the sample and the 4x objective
        #min_z_abs = -1250000  #max mechanical movement range
        #min_z_abs = -500_00  #Minimum for 4x objective focus.

        best_focus_score = 1
        best_focus_position = 0

        increment = abs(input_stepsize)
        min_increment = 1_00   #1.00 um

        #window is +- max_distance/2 for autofocus
        window_min = local_z - max_distance

        #Number of steps using step size as input
        steps = int(np.ceil( max_distance / increment ))

        for step in range(0,steps,1):

            #Check the z coordinate to ensure slicescope_z does not go beyond minimum z threshold
            z_travel = local_z - increment

            if z_travel >= window_min:
                local_x,local_y,local_z = self.slicescope_instance.moveRelative(local_x,local_y,local_z,0,0,-increment)
                focus_score = self.calculate_focus_score()

                if focus_score < best_focus_score:
                    best_focus_score = focus_score
                    best_focus_position = local_z 

            else:
                break

        
        #Always try to center the cell or probe tip for the autofocus. This ensures slicescope x,y are fixed. Only difference is relative z.
        local_x,local_y,local_z = self.slicescope_instance.moveRelative(local_x,local_y,local_z,0,0,best_focus_position-local_z)

        #self.image_stop()

        return local_x,local_y,local_z

    def disconnect(self):
    
        self.cam.close()
        pvc.uninit_pvcam()


def main():
    
    print('-----MAIN-----')

    #Global coordinates for the system
    global slicescope_x, slicescope_y, slicescope_z, patchstar_x, patchstar_y, patchstar_z, patchstar_a #, condenser_z


    print('-----Slicescope-----')

    slicescope = Slicescope('COM3')
    
    slicescope_x,slicescope_y,slicescope_z = slicescope.coordinates()
    print(f"Slicescope values X = {slicescope_x}, Y = {slicescope_y}, Z = {slicescope_z}")


    print('-----Micromanipulator-----')

    patchstar = Patchstar('COM5')

    patchstar_x,patchstar_y,patchstar_z,patchstar_a = patchstar.coordinates()
    print(f"Micromanipulator values X = {patchstar_x}, Y = {patchstar_y}, Z = {patchstar_z}, A = {patchstar_a}")


    print('-----Camera-----')

    cam = PVCAM(slicescope)

    x_s,y_s,z_s = slicescope.coordinates()
    print(f"x={x_s},y={y_s},z={z_s}")

    x_p,y_p,z_p,a_p = patchstar.coordinates()
    print(f"x={x_p},y={y_p},z={z_p},a={a_p}")

    #patchstar_a = patchstar.approachRelative(patchstar_a,5000_00)
    #cam.image()
    #cv.waitKey(0)
    #patchstar_a = patchstar.approachRelative(patchstar_a,-5000_00)
    #slicescope_x,slicescope_y,slicescope_z = slicescope.moveRelative(slicescope_x,slicescope_y,slicescope_z,0,-50_00,0)
    #cam.image()
    #cv.waitKey(0)

    #patchstar_x,patchstar_y,patchstar_z = patchstar.moveRelative(patchstar_x,patchstar_y,patchstar_z,240_20,-178_11,0)
    #patchstar_x,patchstar_y,patchstar_z = patchstar.moveRelative(patchstar_x,patchstar_y,patchstar_z,-5_00,0,0)
    #patchstar_x,patchstar_y,patchstar_z = patchstar.moveRelative(patchstar_x,patchstar_y,patchstar_z,-500_00,500_00,0)
    
    print(f"Angle = {patchstar.probe_angle()}")
    #print(f"Probe scale = {patchstar.probe_scale()}")
    #print(f"SS scale = {slicescope.slicescope_scale()}")


    img = cam.image_cross()

    print("Is it correct angle equal to 29?")
    print("   If NO, press ESC to exit program.")
    print("   If YES, press any key to continue.")    
    cv.imshow('Image',img)

    if cv.waitKey(0)==27:  #Press ESC to exit
        sys.exit()


    #Move slicescope or probe away to test autofocus
    #slicescope_x,slicescope_y,slicescope_z = slicescope.moveRelative(slicescope_x,slicescope_y,slicescope_z,0,-10_00,0)
    #patchstar_x,patchstar_y,patchstar_z = patchstar.moveRelative(patchstar_x,patchstar_y,patchstar_z,-50_00,50_00,0)
    #patchstar_a = patchstar.approachRelative(patchstar_a,-1000_00)
    
    #Autofocus
    #slicescope_x,slicescope_y,slicescope_z = cam.autofocus(10_00,400_00)
    
    #Check if contours return a valid value
    tip_coord = cam.contour()
    print(tip_coord)
    #if np.isnan(np.average(tip_coord)):
    #    print("Did not find correct contour value.")
    #    sys.exit()

    img = cam.image_cross()

    print("Is the probe tip around the centre mark?")
    print("   If NO, press ESC to exit program.")
    print("   If YES, press any key to continue.")    
    cv.imshow('Image',img)

    if cv.waitKey(0)==27:  #Press ESC to exit
        sys.exit()


    #Movement loop
    slicescope_movement = 100_00
    slicescope_movement_list = np.array([[1,-1],[-2,0],[0,2],[2,0],[-1,-1]])*slicescope_movement
    slicescope_movement_idx = 0

    #Start demo loop
    while True:
        
        #Calibration loop
        cnt = 0
        while True:
            cnt = cnt + 1
            slicescope_x,slicescope_y,slicescope_z = slicescope.moveRelative(slicescope_x,slicescope_y,slicescope_z,0,0,-5_00)
            tip_coord = cam.contour()
            print(tip_coord)
            if np.isnan(np.average(tip_coord)):
                break
            #cam.calculate_focus_score()
            #cam.image_stop()

        cnt = 0
        while True:
            cnt = cnt + 1
            slicescope_x,slicescope_y,slicescope_z = slicescope.moveRelative(slicescope_x,slicescope_y,slicescope_z,0,0,1_00)
            tip_coord = cam.contour()
            print(tip_coord)
            if not np.isnan(np.average(tip_coord)):
                break
            #cam.calculate_focus_score()
            print(f"iteration = {cnt}")
            #cam.image_stop()

        tip_coord_list = np.empty(shape=[0,2])
        for idx in range(0,20,1):
            slicescope_x,slicescope_y,slicescope_z = slicescope.moveRelative(slicescope_x,slicescope_y,slicescope_z,0,0,1_00)
            tip_coord = cam.contour()
            print(f"tip = {tip_coord}")
            if not np.isnan(np.average(tip_coord)):
                tip_coord_list = np.append(tip_coord_list,tip_coord,axis=0)
        print(tip_coord_list) # (y values ,x values)

        #cam.image_cross()

        #Move tip to better focus for low magnification objective
        slicescope_x,slicescope_y,slicescope_z = slicescope.moveRelative(slicescope_x,slicescope_y,slicescope_z,0,0,-5_00)

        #cam.image_cross()

        #Histogram of all the x,y coordinate pixels for tip 
        reson = 10
        centre_coord = np.array([688,512])
        
        # REPLACED W/ hist_top_coord FUNCTION
        #tip_coord_list_x = tip_coord_list[:,1]
        #tip_coord_list_y = tip_coord_list[:,0]
        #tip_coord_list_x_range = np.max(tip_coord_list_x) - np.min(tip_coord_list_x)
        #tip_coord_x_hist, tip_coord_x_edge = np.histogram(tip_coord_list_x,int(tip_coord_list_x_range/reson)+1)
        #bin_idx_x = np.argmax(tip_coord_x_hist)
        
        #tip_coord_list_y_range = np.max(tip_coord_list_y) - np.min(tip_coord_list_y)
        #tip_coord_y_hist, tip_coord_y_edge = np.histogram(tip_coord_list_y,int(tip_coord_list_y_range/reson)+1)
        #bin_idx_y = np.argmax(tip_coord_y_hist)

        #tip_coord_most = np.array([int(np.average([tip_coord_x_edge[bin_idx_x], tip_coord_x_edge[bin_idx_x + 1]])), int(np.average([tip_coord_y_edge[bin_idx_y], tip_coord_y_edge[bin_idx_y + 1]]))])
        #print(f"centre distance = {np.linalg.norm(centre_coord - tip_coord_most)}")

        #print(f"tip coord most = {tip_coord_most}")
        tip_coord_most, *_ = hist_top_coord(np.append(tip_coord_list[:, 1], tip_coord_list[:, 0], axis = 1), reson)


        #Relative coordinate plane rotation/translation for patchstar to slicescope
        #Mechanical movement to pixel translation for patchstar
        pixel_2_patch_scale = 165.45
        pixel_2_patch_angle = np.deg2rad(16.3)  # angle in rad
        #20230721 calibrated
        pixel_2_patch_scale = 163.025
        pixel_2_patch_angle = 0.21146  # angle in rad

        centre_dis = (centre_coord - tip_coord_most)
        print(f"centre distance on pixel = {centre_dis}")
        patch_centre_dis_x = (np.cos(pixel_2_patch_angle)*centre_dis[0] - np.sin(pixel_2_patch_angle)*centre_dis[1])*pixel_2_patch_scale
        patch_centre_dis_y = (np.sin(pixel_2_patch_angle)*centre_dis[0] + np.cos(pixel_2_patch_angle)*centre_dis[1])*pixel_2_patch_scale
        print(f"centre distance on patch = {[patch_centre_dis_x, patch_centre_dis_y]}")
        patchstar_x,patchstar_y,patchstar_z = patchstar.moveRelative(patchstar_x,patchstar_y,patchstar_z,int(patch_centre_dis_x),int(patch_centre_dis_y),0)
        
        print("______________________________________________________________________________________________")
        print("  Calibration is finished. Press any key to continue.")
        print("______________________________________________________________________________________________")

        img = cam.image_cross()
        cv.imshow('Image',img)
        cv.waitKey(0)

        #Move tip away from center focused position
        patchstar_a = patchstar.approachRelative(patchstar_a,1000_00)
        
        img = cam.image_cross()
        cv.imshow('Image',img)
        cv.waitKey(5)
        
        time.sleep(5)

        #Move slicescope to next coordinate in Movement loop
        slicescope_x,slicescope_y,slicescope_z = slicescope.moveRelative(slicescope_x,slicescope_y,slicescope_z,
                                                                         int(slicescope_movement_list[slicescope_movement_idx,0]),
                                                                         int(slicescope_movement_list[slicescope_movement_idx,1]),0)
        print("------------scope movement----------------")
        print(f"scope move index [0~4] is {slicescope_movement_idx}")
        print(f"scope move x = {int(slicescope_movement_list[slicescope_movement_idx,0])}")
        print(f"scope move y = {int(slicescope_movement_list[slicescope_movement_idx,1])}")

        img = cam.image_cross()
        cv.imshow('Image',img)
        cv.waitKey(5)

        time.sleep(5)

        #Mechanical movement to pixel translation for slicescope
        pixel_2_scope_scale = 15.88
        # Stage Calibration 20230721
        pixel_2_scope_scale = 16.3065
        pixel_2_scope_angle = -0.00848

        #Without camera-stage angle correction
        #pixel_movement = slicescope_movement_list[slicescope_movement_idx]/pixel_2_scope_scale
        
        #With camera-stage angle correction
        scope_scaled_movement = slicescope_movement_list[slicescope_movement_idx]/pixel_2_scope_scale
        print(f"non-rotated pixel movement = {scope_scaled_movement}")
        pixel_movement_x = (np.cos(pixel_2_scope_angle)*scope_scaled_movement[0] + np.sin(pixel_2_scope_angle)*scope_scaled_movement[1])
        pixel_movement_y = ( (-1) * np.sin(pixel_2_scope_angle)*scope_scaled_movement[0] + np.cos(pixel_2_scope_angle)*scope_scaled_movement[1])
        pixel_movement = [pixel_movement_x, pixel_movement_y]
        print(f"rotated pixel movement = {pixel_movement}")
        
        patch_centre_dis_x = (np.cos(pixel_2_patch_angle)*pixel_movement[0] - np.sin(pixel_2_patch_angle)*pixel_movement[1])*pixel_2_patch_scale
        patch_centre_dis_y = (np.sin(pixel_2_patch_angle)*pixel_movement[0] + np.cos(pixel_2_patch_angle)*pixel_movement[1])*pixel_2_patch_scale
        print(f"centre distance on patch = {[patch_centre_dis_x, patch_centre_dis_y]}")
        
        #img = cam.image_cross()
        #cv.imshow('Image',img)
        #cv.waitKey(0)

        patchstar_x,patchstar_y,patchstar_z = patchstar.moveRelative(patchstar_x,patchstar_y,patchstar_z,int(patch_centre_dis_x),int(patch_centre_dis_y),0)
        
        img = cam.image_cross()
        cv.imshow('Image',img)
        cv.waitKey(5)

        #Move tip back into camera fov/frame in steps
        for idx in range(0,5,1):
            patchstar_a = patchstar.approachRelative(patchstar_a,-200_00)
            #cam.image_stop()
            img = cam.image_cross()
            cv.imshow('Image',img)
            cv.waitKey(5)
    
        if slicescope_movement_idx < (np.shape(slicescope_movement_list)[0] - 1):
            slicescope_movement_idx = slicescope_movement_idx + 1
        else:
            slicescope_movement_idx = 0
            img = cam.image_cross()
            cv.imshow('Image', img)
            print("This is the end of the while loop. Press any key to continue. Press ESC to exit.")
            if cv.waitKey(10000)==27:  #Press ESC to exit
                break
            print("Loop started. Here we go again!")


    #Close and disconnect all equipment

    cam.disconnect()
    
    patchstar.close()

    slicescope.close()

    print('-----End of MAIN-----')
    
    """
    #Run Condenser commands

    print('-----Condenser-----')
    condenser = Condenser('COM4')

    #condenser_z = condenser.coordinates()
    #print(f"Current Condenser value Z = {condenser_z}")

    #z coordinates only. - is UP. + is DOWN. Only Condenser is flipped.
    # move absolute (abs_z)
    condenser_z = condenser.moveAbsolute(1560000)    #Move Condenser down to lowest position.
    print(f"Current Condenser value Z = {condenser_z}")

    # move relative (current_z,rel_z)
    #condenser_z = condenser.moveRelative(condenser_z,-10000) #Negative numbers = UP.

    condenser.close()
    """


#MAIN

if __name__ == '__main__':  #This will ensure main is called first and then you can import other functions and run them inside main
    main()


"""
#Commands that also work but are extraneous in the existing code.

command = 'ABS {} {}\r'.format(patchstar_x,patchstar_y)  #x,y coordinates    
obj.write(command.encode('utf-8'))
print('Moved Micromanipulator to ABS coords = {} {}'.format(patchstar_x,patchstar_y))   #Read the message/acknowledgment from the command
    
command = 'ABSZ {}\r'.format(patchstar_z)  #z coordinates only    
obj.write(command.encode('utf-8'))
print('Moved Micromanipulator to ABS coords = {}'.format(patchstar_z))   #Read the message/acknowledgment from the command


command = 'REL {} {}\r'.format(new_rel_x,new_rel_y)
obj.write(command.encode('utf-8'))
print('Moved Slicescope by REL coords = {} {}'.format(new_rel_x,new_rel_y))

command = 'RELZ {}\r'.format(new_rel_z)
obj.write(command.encode('utf-8'))
print('Moved Slicescope by REL coords = {}'.format(new_rel_z))


#Also can move objective up using OBJU. Same as RELZ.
#Move objective up by set distance 'OBJLIFT'
obj.write(b'OBJLIFT\r')

command = 'OBJLIFT {}\r'.format(rel_z)

obj.write(command.encode('utf-8'))

#Move objective up set distance 'OBJLIFT'
obj.write(b'OBJU\r')

print('Moved Micromanipulator UP by z = {}'.format(rel_z))

###########################################################

#Do not use this. Quicker to manually set the coordinate system origin using patchstar/cube and GUI to zero.
#Only use P to rename coordinates for current position
#Only use ZERO to zero coordinates for current position
#Set coordinates X Y Z A: P # # # # 
#Set x coord PX: PX #
#Set y coord PY: PY #
#Set z coord PZ: PZ #
#Set a coord PA: PA # might only be able to use with micromanipulator
    
    #slicescope.write(b'PX 0\r')
    #slicescope.write(b'PY 0\r')
    #slicescope.write(b'PZ 0\r')
    #slicescope.write(b'PA 0\r')

#Commands that do not work.
    #slicescope.write(b'APPROACH 1\r')  #Does not work. No approach available. Always '0' regardless of which objective is in use.
    #print(slicescope.readline())

    #slicescope.write(b'APPROACH\r')   #Does not work.
    #print(slicescope.readline())
    
    #Which objective is in use (Doesn't work like it should. Always says '2' or 'REAR' objective in use...)
    # slicescope.write(b'OBJ\r')
    # print(slicescope.readline())
    
    #Move objective down set distance 'OBJLIFT' (Does not work)
    #slicescope.write(b'OBJD\r')
    #print(slicescope.readline())
    
    #Switch objective (Does not work.)
    #slicescope.write(b'OBJ 1\r')
    #print(slicescope.readline())
"""