#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed August 9 12:21:00 2023

@author: Dr. Vincent Lee, Dr. WeiDa Zhang
Bioptix Ltd.

"""

# Use # for single line comment
# Use ''' ''' or """ """ as bookends for multiple line comments. Must be indented!

import serial 
import re


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

    def stepOut(self,patchstar_z,patchstar_a,step_out):
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

    def stepIn(self,patchstar_z,patchstar_a,step_in):
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
