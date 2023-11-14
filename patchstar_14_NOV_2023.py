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
        self.x = []
        self.y = []
        self.z = []
        self.a = []
        self.x_origin = []
        self.y_origin = []
        self.z_origin = []
        self.a_origin = []
        self.x_max = []
        self.x_min = []
        self.y_max = []
        self.y_min = []
        self.z_max = []
        self.z_min = []
        self.a_max = []
        self.a_min = []
        self.x_delta = []
        self.y_delta = []
        self.z_delta = []
        self.a_delta = []
        self.probe_x_max = []
        self.probe_z_max = []
        self.probe_x_min = []
        self.probe_z_min = []

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

    def angle(self):

        self.ser.write(b'ANGLE \r')
    
        angle = self.ser.readline().decode('utf-8')

        return angle

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

        #A axis scale
        self.ser.write(b'SCALE A \r')
        a_scale = self.ser.readline().decode('utf-8')

        return x_scale,y_scale,z_scale,a_scale

    def calibration(self):

        x_step = 100000_00
        y_step = 100000_00
        z_step = 100000_00
        a_step = 100000_00

        #Patchstar
        # X axis calibration
        self.x,self.y,self.z,self.a = self.moveRelativeNoLimit(self.x,self.y,self.z,x_step,0,0)
        self.x_max = self.x
        self.x,self.y,self.z,self.a = self.moveRelativeNoLimit(self.x,self.y,self.z,-x_step,0,0)
        self.x_min = self.x

        self.x_delta = int(abs((self.x_max - self.x_min)/2))
        
        #Move patchstar back to center
        self.x,self.y,self.z,self.a = self.moveRelativeNoLimit(self.x,self.y,self.z,self.x_delta,0,0)
        self.x_origin = self.x

        # Y axis calibration
        self.x,self.y,self.z,self.a = self.moveRelativeNoLimit(self.x,self.y,self.z,0,y_step,0)
        self.y_max = self.y
        self.x,self.y,self.z,self.a = self.moveRelativeNoLimit(self.x,self.y,self.z,0,-y_step,0)
        self.y_min = self.y

        self.y_delta = int(abs((self.y_max - self.y_min)/2))

        #Move patchstar back to center
        self.x,self.y,self.z,self.a = self.moveRelativeNoLimit(self.x,self.y,self.z,0,self.y_delta,0)
        self.y_origin = self.y

        # Z axis calibration
        self.x,self.y,self.z,self.a = self.moveRelativeNoLimit(self.x,self.y,self.z,0,0,z_step)
        self.z_max = self.z
        self.x,self.y,self.z,self.a = self.moveRelativeNoLimit(self.x,self.y,self.z,0,0,-z_step)
        self.z_min = self.z

        self.z_delta = int(abs((self.z_max - self.z_min)/2))

        #Move patchstar back to center
        self.x,self.y,self.z,self.a = self.moveRelativeNoLimit(self.x,self.y,self.z,0,0,self.z_delta)
        self.z_origin = self.z

        # A axis calibration
        self.x,self.y,self.z,self.a = self.approachRelativeNoLimit(self.a,a_step)
        self.a_max = self.a
        self.probe_x_max = self.x
        self.probe_z_max = self.z
        self.x,self.y,self.z,self.a = self.approachRelativeNoLimit(self.a,-a_step)
        self.a_min = self.a
        self.probe_x_min = self.x
        self.probe_z_min = self.z

        self.a_delta = int(abs((self.a_max - self.a_min)/2))

        #Move probe (Approach) back to center
        self.x,self.y,self.z,self.a = self.approachRelativeNoLimit(self.a,self.a_delta)
        self.a_origin = self.a

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

        command = f"ABS {local_x} {local_y} {local_z}\r"  #x,y,z coordinates 
        
        self.ser.write(command.encode('utf-8'))

        print(f"Moved Micromanipulator to ABS coords = {local_x} {local_y} {local_z}")   #Read the message/acknowledgment from the command

        self.wait()

        return local_x,local_y,local_z

    def moveRelativeNoLimit(self,current_x,current_y,current_z,rel_x,rel_y,rel_z):
    #Move to NEW COORDINATES by new values relative to current coordinates

        command = f"REL {rel_x} {rel_y} {rel_z}\r"
        
        self.ser.write(command.encode('utf-8'))

        print(f"Moved Micromanipulator by REL coords = {rel_x} {rel_y} {rel_z}")
 
        self.wait()

        new_x,new_y,new_z,new_a = self.coordinates()

        print(f"New Micromanipulator coordinates = {new_x} {new_y} {new_z} {new_a}")

        return new_x,new_y,new_z,new_a

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

        if local_x > 0:
            self.x_delta = int(abs(self.x_max - local_x))
        elif local_x < 0:
            self.x_delta = int(abs(self.x_min - local_x))
        else:
            self.x_delta = int(abs((self.x_max - self.x_min)/2))

        print(f"Micromanipulator X coordinate is now = {local_x}")

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

        if local_y > 0:
            self.y_delta = int(abs(self.y_max - local_y))
        elif local_y < 0:
            self.y_delta = int(abs(self.y_min - local_y))
        else:
            self.y_delta = int(abs((self.y_max - self.y_min)/2))

        print(f"Micromanipulator Y coordinate is now = {local_y}")

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

        if local_z > 0:
            self.z_delta = int(abs(self.z_max - local_z))
        elif local_z < 0:
            self.z_delta = int(abs(self.z_min - local_z))
        else:
            self.z_delta = int(abs((self.z_max - self.z_min)/2))

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

    #Limit is +-12,000.00 or 12e5

        if abs_a > self.a_max:
            local_a = self.a_max
        
        elif abs_a < self.a_min:
            local_a = self.a_min

        else: 
            local_a = abs_a

        if local_a > 0:
            self.a_delta = int(abs(self.a_max - local_a))
        elif local_a < 0:
            self.a_delta = int(abs(self.a_min - local_a))
        else:
            self.a_delta = int(abs((self.a_max - self.a_min)/2))

        command = f"ABSA {local_a}\r"  #A=XZ coordinates 

        self.ser.write(command.encode('utf-8'))

        print(f"Moved Probe to approach ABS = {local_a}")   #Read the message/acknowledgment from the command

        self.wait()

        return local_a

    def approachRelativeNoLimit(self,current_a,rel_a):  

    #Move to relative coordinate A=XZ
    #Set a coord RELA x: RELA #     

        command = f"RELA {rel_a}\r"  #A=XZ coordinates 

        self.ser.write(command.encode('utf-8'))

        print(f"Moved Probe to approach REL = {rel_a}")   #Read the message/acknowledgment from the command

        self.wait()

        new_x,new_y,new_z,new_a = self.coordinates()

        print(f"New Micromanipulator coordinates = {new_x} {new_y} {new_z} {new_a}")  #Read the message/acknowledgment from the command

        return new_x,new_y,new_z,new_a

    def approachRelative(self,current_a,rel_a):  

    #Move to relative coordinate A=XZ
    #Set a coord RELA x: RELA #

    #Limit is +-12,000.00 or 12e5

        new_a = current_a + rel_a        

        if new_a > self.a_max:
        
            new_rel_a = int(self.a_max - current_a)
            local_a = self.a_max

        elif new_a < self.a_min:
        
            new_rel_a = int(self.a_min - current_a)
            local_a = self.a_min

        else:
        
            new_rel_a = rel_a
            local_a = new_a

        if local_a > 0:
            self.a_delta = int(abs(self.a_max - local_a))
        elif local_a < 0:
            self.a_delta = int(abs(self.a_min - local_a))
        else:
            self.a_delta = int(abs((self.a_max - self.a_min)/2))

        command = f"RELA {new_rel_a}\r"  #A=XZ coordinates 

        self.ser.write(command.encode('utf-8'))

        print(f"Moved Probe to approach REL = {new_rel_a}")   #Read the message/acknowledgment from the command
        print(f"Coordinate A = {local_a}")   #Read the message/acknowledgment from the command

        self.wait()

        return local_a

    def moveUp(self,current_z,rel_z):

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

            print(f"Micromanipulator Z coordinate is now = {local_z}")

        self.wait()

        return local_z

    def stepOut(self,patchstar_z,patchstar_a,step_out):
    #Objective step in or out. Step in closer to sample. Step out away from sample.

        #Absolute value of step value
        step_out = abs(step_out)

        self.ser.write(b'APPROACH\r')

        byte_to_string = self.ser.readline().decode('utf-8')
    
        status = int(re.search( r'[-+]?\d+' , byte_to_string).group()) #Search for numbers and convert MatchObject (whatever integer matches) as String.

        if (status == 1):
            #if APPROACH=1, then step can be used for A coordinate/direction
            
            new_a = patchstar_a + step_out

            if new_a > self.a_max:

                new_rel_a = int(self.a_max - patchstar_a)
                local_a = self.a_max

            else:

                new_rel_a = step_out
                local_a = new_a

            self.a_delta = int(abs(self.a_max - local_a))

            command = f"SETSTEP {new_rel_a}\r"  #Set step value.

            self.ser.write(command.encode('utf-8'))

            self.ser.write(b'STEP\r')    #Step out. Moves up in A=XZ direction by SETSTEP value
            print(f"Moved Micromanipulator OUT by STEP = {new_rel_a}")

            print(f"Micromanipulator A coordinate is now = {local_a}")

        else:

            #if APPROACH=0, then step can be used only for Z coordinate/direction 

            new_z = patchstar_z + step_out

            if new_z > self.z_max:

                new_rel_z = int(self.z_max - patchstar_z)
                local_z = self.z_max

            else:

                new_rel_z = step_out
                local_z = new_z

            self.z_delta = int(abs(self.z_max - local_z))

            command = f"SETSTEP {new_rel_a}\r"  #Set step value.

            self.ser.write(command.encode('utf-8'))

            self.ser.write(b'STEP\r')    #Step out. Moves up in Z direction by SETSTEP value
            print(f"Moved Micromanipulator OUT by STEP = {new_rel_z}")

            print(f"Micromanipulator Z coordinate is now = {local_z}")

        self.wait()

        return local_z, local_a

    def stepIn(self,patchstar_z,patchstar_a,step_in):
    #Objective step in or out. Step in closer to sample. Step out away from sample.

        #Absolute value of step value
        step_in = abs(step_in)
    
        self.ser.write(b'APPROACH\r')

        byte_to_string = self.ser.readline().decode('utf-8')

        status = int(re.search( r'[-+]?\d+' , byte_to_string).group()) #Search for numbers and convert MatchObject (whatever integer matches) as String.

        if (status == 1):
            #if APPROACH=1, then step can be used for A coordinate/direction
            
            new_a = patchstar_a - step_in

            if new_a < self.a_min:

                new_rel_a = int(patchstar_a - self.a_min)
                local_a = self.a_min

            else:

                new_rel_a = step_in
                local_a = new_a

            self.a_delta = int(abs(local_a - self.a_min))

            command = f"SETSTEP {new_rel_a}\r"  #Set step value.

            self.ser.write(command.encode('utf-8'))
            
            self.ser.write(b'STEP B\r')    #Step out. Moves up in A=XZ direction by SETSTEP value
            print(f"Moved Micromanipulator IN by STEP = {new_rel_a}")

            print(f"Micromanipulator A coordinate is now = {local_a}")

        else:

            #if APPROACH=0, then step can be used only for Z coordinate/direction 
            
            new_z = patchstar_z - step_in

            if new_z < self.z_min:

                new_rel_z = int(patchstar_z - self.z_min)
                local_z = self.z_min

            else:

                new_rel_z = step_in
                local_z = new_z

            self.z_delta = int(abs(local_z - self.z_min))

            command = f"SETSTEP {new_rel_z}\r"  #Set step value.

            self.ser.write(command.encode('utf-8'))

            self.ser.write(b'STEP B\r')    #Step out. Moves up in Z direction by SETSTEP value
            print(f"Moved Micromanipulator OUT by STEP = {new_rel_z}")

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
