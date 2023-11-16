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
import numpy as np

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
        self.x_origin = [] #8102
        self.y_origin = [] #6591
        self.z_origin = [] #-8
        self.a_origin = [] #128756
        self.x_max = [] #1054764
        self.x_min = [] #-1038559
        self.y_max = [] #1054892
        self.y_min = [] #-1041709
        self.z_max = [] #1050184
        self.z_min = [] #-1050200
        self.a_max = [] #1485026
        self.a_min = [] #-1227512
        self.x_delta = [] #1046661
        self.y_delta = [] #1048300
        self.z_delta = [] #1050192
        self.a_delta = [] #1356269
        self.probe_x_max = [] #1053426
        self.probe_z_max = [] #1035098
        self.probe_x_min = [] #-1035980
        self.probe_z_min = [] #-1034474
        self.x_scale = []
        self.y_scale = []
        self.z_scale = []
        self.a_scale = []
        self.probe_angle = []

    def description(self):
        return "Micromanipulator"    

    def coordinates(self):

        #Get xyza coordinates
        self.ser.write(b'P ? \r')
        byte_to_string = self.ser.readline().decode('utf-8')
        coords = [ int(s) for s in re.findall( r'[-+]?\d+' , byte_to_string) ]

        self.x = coords[0]
        self.y = coords[1]
        self.z = coords[2]
        self.a = coords[3]

    def deltas(self):

        #check if max and min values are not empty lists
        if self.x_max:
            if self.x_min:
                #check if max and min values are not None
                if self.x_max is not None: 
                    if self.x_min is not None:
                        #check if max and min values are not NaN values
                        if not (np.isnan(self.x_max) or np.isnan(self.x_min)):

                            if not np.isnan(self.x):
                                if self.x > 0:
                                    self.x_delta = int(abs(self.x_max - self.x))
                                elif self.x < 0:
                                    self.x_delta = int(abs(self.x_min - self.x))
                                else:
                                    self.x_delta = int(abs((self.x_max - self.x_min)/2))
        else:
            self.x_max = None
            self.x_min = None
            self.x_delta = None

        #check if max and min values are not empty lists
        if self.y_max:
            if self.y_min:
                #check if max and min values are not None
                if self.y_max is not None: 
                    if self.y_min is not None:
                        #check if max and min values are not NaN values
                        if not (np.isnan(self.y_max) or np.isnan(self.y_min)):

                            if not np.isnan(self.y):
                                if self.y > 0:
                                    self.y_delta = int(abs(self.y_max - self.y))
                                elif self.y < 0:
                                    self.y_delta = int(abs(self.y_min - self.y))
                                else:
                                    self.y_delta = int(abs((self.y_max - self.y_min)/2))
        else:
            self.y_max = None
            self.y_min = None
            self.y_delta = None

        #check if max and min values are not empty lists
        if self.z_max:
            if self.z_min: 
                #check if max and min values are not None
                if self.z_max is not None: 
                    if self.z_min is not None:
                        #check if max and min values are not NaN values
                        if not (np.isnan(self.z_max) or np.isnan(self.z_min)):

                            if not np.isnan(self.z):
                                if self.z > 0:
                                    self.z_delta = int(abs(self.z_max - self.z))
                                elif self.z < 0:
                                    self.z_delta = int(abs(self.z_min - self.z))
                                else:
                                    self.z_delta = int(abs((self.z_max - self.z_min)/2))
        else:
            self.z_max = None
            self.z_min = None
            self.z_delta = None

        #check if max and min values are not empty lists
        if self.a_max:
            if self.a_min:
                #check if max and min values are not None
                if self.a_max is not None: 
                    if self.a_min is not None:
                        #check if max and min values are not NaN values
                        if not (np.isnan(self.a_max) or np.isnan(self.a_min)):

                            if not np.isnan(self.a):
                                if self.a > 0:
                                    self.a_delta = int(abs(self.a_max - self.a))
                                elif self.a < 0:
                                    self.a_delta = int(abs(self.a_min - self.a))
                                else:
                                    self.a_delta = int(abs((self.a_max - self.a_min)/2))
        else:
            self.a_max = None
            self.a_min = None
            self.a_delta = None

    def angle(self):

        self.ser.write(b'ANGLE \r')
        byte_to_string = self.ser.readline().decode('utf-8')
        probe_angle = [ int(s) for s in re.findall( r'[-+]?\d+' , byte_to_string) ]

        self.probe_angle = probe_angle[0]

    def scale(self):

        #X axis scale
        self.ser.write(b'SCALE X \r')
        byte_to_string = self.ser.readline().decode('utf-8')
        x_scale = [ int(s) for s in re.findall( r'[-+]?\d+' , byte_to_string) ]

        self.x_scale = x_scale[0]

        #Y axis scale
        self.ser.write(b'SCALE Y \r')
        byte_to_string = self.ser.readline().decode('utf-8')
        y_scale = [ int(s) for s in re.findall( r'[-+]?\d+' , byte_to_string) ]

        self.y_scale = y_scale[0]

        #Z axis scale
        self.ser.write(b'SCALE Z \r')
        byte_to_string = self.ser.readline().decode('utf-8')
        z_scale = [ int(s) for s in re.findall( r'[-+]?\d+' , byte_to_string) ]

        self.z_scale = z_scale[0]

        #A axis scale
        self.ser.write(b'SCALE A \r')
        byte_to_string = self.ser.readline().decode('utf-8')
        a_scale = [ int(s) for s in re.findall( r'[-+]?\d+' , byte_to_string) ]

        self.a_scale = a_scale[0]

    def calibration(self):

        x_step = 100000_00
        y_step = 100000_00
        z_step = 100000_00
        a_step = 100000_00

        #Patchstar
        # X axis calibration
        self.moveRelativeNoLimit(self.x,self.y,self.z,x_step,0,0)
        self.x_max = self.x
        self.moveRelativeNoLimit(self.x,self.y,self.z,-x_step,0,0)
        self.x_min = self.x

        self.x_delta = int(abs((self.x_max - self.x_min)/2))
        
        #Move patchstar back to center
        self.moveRelativeNoLimit(self.x,self.y,self.z,self.x_delta,0,0)
        self.x_origin = self.x

        # Y axis calibration
        self.moveRelativeNoLimit(self.x,self.y,self.z,0,y_step,0)
        self.y_max = self.y
        self.moveRelativeNoLimit(self.x,self.y,self.z,0,-y_step,0)
        self.y_min = self.y

        self.y_delta = int(abs((self.y_max - self.y_min)/2))

        #Move patchstar back to center
        self.moveRelativeNoLimit(self.x,self.y,self.z,0,self.y_delta,0)
        self.y_origin = self.y

        # Z axis calibration
        self.moveRelativeNoLimit(self.x,self.y,self.z,0,0,z_step)
        self.z_max = self.z
        self.moveRelativeNoLimit(self.x,self.y,self.z,0,0,-z_step)
        self.z_min = self.z

        self.z_delta = int(abs((self.z_max - self.z_min)/2))

        #Move patchstar back to center
        self.moveRelativeNoLimit(self.x,self.y,self.z,0,0,self.z_delta)
        self.z_origin = self.z

        # A axis calibration
        self.approachRelativeNoLimit(self.a,a_step)
        self.a_max = self.a
        self.probe_x_max = self.x
        self.probe_z_max = self.z
        self.approachRelativeNoLimit(self.a,-a_step)
        self.a_min = self.a
        self.probe_x_min = self.x
        self.probe_z_min = self.z

        self.a_delta = int(abs((self.a_max - self.a_min)/2))

        #Move probe (Approach) back to center
        self.approachRelativeNoLimit(self.a,self.a_delta)
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
            self.x = self.x_max
        
        elif abs_x < self.x_min:
            self.x = self.x_min

        else: 
            self.x = abs_x

        if self.x > 0:
            self.x_delta = int(abs(self.x_max - self.x))
        elif self.x < 0:
            self.x_delta = int(abs(self.x_min - self.x))
        else:
            self.x_delta = int(abs((self.x_max - self.x_min)/2))

        if abs_y > self.y_max:
            self.y = self.y_max
        
        elif abs_y < self.y_min:
            self.y = self.y_min

        else: 
            self.y = abs_y

        if self.y > 0:
            self.y_delta = int(abs(self.y_max - self.y))
        elif self.y < 0:
            self.y_delta = int(abs(self.y_min - self.y))
        else:
            self.y_delta = int(abs((self.y_max - self.y_min)/2))

        if abs_z > self.z_max:
            self.z = self.z_max
        
        elif abs_z < self.z_min:
            self.z = self.z_min

        else: 
            self.z = abs_z

        if self.z > 0:
            self.z_delta = int(abs(self.z_max - self.z))
        elif self.z < 0:
            self.z_delta = int(abs(self.z_min - self.z))
        else:
            self.z_delta = int(abs((self.z_max - self.z_min)/2))

        command = f"ABS {self.x} {self.y} {self.z}\r"  #x,y,z coordinates 
        self.ser.write(command.encode('utf-8'))

        print(f"Moved Micromanipulator to ABS coords = {self.x} {self.y} {self.z}")   #Read the message/acknowledgment from the command

        self.wait()

    def moveRelativeNoLimit(self,current_x,current_y,current_z,rel_x,rel_y,rel_z):
    #Move to NEW COORDINATES by new values relative to current coordinates

        command = f"REL {rel_x} {rel_y} {rel_z}\r"
        self.ser.write(command.encode('utf-8'))

        print(f"Moved Micromanipulator by REL coords = {rel_x} {rel_y} {rel_z}")
 
        self.wait()

        self.coordinates()

        print(f"New Micromanipulator coordinates = {self.x} {self.y} {self.z} {self.a}")

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
            self.x = self.x_max

        elif new_x < self.x_min:

            new_rel_x = int(self.x_min - current_x)
            self.x = self.x_min

        else:

            new_rel_x = rel_x
            self.x = new_x

        if self.x > 0:
            self.x_delta = int(abs(self.x_max - self.x))
        elif self.x < 0:
            self.x_delta = int(abs(self.x_min - self.x))
        else:
            self.x_delta = int(abs((self.x_max - self.x_min)/2))

        print(f"Micromanipulator X coordinate is now = {self.x}")

    #Set a coord RELY x: RELY #

        new_y = current_y + rel_y   

        if new_y > self.y_max:
        
            new_rel_y = int(self.y_max - current_y)
            self.y = self.y_max

        elif new_y < self.y_min:
        
            new_rel_y = int(self.y_min - current_y)
            self.y = self.y_min

        else:
        
            new_rel_y = rel_y
            self.y = new_y

        if self.y > 0:
            self.y_delta = int(abs(self.y_max - self.y))
        elif self.y < 0:
            self.y_delta = int(abs(self.y_min - self.y))
        else:
            self.y_delta = int(abs((self.y_max - self.y_min)/2))

        print(f"Micromanipulator Y coordinate is now = {self.y}")

    #Set a coord RELZ x: RELZ #

        new_z = current_z + rel_z        

        if new_z > self.z_max:
        
            new_rel_z = int(self.z_max - current_z)
            self.z = self.z_max

        elif new_z < self.z_min:
        
            new_rel_z = int(self.z_min - current_z)
            self.z = self.z_min

        else:
        
            new_rel_z = rel_z
            self.z = new_z

        if self.z > 0:
            self.z_delta = int(abs(self.z_max - self.z))
        elif self.z < 0:
            self.z_delta = int(abs(self.z_min - self.z))
        else:
            self.z_delta = int(abs((self.z_max - self.z_min)/2))

        print(f"Micromanipulator Z coordinate is now = {self.z}")

    #Move to NEW COORDINATES by new values relative to current coordinates

        command = f"REL {new_rel_x} {new_rel_y} {new_rel_z}\r"
        self.ser.write(command.encode('utf-8'))

        print(f"Moved Micromanipulator by REL coords = {new_rel_x} {new_rel_y} {new_rel_z}")
        print(f"New Micromanipulator coordinates = {self.x} {self.y} {self.z}")

        self.wait()

    def approachAbsolute(self,abs_a): 

    #Move to absolute coordinate A=XZ
    #Set a coord ABSA x: ABSA #

    #Limit is +-12,000.00 or 12e5

        if abs_a > self.a_max:
            self.a = self.a_max
        
        elif abs_a < self.a_min:
            self.a = self.a_min

        else: 
            self.a = abs_a

        if self.a > 0:
            self.a_delta = int(abs(self.a_max - self.a))
        elif self.a < 0:
            self.a_delta = int(abs(self.a_min - self.a))
        else:
            self.a_delta = int(abs((self.a_max - self.a_min)/2))

        command = f"ABSA {self.a}\r"  #A=XZ coordinates 
        self.ser.write(command.encode('utf-8'))

        print(f"Moved Probe to approach ABS = {self.a}")   #Read the message/acknowledgment from the command

        self.wait()

    def approachRelativeNoLimit(self,current_a,rel_a):  

    #Move to relative coordinate A=XZ
    #Set a coord RELA x: RELA #     

        command = f"RELA {rel_a}\r"  #A=XZ coordinates 
        self.ser.write(command.encode('utf-8'))

        print(f"Moved Probe to approach REL = {rel_a}")   #Read the message/acknowledgment from the command

        self.wait()

        self.coordinates()

        print(f"New Micromanipulator coordinates = {self.x} {self.y} {self.z} {self.a}")  #Read the message/acknowledgment from the command

    def approachRelative(self,current_a,rel_a):  

    #Move to relative coordinate A=XZ
    #Set a coord RELA x: RELA #

    #Limit is +-12,000.00 or 12e5

        new_a = current_a + rel_a        

        if new_a > self.a_max:
        
            new_rel_a = int(self.a_max - current_a)
            self.a = self.a_max

        elif new_a < self.a_min:
        
            new_rel_a = int(self.a_min - current_a)
            self.a = self.a_min

        else:
        
            new_rel_a = rel_a
            self.a = new_a

        if self.a > 0:
            self.a_delta = int(abs(self.a_max - self.a))
        elif self.a < 0:
            self.a_delta = int(abs(self.a_min - self.a))
        else:
            self.a_delta = int(abs((self.a_max - self.a_min)/2))

        command = f"RELA {new_rel_a}\r"  #A=XZ coordinates 
        self.ser.write(command.encode('utf-8'))

        print(f"Moved Probe to approach REL = {new_rel_a}")   #Read the message/acknowledgment from the command
        print(f"Coordinate A = {self.a}")   #Read the message/acknowledgment from the command

        self.wait()

    def moveUp(self,current_z,rel_z):

        #Max is 12470.69 or 1.25e6
        #Min is -12470.00 or -1.25e6

        new_z = current_z + rel_z

        if rel_z > 0:

            if new_z > self.z_max:

                new_rel_z = int(self.z_max - current_z)
                self.z = self.z_max

            else:

                new_rel_z = rel_z
                self.z = new_z

            self.z_delta = int(abs(self.z_max - self.z))

            #Create string with command, then convert to byte to write to serial.

            command = f"OBJLIFT {new_rel_z}\r"  #Values in OBJLIFT can be negative.
            self.ser.write(command.encode('utf-8'))
            self.ser.write(b'OBJU\r')  #Moves like RELZ. 

            print(self.ser.readline().decode('utf-8'))

            print(f"Micromanipulator Z coordinate is now = {self.z}")

        self.wait()

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
                self.a = self.a_max

            else:

                new_rel_a = step_out
                self.a = new_a

            self.a_delta = int(abs(self.a_max - self.a))

            command = f"SETSTEP {new_rel_a}\r"  #Set step value.
            self.ser.write(command.encode('utf-8'))
            self.ser.write(b'STEP\r')    #Step out. Moves up in A=XZ direction by SETSTEP value

            print(f"Moved Micromanipulator OUT by STEP = {new_rel_a}")

            print(f"Micromanipulator A coordinate is now = {self.a}")

        else:

            #if APPROACH=0, then step can be used only for Z coordinate/direction 

            new_z = patchstar_z + step_out

            if new_z > self.z_max:

                new_rel_z = int(self.z_max - patchstar_z)
                self.z = self.z_max

            else:

                new_rel_z = step_out
                self.z = new_z

            self.z_delta = int(abs(self.z_max - self.z))

            command = f"SETSTEP {new_rel_a}\r"  #Set step value.
            self.ser.write(command.encode('utf-8'))
            self.ser.write(b'STEP\r')    #Step out. Moves up in Z direction by SETSTEP value

            print(f"Moved Micromanipulator OUT by STEP = {new_rel_z}")

            print(f"Micromanipulator Z coordinate is now = {self.z}")

        self.wait()

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
                self.a = self.a_min

            else:

                new_rel_a = step_in
                self.a = new_a

            self.a_delta = int(abs(self.a - self.a_min))

            command = f"SETSTEP {new_rel_a}\r"  #Set step value.
            self.ser.write(command.encode('utf-8'))
            self.ser.write(b'STEP B\r')    #Step out. Moves up in A=XZ direction by SETSTEP value

            print(f"Moved Micromanipulator IN by STEP = {new_rel_a}")

            print(f"Micromanipulator A coordinate is now = {self.a}")

        else:

            #if APPROACH=0, then step can be used only for Z coordinate/direction 
            
            new_z = patchstar_z - step_in

            if new_z < self.z_min:

                new_rel_z = int(patchstar_z - self.z_min)
                self.z = self.z_min

            else:

                new_rel_z = step_in
                self.z = new_z

            self.z_delta = int(abs(self.z - self.z_min))

            command = f"SETSTEP {new_rel_z}\r"  #Set step value.
            self.ser.write(command.encode('utf-8'))
            self.ser.write(b'STEP B\r')    #Step out. Moves up in Z direction by SETSTEP value

            print(f"Moved Micromanipulator OUT by STEP = {new_rel_z}")

            print(f"Micromanipulator Z coordinate is now = {self.z}")

        self.wait()

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
