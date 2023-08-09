#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed August 9 12:20:00 2023

@author: Dr. Vincent Lee, Dr. WeiDa Zhang
Bioptix Ltd.

"""

# Use # for single line comment
# Use ''' ''' or """ """ as bookends for multiple line comments. Must be indented!

import serial 
import re


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
