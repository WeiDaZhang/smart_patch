#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu December 7 13:42:00 2023

@author: Dr. Vincent Lee
Bioptix Ltd.

"""

#Classes
from slicescope import Slicescope
from condenser import Condenser
from patchstar import Patchstar
from pvcam import PVCAM
#from tkgui import App
#from tkgui_highmag import App
from tkgui_highmag_app_window import App

def main():

    print('-----MAIN-----')

    print('-----Slicescope-----')

    slicescope = Slicescope('COM3')

    slicescope.coordinates()
    print(f"Slicescope values X = {slicescope.x}, Y = {slicescope.y}, Z = {slicescope.z}")

    print('-----Condenser-----')

    condenser = Condenser('COM4')

    condenser.coordinates()
    print(f"Condenser value Z = {condenser.z}")

    print('-----Micromanipulator-----')

    patchstar = Patchstar('COM7')

    patchstar.coordinates()
    print(f"Micromanipulator values X = {patchstar.x}, Y = {patchstar.y}, Z = {patchstar.z}, A = {patchstar.a}")

    print('-----Camera-----')

    cam = PVCAM()

    #GUI
    root = App(slicescope, condenser, patchstar, cam)
    root.mainloop()

    #Close and disconnect all equipment

    cam.disconnect()

    patchstar.close()

    condenser.close()

    slicescope.close()


if __name__ == '__main__':

    main()