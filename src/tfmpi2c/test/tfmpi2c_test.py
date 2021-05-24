'''=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# File Name: tfmpi2c_test.py
# Inception: 03 MAY 2021
# Developer: Bud Ryerson
# Version:   0.0.7
# Last work: 21 MAY 2021

# Description: Python script to test the Benewake TFMini Plus
# time-of-flight Lidar ranging sensor in I2C mode
# using the 'tfmpi2c' module in development.

# Default settings for the TFMini-Plus is a 0x10 I2C address
# and a 100Hz measurement frame rate. The device will return
# three measurement datums as commanded:
#   Distance in centimeters,
#   Signal strength in arbitrary units and
#   Temperature encoded for degrees centigrade

# 'begin( port, address)' can be used to reset the default
# I2C port number and device address.  It tests existence
# of the port and address, and returns a boolean result.

# 'getData()' commands device to return a data-frame and
# sets variables for distance (dist) in centimeters, signal
# strength (flux) and temperature (temp) in Centigrade.
# Returns a boolean result and sets a one byte status code.

# 'sendCommand( cmnd, param)' sends a command and a parameter to the device.
# Returns a boolean result and sets a one byte status code.
# Commands are defined in the module's list of commands.
# Parameters can be entered directly (115200, 250, etc) but for
# safety, they should be chosen from the module's defined lists.

# NOTE:
#   I2C(1) is default RPi I2C port, used by real-time clock
#   Other I2C Ports are initialized in the 'boot/config.txt' file
#   I2C(0) = GPIO0 Pin 27 SDA, GPIO1 Pin 28 SCL
#   I2C(4) = GPIO8 Pin 24 SDA, GPIO9 Pin 21 SCL
#
# Press Ctrl-C to break the loop
#
# 'tmfpi2c' Module does not work in Windows because required
# 'smbus' module only works in Linux/Raspian/MacOS
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-'''

# Skip a line and say 'Hello!'
print( "\n\rTFMPlus RPi I2C Example - 21MAY2021")

import time
import sys
import tfmpi2c as tfmP   # Import `tfmpi2c` module v0.0.7
from tfmpi2c import *    # and also import all definitions

# - - - -  Set and Test I2C communication  - - - -
#  I2C(4) is default I2CPort for this module
#  0x10 is the default I2CAddr for TFMini-Plus
#  This function is needed only to change I2CPort
#  and/or I2CAddr, and to test those settings.
# - - - - - - - - - - - - - - - - - - - - - - - - -
#I2CPort = 0     # I2C(0), /dev/i2c-0, GPIO 0/1, pins 27/28
I2CPort = 4     # I2C(4), /dev/i2c-4, GPIO 8/9, pins 24/21
I2CAddr = 0x10  # Device address in Hex, Decimal 16
print( "I2C mode: ", end='')
if(tfmP.begin( I2CPort, I2CAddr)):
    print( "ready")
else:
    print( "not ready")
    sys.exit()   #  quit the program if I2C bus not ready
# - - - - - - - - - - - - - - - - - - - - - - - -'''

# - - - - - -  Miscellaneous commands  - - - - - - -
#      None of these are strictly necessary.
#        Many more commands are available.
# - - - - - - - - - - - - - - - - - - - - - - - - -
#
# - - Perform a system reset - - - - - - - -
print( "System reset: ", end= '')
if( tfmP.sendCommand( SYSTEM_RESET, 0)):
    print( "passed")
else:
    tfmP.printReply()
time.sleep(0.5)  # allow 500ms for reset to complete
# - - - - - - - - - - - - - - - - - - - - - - - -
#
# - - Get and Display the firmware version - - - - - - -
print( "Firmware version: ", end= '')
if( tfmP.sendCommand( OBTAIN_FIRMWARE_VERSION, 0)):
    print( tfmP.version)
else:
    tfmP.printReply()
# - - - - - - - - - - - - - - - - - - - - - - - -
#
# - - Set the data-frame rate to 20Hz - - - - - - - -
print( "Data-Frame rate: ", end= '')
if( tfmP.sendCommand( SET_FRAME_RATE, FRAME_20)):
    print( str(FRAME_20) + 'Hz')
else:
    tfmP.printReply()
# - - - - - - - - - - - - - - - - - - - - - - - -
time.sleep(0.5)     # Wait half a second.
#
# - - - - - -  miscellaneous commands ends here  - - - - - - -


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - -  the main program loop begins here  - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Program will report an error, wait two seconds, restart
# up to three times and then quit.
#
attempts = 0
#
while attempts < 3:
    try:
        while True:
            time.sleep(0.05)   # Loop delay 50ms to match the 20Hz data-frame rate
            # Use the 'getData' function to get data from device
            if( tfmP.getData()):
                print( f" Dist: {tfmP.dist:{3}}cm ", end= '')   # display distance,
                print( " | ", end= '')
                print( f"Flux: {tfmP.flux:{5}d} ",   end= '')   # display signal strength/quality,
                print( " | ", end= '')
                print( f"Temp: {tfmP.temp:{2}}°C",  )   # display temperature,
            else:                  # If the command fails...
                tfmP.printFrame()  # display the error and HEX data
    #
    except KeyboardInterrupt:
        print( 'Keyboard Interrupt')
        break
    #
    except: # catch all other exceptions
        eType = sys.exc_info()[0]  # return exception type
        print( eType)
        attempts += 1
        print( "Attempts: " + str(attempts))
        time.sleep(2.0)  # wait two seconds and retry
#
print( "That's all folks!") # Say "Goodbye!"
sys.exit()                  # Clean up the OS and exit.
#
# - - - - - -  the main program sequence ends here  - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
