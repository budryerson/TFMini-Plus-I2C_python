'''=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
 # Package:   tfmpi2c
 # Inception: 22 MAY 2021
 # Described: Python package for the Benewake TFMini-Plus Lidar
 #            sensor using the I2C communication mode interface.
 #            TFMini-Plus packages are not compatible with the TFMini.
 # Developer: Bud Ryerson
 # Version:   0.0.6
 # Last work: 15 MAY 2021
 #            Opened and closed I2C port with every I/O call
 #            19 MAY 2021 - Corrected FRAME_5 value
 #
 # Default settings for the TFMini-Plus in I2C mode:
 # I2C Address: 0x10; Data frame rate: 100Hz
 #
 # 'begin( portNumber, devAddress)' passes an I2C port
 #  number and device address to the function.
 #  Returns 'True' if port exists and is writable.
 #  Also sets a public one byte status code, defined below.
 #
 # `getData()` gets a 9 byte data frame from the device
 #  and sets the value of three variables:
 #  • `dist` = distance in centimeters,
 #  • `flux` = signal strength, also certain error codes
 #  • `temp` = internal chip temperature in degrees centigrade
 #  Returns 'True' if completed without error.
 #  Also sets a one byte `status` code.
 #
 # `sendCommand( cmnd, param)` sends appropriate command
 #  and parameter values, returns a boolean success value
 #  and sets an explanatory, public, one byte `status` code.
 #  Commands are chosen from the module's defined commands.
 #  Parameter values can be entered directly (115200, 250, etc)
 #  but for safety, should be chosen from the module's
 #  defined values. Incorrect values can render the device
 #  permanently uncommunicative.
 #
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-'''

import time
from smbus import SMBus

status = 0            # error status code
dist =   0            # distance to target
flux =   0            # signal quality or intensity
temp =   0            # internal chip temperature
version = ''          # firmware version number

# Buffer sizes
TFMP_FRAME_SIZE =  9   # Size of one data frame = 9 bytes
TFMP_COMMAND_MAX = 8   # Longest command = 8 bytes
TFMP_REPLY_SIZE =  8   # Longest command reply = 8 bytes

# Buffers
frame: list = []    # received data buffer
reply: list = []    # command reply buffer

# Timeout Limits for various functions
TFMP_MAX_READS          = 20   # readData() sets SERIAL error
MAX_BYTES_BEFORE_HEADER = 20   # getData() sets HEADER error
MAX_ATTEMPTS_TO_MEASURE = 20

addr = 0x10           # default TFMini-Plus I2C device address
                      # 16 in decimal, 0x10 in hexadecimal
port = 4              # default Raspberry Pi I2C port number
                      # 4 = /dev/i2c-4, GPIO 8/9, pins 24/21

def begin( portNumber, devAddress):
    global port, addr
    addr = devAddress         # re-assign device address
    port = portNumber  # re-assign port number
    try:
        bus = SMBus( port)
        bus.open( port)
        bus.write_quick(addr)
        bus.close()
        return True
    except Exception:
        return False

#
# System Error Status Condition
TFMP_READY        =  0  # no error
TFMP_SERIAL       =  1  # serial timeout
TFMP_HEADER       =  2  # no header found
TFMP_CHECKSUM     =  3  # checksum doesn't match
TFMP_TIMEOUT      =  4  # I2C timeout
TFMP_PASS         =  5  # reply from some system commands
TFMP_FAIL         =  6  #           "
TFMP_I2CREAD      =  7
TFMP_I2CWRITE     =  8  # I2C write failure
TFMP_I2CLENGTH    =  9
TFMP_WEAK         = 10  # Signal Strength ≤ 100
TFMP_STRONG       = 11  # Signal Strength saturation
TFMP_FLOOD        = 12  # Ambient Light saturation
TFMP_MEASURE      = 13

'''- - - - - -  TFMini Plus data formats  - - - - - - - - -
  Data Frame format:
  Byte0  Byte1  Byte2   Byte3   Byte4   Byte5   Byte6   Byte7   Byte8
  0x59   0x59   Dist_L  Dist_H  Flux_L  Flux_H  Temp_L  Temp_H  CheckSum_
  Data Frame Header character: Hex 0x59, Decimal 89, or "Y"

  Command format:
  Byte0  Byte1   Byte2   Byte3 to Len-2  Byte Len-1
  0x5A   Length  Cmd ID  Payload if any   Checksum
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - '''

#  Return TRUE/FALSE whether data received without error
#  and set system status to provide more information.
def getData():
    ''' Get frame data from device'''

    # - - - - - - - - - - - - - - - - - - - - - - - - -
    # Step 1 - A little housekeeping
    # - - - - - - - - - - - - - - - - - - - - - - - - -
    # make data and status variables global
    global status, dist, flux, temp, frame
    # clear status variable of any error condition
    status = TFMP_READY;

    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    #  Step 2 - Get data from the device.
    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    #    - It is possible to return data in millimeters, but
    #  resolution is 0.5cm and accuracy is only ±5.0cm.
    #    - smbus protocol is: write_i2c_block_data( addr, cmd, vals)
    #  Slave device does not use 'cmd' so header byte is in 'cmd'
    #  position and rest of command follows as a list of 'vals'.
    #
    #  2.1 Open I2C communication
    bus = SMBus( port)
    #  2.2 Send command to return distance data in centimeters.
    bus.write_i2c_block_data( addr, 0x5a, [0x05, 0x00, 0x01, 0x60])
    #  2.3 Get a frame of data from device
    frame = bus.read_i2c_block_data( addr, 0, TFMP_FRAME_SIZE)
    #  2.4 Close I2C communication
    bus.close()

    '''
    for i in range( TFMP_FRAME_SIZE):
        frame[ i] = bus.read_byte( addr)
    for i in range( len(frame)):
        print( f" {frame[i]:0{2}X}", end='')
    print()
    '''

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    #  Step 3 - Perform a checksum test.
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    #  3.1 Declare and clear the 'chkSum' variable
    chkSum = 0
    #  3.2 Add together all frame bytes but the last.
    for i in range( TFMP_FRAME_SIZE - 1): chkSum += frame[ i]
    #  3.3 Test low-order 'chkSum' byte against last byte of frame
    if( chkSum & 0xff != frame[ TFMP_FRAME_SIZE - 1]):
        status = TFMP_CHECKSUM  # If not same, set error...
        return False            # and return "false."

    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    #  Step 4 - Interpret the frame data
    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # Shift left 8 bits equals multiply by 256
    dist = (frame[3] << 8) + frame[2]
    flux = (frame[5] << 8) + frame[4]
    temp = (frame[7] << 8) + frame[6]
    #  Convert temp code to degrees Celsius.
    temp = ( temp >> 3) - 256
    #  Convert Celsius to degrees Fahrenheit
    #temp = ( temp * 9 / 5) + 32

    #  - - Evaluate Abnormal Data Values - -
    #  Values are from the TFMini-S Product Manual
    #     - If 'flux' or signal strength is less than 100
    #  then set distance to negative value
    if( dist == -1):     status = TFMP_WEAK
    #     - If 'flux' or signal strength is 'saturated'
    elif( flux == -1):   status = TFMP_STRONG
    #  Ambient Light saturation
    elif( dist == -4):   status = TFMP_FLOOD
    #  Data is apparently okay
    else:                status = TFMP_READY

    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    #  Step 5 - Set status and return
    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if( status != TFMP_READY):
        return False;
    else:
        return True;
#
# - - - - - - -  End of getData()  - - - - - - - - - - -


#  = = = = =  SEND A COMMAND TO THE DEVICE  = = = = = = = = = =0
#
#  - - -  Command Codes - - - - -
#  The 'sendCommand()' function expects the command
#  (cmnd) code to be in the the following format:
#  0x     00       00       00       00
#      one byte  command  command   reply
#      payload   number   length    length
OBTAIN_FIRMWARE_VERSION   = 0x00010407   # returns 3 byte firmware version
TRIGGER_DETECTION         = 0x00040400   # frame rate must be set to zero
                                         # returns a 9 byte data frame
SYSTEM_RESET              = 0x00020405   # returns a 1 byte pass/fail (0/1)
RESTORE_FACTORY_SETTINGS  = 0x00100405   #           "
SAVE_SETTINGS             = 0x00110405   # This must follow every command
                                         # that modifies volatile parameters.
                                         # Returns a 1 byte pass/fail (0/1)

SET_FRAME_RATE            = 0x00030606   # Each of these commands return
SET_BAUD_RATE             = 0x00060808   # an echo of the command
STANDARD_FORMAT_CM        = 0x01050505   #           "
PIXHAWK_FORMAT            = 0x02050505   #           "
STANDARD_FORMAT_MM        = 0x06050505   #           "
ENABLE_OUTPUT             = 0x00070505   #           "
DISABLE_OUTPUT            = 0x01070505   #           "
SET_I2C_ADDRESS           = 0x100B0505   #           "

SET_SERIAL_MODE           = 0x000A0500   # default is Serial (UART)
SET_I2C_MODE              = 0x010A0500   # set device as I2C slave

I2C_FORMAT_CM             = 0x01000500   # returns a 9 byte data frame
I2C_FORMAT_MM             = 0x06000500   #           "
#
# - - Command Parameters - - - -
BAUD_9600          = 0x002580   # UART serial baud rate
BAUD_14400         = 0x003840   # expressed in hexadecimal
BAUD_19200         = 0x004B00
BAUD_56000         = 0x00DAC0
BAUD_115200        = 0x01C200
BAUD_460800        = 0x070800
BAUD_921600        = 0x0E1000

FRAME_0            = 0x0000    # internal measurement rate
FRAME_1            = 0x0001    # expressed in hexadecimal
FRAME_2            = 0x0002
FRAME_5            = 0x0005    # set to 0x0003 in prior version
FRAME_10           = 0x000A
FRAME_20           = 0x0014
FRAME_25           = 0x0019
FRAME_50           = 0x0032
FRAME_100          = 0x0064
FRAME_125          = 0x007D
FRAME_200          = 0x00C8
FRAME_250          = 0x00FA
FRAME_500          = 0x01F4
FRAME_1000         = 0x03E8
#
#  Create a proper command byte array, send the command,
#  get a response, and return the status
def sendCommand( cmnd, param):
    ''' Send command and get reply data'''

    # - - - - - - - - - - - - - - - - - - - - - - - - -
    # Step 1 - A little housekeeping
    # - - - - - - - - - - - - - - - - - - - - - - - - -
    # make data and status variables global
    global status, dist, flux, temp, version, reply
    # clear status variable of any error condition
    status = TFMP_READY;
    # - - - - - - - - - - - - - - - - - - - - - - - - -

    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    #  Step 1 - Build a command data array
    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # From 32bit 'cmnd' integer, create a four byte array of:
    # reply length, command length, command number and a one byte parameter
    cmndData = bytearray( cmnd.to_bytes( TFMP_COMMAND_MAX, byteorder = 'little'))
    #
    replyLen = cmndData[ 0]    #  Save the first byte as reply length.
    cmndLen = cmndData[ 1]     #  Save the second byte as command length.
    cmndData[ 0] = 0x5A        #  Set the first byte to the HEADER code.
    #
    if( cmnd == SET_FRAME_RATE):    #  If the command is Set FrameRate...
        cmndData[3:2] = param.to_bytes( 2, byteorder = 'little')     #  add the 2 byte FrameRate parameter.
    elif( cmnd == SET_BAUD_RATE):   #  If the command is Set BaudRate...
        cmndData[3:3] = param.to_bytes( 3, byteorder = 'little')     #  add the 3 byte BaudRate parameter.
    #
    cmndData = cmndData[0:cmndLen]  # re-establish length of command data array
    #
    #  Create a checksum for the last byte of the array
    #  (Tests indicate the presence of the byte is
    #  necessary, but the value is irrelevant.)
    chkSum = 0
    #  Add together all bytes but the last.
    for i in range( cmndLen -1):
        chkSum += cmndData[ i]
    #  and save it as the last byte of command data.
    cmndData[ cmndLen -1] = ( chkSum & 0xFF)
    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    #  Step 2 - Send the command data array to the device
    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    cmndList = list(cmndData)
    bus = SMBus( port)    
    bus.write_i2c_block_data( addr, 0, cmndList)
    bus.close()
    #
    #  If the command does not expect a reply, then we're
    #  finished here. Go home.
    if( replyLen == 0):
        return True
    #  + + + + + + + + + + + + + + + + + + + + + + + + +

    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    #  Step 3 - Get command reply data back from the device.
    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    #  Give device a chance to fill its registers
    time.sleep(0.002)
    #  Read block of data into declared list 'reply'
    bus = SMBus( port)
    reply = bus.read_i2c_block_data( addr, 0, replyLen)
    bus.close()

    '''
    for i in range( replyLen):
        reply[ i] = bus.read_byte( addr)
    for i in range( len(reply)):
        print( f" {reply[i]:0{2}X}", end='')
    print()
    '''

    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    #  Step 4 - Perform a checksum test.
    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    #  Declare and clear the 'chkSum' variable
    chkSum = 0
    #  Add together all bytes but the last.
    for i in range( replyLen -1):
        chkSum += reply[ i]
    #  If the low order byte does not equal the last byte...
    if( ( chkSum & 0xff) != reply[ replyLen - 1]):
        status = TFMP_CHECKSUM  #  ...then set error
        return False            #  and return 'False.'

    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    #  Step 5 - Interpret different command responses.
    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if( cmnd == OBTAIN_FIRMWARE_VERSION):
        version =\
            str( reply[ 5]) + '.' +\
            str( reply[ 4]) + '.' +\
            str( reply[ 3])
    else:
        if( cmnd == SYSTEM_RESET or
            cmnd == RESTORE_FACTORY_SETTINGS or
            cmnd == SAVE_SETTINGS ):
            if( reply[ 3] == 1):    #  If PASS/FAIL byte non-zero
                status = TFMP_FAIL  #  then set status to 'FAIL'
                return False        #  and return 'False'.

    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    #  Step 6 - Set status to 'READY' and return 'True'
    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    status = TFMP_READY
    return True

#  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#  - - - - -    The following are for testing purposes   - - - -
#     They interpret error status codes and display HEX data
#  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#
#  Called by either 'printFrame()' or 'printReply()'
#  Print status condition either 'READY' or error type
def printStatus():
    ''' Print status condition'''
    global status

    print("Status: ", end= '')
    if( status == TFMP_READY):       print( "READY", end= '')
    elif( status == TFMP_SERIAL):    print( "SERIAL", end= '')
    elif( status == TFMP_HEADER):    print( "HEADER", end= '')
    elif( status == TFMP_CHECKSUM):  print( "CHECKSUM", end= '')
    elif( status == TFMP_TIMEOUT):   print( "TIMEOUT", end= '')
    elif( status == TFMP_PASS):      print( "PASS", end= '')
    elif( status == TFMP_FAIL):      print( "FAIL", end= '')
    elif( status == TFMP_I2CREAD):   print( "I2C-READ", end= '')
    elif( status == TFMP_I2CWRITE):  print( "I2C-WRITE", end= '')
    elif( status == TFMP_I2CLENGTH): print( "I2C-LENGTH", end= '')
    elif( status == TFMP_WEAK):      print( "Signal weak", end= '')
    elif( status == TFMP_STRONG):    print( "Signal saturation", end= '')
    elif( status == TFMP_FLOOD):     print( "Ambient light saturation", end= '')
    else:                            print( "OTHER", end= '')
    print()
#
#  Print error type and HEX values
#  of each byte in the data frame
def printFrame():
    '''Print status and frame data'''
    global frame

    printStatus();
    print("Data:", end= '')  # no carriage return
    for i in range( len( frame)):
        #  >>> f"{value:#0{padding}X}"
        # Pad hex number with 0s to length of n characters
        print(f" {frame[ i]:0{2}X}", end='')
    print()
#
#  Print error type and HEX values of
#  each byte in the command response frame.
def printReply():
    '''Print status and reply data'''
    global reply

    printStatus()
    #  Print the Hex value of each byte
    for i in range( len (reply)):
        print(f" {reply[ i]:0{2}X}", end='')
    print()
#
# ====   For test and troubleshooting:  ====
# ====   Format a value as hexadecimal  ====
'''
for i in range( len(cmndData)):
    print( f" {cmndData[i]:0{2}X}", end='')
print()
'''
# f"{value:#0{padding}X}"
# formats 'value' as a hex number padded with
# 0s to the length of 'padding'
#============================================

#  Definitions that need to be exported
__all__ = ['OBTAIN_FIRMWARE_VERSION', 'TRIGGER_DETECTION', 'SYSTEM_RESET',
           'RESTORE_FACTORY_SETTINGS', 'SAVE_SETTINGS', 'SET_FRAME_RATE',
           'SET_BAUD_RATE', 'STANDARD_FORMAT_CM', 'PIXHAWK_FORMAT',
           'STANDARD_FORMAT_MM', 'ENABLE_OUTPUT', 'DISABLE_OUTPUT',
           'SET_I2C_ADDRESS', 'SET_SERIAL_MODE', 'SET_I2C_MODE',
           'I2C_FORMAT_CM', 'I2C_FORMAT_MM',
           'BAUD_9600', 'BAUD_14400', 'BAUD_19200', 'BAUD_56000',
           'BAUD_115200', 'BAUD_460800', 'BAUD_921600',
           'FRAME_0', 'FRAME_1', 'FRAME_2', 'FRAME_5', 'FRAME_10',
           'FRAME_20', 'FRAME_25', 'FRAME_50', 'FRAME_100', 'FRAME_125',
           'FRAME_200', 'FRAME_250', 'FRAME_500', 'FRAME_1000']

# If this module is executed by itself
if __name__ == "__main__":
    print( "tfmpi2c - This Python module supports the Benewake" +\
           " TFMini-Plus Lidar device in I2C mode")
