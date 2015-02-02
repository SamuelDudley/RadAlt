#!/usr/bin/env python
"""
Script to read 485 framed CAN messages from Smartmicro Micro Radar Altimeter

Python code is based on C code supplied from manufacturer

TODO: 
 - Make classes
 - Write data out as a mavlink stream (MAVLink_distance_sensor_message)

Samuel Dudley - Jan 2015
"""

import serial
from pymavlink import mavutil


def parseCanbusFrames(canbusFramesHolders):
    for canbusFrames in canbusFramesHolders:
    
        bytesInCanbusFrames = len(canbusFrames)
        currentByte = 0
        
        
        #there will be n canbus frames in the canbusFrames data stream
        while currentByte < bytesInCanbusFrames:
            #read the first three bytes from the canbusFrames data stream
            canbusIdMSB = ord(canbusFrames[currentByte])
            canbusIdLSB = ord(canbusFrames[currentByte+1])
            canbusFrameLength = ord(canbusFrames[currentByte+2])
            
            canbusId = (canbusIdMSB <<8) + canbusIdLSB
            
            canbusPayload = canbusFrames[currentByte+3:currentByte+canbusFrameLength+3]
            currentByte += canbusFrameLength+3# += len(CANPAYLOAD)+3
            
            
            if canbusId == 1872: #sensor data msg
                #read bytes and shift to report range to the object
                rangeToObject = ord(canbusPayload[7]) << 0
                rangeToObject += ord(canbusPayload[6]) << 8
                rangeToObject += ord(canbusPayload[5]) << 16
                rangeToObject *= 0.01
                print 'alt', rangeToObject
                
                #read bytes and shift to report speed the object is approaching
                speedApproachObject = ord(canbusPayload[4]) << 0
                speedApproachObject += ord(canbusPayload[3]) << 8
                speedApproachObject += ord(canbusPayload[2]) << 16
                speedApproachObject -= 0x800000 #offset to allow neg. values
                speedApproachObject *= 0.01
                print 'spd', speedApproachObject
                
                #read bytes and obtain valid / error flags
                flags = ord(canbusPayload[1])
                
                ALT_VALID_BIT_MASK =   0b00000001
                PWR_ERROR_BIT_MASK =   0b00000010
                TEMP_ERROR_BIT_MASK =  0b00000100
                PLL_ERROR_BIT_MASK =   0b00001000
                RAM_ERROR_BIT_MASK =   0b00010000
                FLASH_ERROR_BIT_MASK = 0b00100000
                
                valid = flags & ALT_VALID_BIT_MASK
                pwr = flags & PWR_ERROR_BIT_MASK
                temp = flags & TEMP_ERROR_BIT_MASK
                pll = flags & PLL_ERROR_BIT_MASK
                ram = flags & RAM_ERROR_BIT_MASK
                flash = flags & FLASH_ERROR_BIT_MASK
                
                print 'flags', valid, pwr, temp, pll, ram, flash
    
                #the remainder of the payload (canbusPayload[0]) is unused.
                
                
        
    
    
def parseRawBytes(radAltData):
    startSequence = [0xCA, 0xCB, 0xCC, 0xCD] #hex bytes
    endSequence = [0xEA, 0xEB, 0xEC, 0xED] #hex bytes
    
    startSequenceLength = len(startSequence)
    endSequenceLength = len(endSequence)
    
    startSequenceIndex = 0
    endSequenceIndex = 0
    
    hasStarted = False

    serialFrames = [] #list to hold possibly valid RS485 frames
    radAltDataPayloadBytes = '' #payload bytes extracted from the raw byte stream

    for byte in radAltData:
        if ord(byte) == startSequence[startSequenceIndex]:
            startSequenceIndex += 1
            if startSequenceIndex >= startSequenceLength:
                startSequenceIndex = 0
                
                if hasStarted: #already have a start seq open
                    radAltDataPayloadBytes = '' #clear the current data and start again
                else:
                    hasStarted = True
        else:
            startSequenceIndex = 0
            
            
            
        if ord(byte) == endSequence[endSequenceIndex]:
            endSequenceIndex += 1
            
            if endSequenceIndex >= len(endSequence):
                endSequenceIndex = 0
                if hasStarted:
                    serialFrames.append(radAltDataPayloadBytes[1:-3])
                    #The final starting seq. byte and three end seq. bytes need to be removed
                    
                hasStarted = False #the message has ended
                radAltDataPayloadBytes = '' #clear the current data and start again
                
        else:
            endSequenceIndex = 0
            
            
            
        if hasStarted: #if the starting 
            radAltDataPayloadBytes += byte
            
    return serialFrames, radAltDataPayloadBytes
        

def parseSerialFrames(serialFrames):
    canbusFramesHolders = []
    for serialFrame in serialFrames:
        checksum = serialFrame[-1] #final byte of the frame
        dataBytes = serialFrame[:-1] #all but the checksum byte
        
        xorbytes = 0
        
        for byte in dataBytes:
           xorbytes = xorbytes ^ ord(byte) 
        
        if xorbytes == ord(checksum):
            canbusFramesHolders.append(dataBytes)
              
    return canbusFramesHolders

        

#open serial port:
ser = serial.Serial(
    port='COM1',
    baudrate=115200, 
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=5
    )


radAltData = ''

while 1:
    radAltData += ser.read(100)
    serialFrames, radAltData = parseRawBytes(radAltData)
    canbusFramesHolders = parseSerialFrames(serialFrames)
    parseCanbusFrames(canbusFramesHolders)



        
