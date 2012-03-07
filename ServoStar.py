"""
    Copyright 2012 Stanton T. Cady
    Copyright 2012 Hannah Hasken
    
    ServoStar_python  v0.1.7 -- March 7, 2012
    
    This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
	
    This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
"""

import sys
import serial
import time
from datetime import datetime
import operator
import itertools
import _winreg as winreg
import msvcrt
import os
import math

global quiet
quiet = False
global crLast
crLast = False
global dynoMode
dynoMode = -1
global CURRENT_SCALING_FACTOR
CURRENT_SCALING_FACTOR = 50 # drive command/amps
global TORQUE_CONSTANT
TORQUE_CONSTANT = 1.3975964 # Amps/Nm
global CONSTANT_COEFFICIENT
CONSTANT_COEFFICIENT = 0.1262042969
global LINEAR_COEFFICIENT
LINEAR_COEFFICIENT = 0.0000535541
global QUADRATIC_COEFFICIENT
QUADRATIC_COEFFICIENT = -0.0000000074
global CUT_OUT_SPEED
CUT_OUT_SPEED = 1350
global torqueLimit
torqueLimit = 6

def sendCommand(ser,command,l = None):
    """ 
        Send serial command byte by byte to dyno.
        
        Arguments:
        ser -- (required) the serial port object to use
        command -- (required) a string containing the command to be sent
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        True -- everything worked
        False -- otherwise
        
    """
	command = command + chr(0x0D)
	try:
		if(ser.isOpen()):
            # Clear the serial output buffer.
			ser.flushOutput
			ser.flushInput
            # Clear anything that is in waiting.
			ser.read(ser.inWaiting())
            # Do for each character of command string.
			for char in command:
                # Send character out serial port.
				if(ser.write(char) > 0):
					try:
                        # Read a byte from the serial port.
						c = ser.read()
						if(c != char):
							printStdOut("Byte received (" + str(ord(c)) + ") does not match byte sent (" + str(ord(char)) + ")",l)
							return False
					except ser.SerialException:
						printStdOut("Serial exception.",l)
						return False
				else:
					printStdOut("Serial port error.",l)
					return False
			return True
		else:
			printStdOut("Serial connection not open.",l)
	except AttributeError:
		printStdOut("Serial connection never opened.",l)
	return False

def verifyCommand(ser,l = None):
    """ 
        Verify serial command to dyno.
        
        Arguments:
        ser -- (required) the serial port object to use
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        True -- everything worked
        False -- serial connection not available or unexpected character read
        val -- err integer number returned by dyno if an error occurred
        
    """
	try:
		if(ser.isOpen()):
            # Check for new line character.
			if(ser.read() == chr(0x0A)):
                # Read a byte from the serial port.
				c = ser.read()
				if(c == '-'):
					if(ser.read() == '-'):
						if(ser.read() == '>'):
							return True
                # Check for bell character.
				elif(c == chr(0x07)):
					val = str()
					while True:
						b = ser.read()
                        # Check for carriage return character.
						if(b != chr(0x0D)):
							val = val + b
						else:
							if(ser.read() == chr(0x0A)):
								if(ser.read() == '-'):
									if(ser.read() == '-'):
										if(ser.read() == '>'):
											if val[:3] == 'ERR':
												printStdOut val
												# Return err number if there is an error.
												return int(val[4:6])
											break
		else:
			printStdOut("Serial connection not open.",l)
	except AttributeError:
	printStdOut("Serial connection never opened.",l)
	return False

def sendWriteCommand(ser,command,l = None):
    """ 
        Call verifyCommand function if a serial command was sent successfully to dyno.
        
        Arguments:
        ser -- (required) the serial port object to use
        command -- (required) a string containing the command to be sent
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        False -- call to sendCommand was unsuccessful
        result of verifyCommand otherwise
        
    """
	if(sendCommand(ser,command,l)):
		return verifyCommand(ser,l)
    return False

def sendReadCommand(ser,command,l = None):
    """ 
        Send read command to dyno.
        
        Arguments:
        ser -- (required) the serial port object to use
        command -- (required) a string containing the command to be sent
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        False -- serial connection not available or unexpected character read
        val -- the string that was read
        
    """
	try:
		if(ser.isOpen()):
			if(sendCommand(ser,command,l)):
                # Check if new line character.
				if(ser.read() == chr(0x0A)):
					val = str()
                    # Read a byte from the serial port until a carriage return is read.
					while True:
						b = ser.read()
                        # Check if carriage return character.
						if(b != chr(0x0D)):
							val = val + b
						else:
							if(ser.read() == chr(0x0A)):
								if(ser.read() == '-'):
									if(ser.read() == '-'):
										if(ser.read() == '>'):
											return val                                    
			else:
				printStdOut("Read command could not be sent.",l)
		else:
			printStdOut("Serial connection not open.",l)
	except AttributeError:
		printStdOut("Serial connection never opened.",l)
	return False	

def checkActive(ser,l = None):
    """ 
        Check to see if dyno is active by calling sendReadCommand.
        
        Arguments:
        ser -- (required) the serial port object to use
        command -- (required) a string containing the command to be sent
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        True -- call to sendReadCommand was successful
        False -- call to sendReadCommand was unsuccessful
        rsp -- result of sendReadCommand when call returned unexpected result
        
    """
	rsp = sendReadCommand(ser,'active',l)
    if(rsp == false):
        return False
	elif(int(rsp) == 1):
		return True
	return rsp

def driveOk(ser,l = None):
    """ 
        Check to see if dyno drive is okay by calling sendReadCommand.
        
        Arguments:
        ser -- (required) the serial port object to use
        command -- (required) a string containing the command to be sent
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        True -- call to sendReadCommand was successful
        False -- call to sendReadCommand was unsuccessful
        rsp -- result of sendReadCommand when call returned unexpected result
        
    """
	rsp = sendReadCommand(ser,'driveok',l)
    if(rsp == false):
        return False
	elif(int(rsp) == 1):
		return True
	return rsp

def driveReady(ser,l = None):
    """ 
        Check to see if dyno drive is ready by calling sendReadCommand.
        
        Arguments:
        ser -- (required) the serial port object to use
        command -- (required) a string containing the command to be sent
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        True -- call to sendReadCommand was successful
        False -- call to sendReadCommand was unsuccessful
        rsp -- result of sendReadCommand when call returned unexpected result
        
    """
	rsp = sendReadCommand(ser,'ready',l)
    if(rsp == false):
        return False
	elif(int(rsp) == 1):
		return True
	return rsp

def getVelocity(ser,l = None):
    """ 
        Get velocity from dyno by calling sendReadCommand.
        
        Arguments:
        ser -- (required) the serial port object to use
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        False -- call to sendReadCommand was unsuccessful
        rsp -- result of sendReadCommand as an integer when call was successful
        
    """
    rsp = sendReadCommand(ser,'v',l)
    if(rsp == false):
        return False
    return int(rsp)

def getCurrent(ser,l = None):
    """ 
        Get current from dyno by calling sendReadCommand.
        
        Arguments:
        ser -- (required) the serial port object to use
        command -- (required) a string containing the command to be sent
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        False -- call to sendReadCommand was unsuccessful
        rsp -- result of sendReadCommand when call was successful
        
    """
    rsp = sendReadCommand(ser,'i',l)
    if(rsp == false):
        return False
    return rsp

def getTorque(ser,l = None):
    """ 
        Get torque by using the current from dyno.
        
        Arguments:
        ser -- (required) the serial port object to use
        command -- (required) a string containing the command to be sent
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        False -- call to getCurrent was unsuccessful
        rsp -- calculated torque value if call to getCurrent was successful
        
    """
	global CURRENT_SCALING_FACTOR
	global TORQUE_CONSTANT
    rsp = getCurrent(ser,l)
    if(rsp == false):
        return False
	return float(rsp)/(CURRENT_SCALING_FACTOR*TORQUE_CONSTANT)

def enableDrive(ser,l = None):
    """ 
        Enable dyno drive by calling sendWriteCommand.
        
        Arguments:
        ser -- (required) the serial port object to use
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        True -- call to sendWriteCommand was successful
        rsp -- otherwise
        
    """
	rsp = sendWriteCommand(ser,'en',l)
	if(rsp == True):
		printStdOut("Drive enabled successfully.",l)
		return True
	printStdOut("There was an error enabling the drive.",l)
	return rsp

def disableDrive(ser,l = None):
    """ 
        Disable dyno drive by calling sendWriteCommand.
        
        Arguments:
        ser -- (required) the serial port object to use
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        True -- call to sendWriteCommand was successful
        False -- otherwise
        
    """
	rsp = sendWriteCommand(ser,'dis',l)
	if(rsp == True):
		printStdOut("Drive disabled succesfully.",l)
		return True
	printStdOut("There was an error disabling the drive.",l)
	return rsp

def setOpmode(ser,mode,l = None):
    """ 
        Set operation mode as velocity (1) or torque (2) by calling sendWriteCommand.
        
        Arguments:
        ser -- (required) the serial port object to use
        mode -- (required) the operation mode to put dyno in
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        True -- successfully changed opmode
        False -- invalid opmode sent
        rsp -- call to sendWriteCommand was unsuccessful
        
    """
	if(mode == 1 or mode == 2):
		rsp = sendWriteCommand(ser,'opmode=' + str(mode),l)
		if(rsp == True):
			printStdOut("Opmode changed successfully.",l)
			return True
		else:
			printStdOut,("There was an error setting the opmode.",l)
			return rsp
	else:
		printStdOut("Valid opmodes are 1 or 2.",l)
	return False

def setVelocity(ser,velocity,l = None):
    """ 
        Set dyno velocity by calling sendWriteCommand.
        
        Arguments:
        ser -- (required) the serial port object to use
        velocity -- (required) the desired velocity to be set
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        True -- successfully set velocity
        False -- dyno in invalid mode or invalid velocity command sent
        rsp -- call to sendWriteCommand was unsuccessful
        
    """
	if dynoMode == 1:
		try:
			if (float(velocity) >= 0 and float(velocity) <= 1300):
				rsp = sendWriteCommand(ser,'v=' + str(velocity),l)
				if(rsp == True):
					printStdOut("Velocity command sent successfully.",l)
					return True
				else:
					printStdOut("There was an error commanding the specified velocity.",l)
					return rsp
			else:
				printStdOut("Velocity must be between 0 and 1300 RPM.",l)
		except ValueError:
			printStdOut("Invalid velocity command.",l)
    else:
        printStdOut("Dynamometer is in an unknown mode.",l)
	return False

def setTorque(ser,torque,l = None,quiet = False):
    """ 
        Set dyno torque by converting to current then calling sendWriteCommand.
        
        Arguments:
        ser -- (required) the serial port object to use
        torque -- (required) the desired torque to be set
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        True -- successfully set torque
        False -- dyno in invalid mode or invalid torque command sent
        rsp -- call to sendWriteCommand was unsuccessful
        
    """
	global CURRENT_SCALING_FACTOR
	global TORQUE_CONSTANT
	if dynoMode == 2:
		try:
			if(float(torque) >= 0 and float(torque) <= 6):
				# Convert to current as the drive torque command is actually a current command.
				i = str(int(round(float(torque)*CURRENT_SCALING_FACTOR*TORQUE_CONSTANT,0)))
				rsp = sendWriteCommand(ser,'t=' + i,l)
				if(rsp == True):
                    printStdOut("Te: %0.2f Nm" % (t),l) if not quiet else ''
					return True
				else:
					printStdOut("There was an error commanding the specified torque.",l)
					return rsp
			else:
				printStdOut("Torque must be between 0 and 6 N-m.",l)
		except ValueError:
			printStdOut("Invalid torque command.",l)
	else:
		printStdOut("Dynamometer is in an unknown mode.",l)
	return False

def computeLossTorque(v):
    """
        Convert current loss to torque loss.
        
        Arguments:
        v -- (required)
        
        Returns:
        modified result of computeLossCurrent
        
    """
	global TORQUE_CONSTANT
	return computeLossCurrent(v)/TORQUE_CONSTANT;

def computeLossCurrent(v):
    """
        Calculate amount of current lost due to velocity.
        
        Arguments:
        v -- (required)
        
        Returns:
        lost current
        
    """
	global CONSTANT_COEFFICIENT
	global LINEAR_COEFFICIENT
	global QUADRATIC_COEFFICIENT
	return CONSTANT_COEFFICIENT + v*LINEAR_COEFFICIENT + math.pow(v,2)*QUADRATIC_COEFFICIENT

def setCurrentLimit(ser,limit,quiet = true,l = None):
    """
        Send command to dyno to set current limit.
        
        Arguments:
        ser -- (required) the serial port object to use
        limit -- (required) desired current limit
        quiet -- (optional) optionally print to StdOut
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        True -- current limit successfully set
        rsp -- call to sendWriteCommand was unsuccessful
        
    """
	rsp = sendWriteCommand(ser,'ilim=' + str(limit),l)
	if(rsp == True):
		printStdOut("Current limit successfully set.",l) if not quiet else sys.stdout.write('')
        return True
	printStdOut("There was an error setting the current limit.",l) if not quiet else sys.stdout.write('')
	return rsp

def setTorqueLimit(ser,limit,l = None):
    """
        Send command to dyno to set torque limit.
        
        Arguments:
        ser -- (required) the serial port object to use
        limit -- (required) desired torque limit
        l -- lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        True -- torque limit successfully set
        rsp -- call to setCurrentLimit was unsuccessful
        
    """
	rsp = setCurrentLimit(ser,int(round(float(limit)*CURRENT_SCALING_FACTOR*TORQUE_CONSTANT)),False,l)
	if(rsp == True):
		printStdOut("Torque limit successfully set.",l)
		return True
	printStdOut("There was an error setting the torque limit.",l)
	return rsp

def openSerial(port = None, baud = None,l = None):
    """
        Open and connect to serial port.
        
        Arguments:
        port -- (optional)
        baud -- (optional)
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        ser -- the serial port object to use
        False -- unable to open serial port
        
    """
    if port == None:
        port = promptForPort(l)
    if baud == None:
        baud = promptForBaud(l)
	printStdOut("Connecting...",l)
	ser = serial.Serial(int(port),long(baud),timeout=5)
	if (ser.isOpen()):
		ser.flushInput()
		printStdOut("Serial opened successfully.",l)
		return ser
	else:
		printStdOut("Unable to open serial port.",l) 
	return False

def closeSerial(ser,l = None):
    """
        Close and disconnect from serial port.
        
        Arguments:
        ser -- (required) the serial port object to use
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        True -- serial closed successfully
        False -- unable to close serial port or serial connection never opened
        
    """
	printStdOut("Attempting to disconnect serial connection...",l) 
	try:
		ser.close()
		if(ser.isOpen()):
			printStdOut("Unable to close serial port.",l)
		else:
			printStdOut("Serial closed successfully.",l)
			return True
	except AttributeError:
		printStdOut("Serial connection never opened.",l) 
	return False

def scanSerial(l = None):
    """
        Scan serial
        
        Arguments:
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        next position to be checked
        
    """
	path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
	key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
	for i in itertools.count():
		try:
			val = winreg.EnumValue(key, i)
			portName = str(val[1])
			printStdOut(str(int(portName[-1])-1) + ": " + portName,l)
		except EnvironmentError:
			break
	return i-1

def promptForPort(l = None):
    """
        Obtain desired port to be used.
        
        Arguments:
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        recursively call promptForPort if invalid port selected
        port -- the port to be used
        
    """
	printStdOut "Serial ports available:"
	numPorts = scanSerial()
	port = raw_input("Select a serial port number: ")
	if port == "q":
		sys.exit()
	elif (int(port) < 0):
		printStdOut("Invalid port selected. Try again (or enter q to quit).",l)
		return promptForPort()
	else:
		port = port.strip()
		return port

def promptForBaud(l = None):
    """
        Obtain desired baud rate to be used.
        
        Arguments:
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        recursively call promptForBaud if invalid baud rate selected
        baud -- the baud rate to be used
        
    """
    availableBaud = ['9600','19200']
	printStdOut("Baud rates available:")
	for baud in availableBaud:
		printStdOut(baud,l)
	baud = raw_input("Select a baud rate: ")
	if baud == "q":
		sys.exit()
	elif not baud in availableBaud:
		printStdOut("Inavlid baud rate. Try again (or enter q to quit).",l)
		return promptForBaud()
	else:
		return baud

def killSystem(dyno = None,l = None,exit = True):
    """
        Disable dyno drive.
        
        Arguments:
        dyno -- (optional) dyno object
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        exit -- (optional) option to exit system
        
    """
	if dyno != None and dyno.isOpen():
		# Attempt to disable drive until successful.
		try:
			while disableDrive(dyno,l) != True:
				time.sleep(0.1)
		except KeyboardInterrupt:
			sys.exit()
		closeSerial(dyno,l)
	if exit == True:
		sys.exit()

def enableDyno(ser, mode = -1, torqueLimit = 6, testTorque = True, testVelocity = True, l = None):
    """
        Enable dyno drive in velocity or torque mode.
        
        Arguments:
        ser -- (required) the serial port object to be used
        mode -- (optional) 1 for velocity mode, 2 for torque mode, or -1 to prompt for desired mode
        torqueLimit -- (optional) maximum torque value
        testTorque -- (optional)
        testVelocity -- (optional)
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        rsp -- result of trying to enable a dyno mode
        recursivly call enableDyno if easily fixable error found
        False -- invalid mode entered
        
    """
	global dynoMode
	# Ask for mode if none is given.
	if mode == -1:
        if l != None:
            l.acquire()
		mode = raw_input("Which mode should the dyno be placed in (1) velocity mode or (2) torque mode? ")
        if l != None:
            l.release()
		mode = int(mode)
	if mode == 1:
        # Attempt to enable velocity mode.
        rsp = enableVelocityMode(ser,testVelocity,l)
        if rsp == True:
            dynoMode = 1
		return rsp
	elif mode == 2:
		# Attempt to enable torque mode.
		rsp = enableTorqueMode(ser,torqueLimit,testTorque,l,quiet)
		if rsp == True:
			dynoMode = 2
		# Look for easily fixable errors.
		elif rsp == 5:
			# A response code of 5 can usually be fixed by trying again.
			time.sleep(0.1)
			return enableDyno(ser,mode,torqueLimit,testTorque,testVelocity,l)
		elif rsp == 23:
			disableDrive(ser,l)
			return enableDyno(ser,mode,torqueLimit,testTorque,testVelocity,l)
		return rsp
	else:
		printStdOut("Inavlid mode.",l)
	return False

def enableVelocityMode(ser, testVelocity = True, l = None):
    """
        Enable dyno for velocity commands in velocity mode after disabling dyno drive.
        
        Arguments:
        ser -- (required) the serial port object to be used
        testVelocity -- (optional) option for testing drive for a given velocity
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        True -- drive ready for velocity commands
        rsp -- error enabling velocity mode
        
    """
	rsp = disableDrive(ser,l)
	if(rsp == True):
		time.sleep(0.1)
		rsp = setOpmode(ser,1,l)
		if(rsp == True):
			time.sleep(0.1)
			rsp = enableDrive(ser,l)
			if(rsp == True):
				active = checkActive(ser,l)
				time.sleep(0.1)
				driveok = driveOk(ser,l)
				if active == True and driveok == True:
					printStdOut("The drive should be ready to accept velocity commands.",l)
					if testVelocity == True:
						q = raw_input("Shall I test the drive? (y/n) ")
						if q == 'y':
							testVelocityMode(ser,None,l)
					return True
				else:
					rsp = "Active response: " + str(active) + "; Driveok response: " + str(driveok)
	printStdOut("There was an error enabling velocity mode.",l)
	return rsp

def enableTorqueMode(ser, torqueLimit = None, testTorque = True,l = None,quiet = False):
    """
        Enable dyno for torque commands in torque mode after disabling dyno drive.
        
        Arguments:
        ser -- (required) the serial port object to be used
        torqueLimit -- (optional) maximum amount of torque to be used
        testTorque -- (optional) option for testing drive for a given torque
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        True -- drive ready for torque commands
        rsp -- error enabling torque mode
        
    """
	rsp = disableDrive(ser,l)
	if(rsp == True):
		time.sleep(0.1)
		rsp = setOpmode(ser,2,l)
		if(rsp == True):
			time.sleep(0.1)
			rsp = enableDrive(ser,l)
			if(rsp == True):
				if(torqueLimit != None):
					time.sleep(0.1)
					if(setTorqueLimit(ser,torqueLimit == True,l,quiet)):
						printStdOut("The torque limit has been set.",l)
				active = checkActive(ser,l)
				time.sleep(0.1)
				driveok = driveOk(ser,l)
				if active == True and driveok == True:
					printStdOut("The drive should be ready to accept torque commands.",l)
					if testTorque == True:
						q = raw_input("Shall I test the drive? (y/n) ")
						if q == 'y':
							testTorqueMode(ser,None,l,quiet)
					return True
				else:
					rsp = "Active response: " + str(active) + "; Driveok response: " + str(driveok)
	printStdOut("There was an error enabling torque mode.",l)
	return rsp

def testTorqueMode(ser, t = None, l = None, quiet = False):
    """
        Get torque and check if it is valid and does not cause the system to fail.
        
        Arguments:
        ser -- (required) the serial port object to be used
        t -- (optional) the torque to test
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
    """
	if t == None:
		t = raw_input(" What torque would you like to apply for the test? ")
		t = float(t)
	if t > 1:
		printStdOut("That seems a little high for a simple test. I recommend using a torque at or below 1 Nm.",l)
		testTorqueMode(ser,None,l,quiet)
	rsp = setTorque(ser,t,l,quiet)
	if rsp == True:
		printStdOut("Applying test torque for 10 seconds...",l)
		time.sleep(10)
		if(setTorque(ser,0,l,quiet) == True):
			printStdOut("Test completed successfully.",l)

def testVelocityMode(ser, v = None,l = None):
    """
        Get velocity and check if it is valid and does not cause the system to fail.
        
        Arguments:
        ser -- (required) the serial port object to be used
        v -- (optional) the velocity to test
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
    """
	if v == None:
		v = raw_input(" What velocity would you like to apply for the test? ")
		v = float(v)
	if v > 1000:
		printStdOut("That seems a little high for a simple test. I recommend using a velocity at or below 1000 rpm.",l)
		testVelocityMode(ser,None,l)
	rsp = setVelocity(ser,v,l)
	if rsp == True:
		printStdOut("Applying test velocity for 10 seconds...",l)
		time.sleep(10)
		if(setVelocity(ser,0,l) == True):
			printStdOut("Test completed successfully.",l)

def checkSpeed(ser,t,l = None):
    """
        Check that the given velocity is valid.
        
        Arguments:
        ser -- (required) the serial port object to be used
        t -- (reqired) 
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        v -- the given, valid speed
        killSystem if high invalid speed given
        
    """
	v = float(getVelocity(ser,l))
	if v > CUT_OUT_SPEED:
		printStdOut("Speed of " + str(v) + " rpm exceeds cut out speed of " + str(CUT_OUT_SPEED) + " rpm.  Killing system.",l)
		killSystem(ser,l)
	else:
		return v

def setupDynoSerial(port = None, baud = None, l = None):
    """
        Obtain port and baud parameters and attempt to open a serial connection to dyno.
        
        Arguments:
        port -- (optional) the port number to use
        baud -- (optional) the baud rate to use
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
        Returns:
        call to openSerial
        
    """
	# Check if parameters for serial port and baud rate have been passed in.
	if port == None or baud == None:
		# Missing port or baud so ask for them.
		q = raw_input("Port and/or baud rate not provided. Shall I use the default settings? (y/n): ")
		if port == None and q == 'y':
				port = 0
		if baud == None and q == 'y':
				baud = 9600
	printStdOut("Attempting to setup the dynamometer.",l)
	# Attempt to open a serial connection to the dyno.
	return openSerial(port,baud,l)

def startDyno(ser, initialTorque = None, vref = None, initialDelay = None, l = None, quiet = False):
    """
        Attempt to start the dyno with given velocity and torque values.
        
        Arguments:
        ser -- (required) the serial port object to be used
        initialTorque -- (optional) how much torque to initially apply
        vref -- (optional) velocity to set dyno to in rpm
        initialDelay -- (optional) how long to let the drive accelerate in seconds
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        
    """
	if vref == None:
		vref = 1000
    #initialSpeed = int(raw_input("What velocity in rpm would like to use? "))
	if vref < 1300:
		printStdOut("Starting dyno to " + str(vref) + " rpm.",l)
		if initialTorque == None:
			t = 0
		else:
			t = initialTorque
			if initialDelay == None:
				initialDelay = 15
			if initialDelay > 0:
				setTorque(ser,t,l,quiet)
				samplePeriod = 0.2
				try:
					for i in range(int(initialDelay/samplePeriod)):
						v = checkSpeed(ser,t,l)
						printStdOut("Letting drive accelerate for %d seconds.  Current speed: %d rpm" % (initialDelay,v),True,l)
						time.sleep(samplePeriod)
					sys.stdout.write("\n")
					return True
				except KeyboardInterrupt:
					killSystem(ser,l)
	else:
		printStdOut("Please choose a speed below 1300 rpm.",l)
		startDyno(ser,initialTorque,vref,initialDelay,l,quiet)

def printStdOut(msg, l = None, cr = False):
    """
        Print using StdOut for option of using multiprocessing
        
        Arguments:
        msg -- (required) the message to be printed
        l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
        cr -- (optional) print a carriage return after the message if True
        
    """
	global crLast
    if l != None:
        l.acquire()
	if crLast == True and cr == False:
		crLast = False
		sys.stdout.write("\n")
	if cr == True:
		crLast = True
		msg = "\r" + msg
	sys.stdout.write(msg)
	if cr == False:
		sys.stdout.write("\n")
    if l != None:
        l.release()