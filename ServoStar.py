"""
    Copyright 2012 Stanton T. Cady
    Copyright 2012 Hannah Hasken
    
    ServoStar_python  v0.5.5 -- August 17, 2012
    
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
import os
import math

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
    
class dyno:
    """
        The dyno.
        
        Methods:
        dynoSerial (class)
        checkActive
        driveOk
        driveReady
        getVelocity
        getCurrent
        getTorque
        enableDrive
        disableDrive
        __setOpmode
        setVelocity
        setTorque
        computeLossTorque
        computeLossCurrent
        setCurrentLimit
        setTorqueLimit
        killSystem
        setDynoMode
        __enableVelocityMode
        __enableTorqueMode
        testTorqueMode
        testVelocityMode
        checkSpeed
        startDyno
        printStdOut
        
    """
    # Variables
#    __dS
#    __mode
#    __torque
#    __velocity
#    __l
    
    
    def __init__(self, port = None, baud = None, mode = -1, initial = None, tlim = 6, vlim = 1300, quiet = False, verbose = False, l = None):
        self.__l = l
        self.__crLast = False
        self.__mode = -1
        self.__tlim = tlim
        self.__vlim = vlim
        self.__quiet = quiet		# allows/disallows anything printing to standard out
        self.__verbose = verbose	# limits print to standard out to only errors
        self.__dS = dynoSerial(port,baud,l)
        if self.setTorqueLimit(tlim):
            rsp = self.setDynoMode(mode)
            if(rsp == True):
                if(initial != None and self.__mode == 2):
                    self.printStdOut("Commanding initial torque.")
                    self.setTorque(initial)
                elif(initial != None and self.__mode == 0):
                    self.printStdOut("Commanding initial velocity.")
                    self.setVelocity(initial)
                self.printStdOut("Dyno object created successfully.")
            else:
                self.printStdOut("There was an error enabling the drive: " + str(rsp),True)
        else:
            self.printStdOut("Could not set torque limit, aborting drive enable",True)
    
    def setQuiet(quiet):
    	self.__quiet = quiet
            
    def printStdOut(self, msg, force = False, cr = False):
        """
            Print using StdOut for option of using multiprocessing
            
            Arguments:
            msg -- (required) the message to be printed
            force -- (optional) forces message to be print if verbose directive is false
            l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
            cr -- (optional) print a carriage return after the message if True
            
        """
	if self.__quiet == False and (self.__verbose == True or force == True):
            if self.__l != None:
                self.__l.acquire()
            if self.__crLast == True and cr == False:
                self.__crLast = False
                sys.stdout.write("\n")
            if cr == True:
                self.__crLast = True
                msg = "\r" + msg
            sys.stdout.write(msg)
            if cr == False:
                sys.stdout.write("\n")
            if self.__l != None:
                self.__l.release()

    def checkActive(self):
        """ 
            Check to see if dyno is active by calling sendReadCommand.
            
            Returns:
            True -- call to sendReadCommand was successful
            False -- call to sendReadCommand was unsuccessful
            rsp -- result of sendReadCommand when call returned unexpected result
            
        """
        rsp = self.__dS.sendReadCommand('active')
        if(rsp == False):
            return False
        elif(int(rsp) == 1):
            return True
        return rsp

    def driveOk(self):
        """ 
            Check to see if dyno drive is okay by calling sendReadCommand.
            
            Returns:
            True -- call to sendReadCommand was successful
            False -- call to sendReadCommand was unsuccessful
            rsp -- result of sendReadCommand when call returned unexpected result
            
        """
        rsp = self.__dS.sendReadCommand('driveok')
        if(rsp == False):
            return False
        elif(int(rsp) == 1):
            return True
        return rsp

    def driveReady(self):
        """ 
            Check to see if dyno drive is ready by calling sendReadCommand.
            
            Returns:
            True -- call to sendReadCommand was successful
            False -- call to sendReadCommand was unsuccessful
            rsp -- result of sendReadCommand when call returned unexpected result
            
        """
        rsp = self.__dS.sendReadCommand('ready')
        if(rsp == False):
            return False
        elif(int(rsp) == 1):
            return True
        return rsp

    def getVelocity(self):
        """ 
            Get velocity from dyno by calling sendReadCommand.
            
            Returns:
            False -- call to sendReadCommand was unsuccessful
            rsp -- result of sendReadCommand as an integer when call was successful
            
        """
        rsp = self.__dS.sendReadCommand('v')
        if(rsp == False):
            return False
        v = int(rsp)
        if v <= self.__vlim:
            return v
        else:
            self.killDrive(0)
            return False

    def getCurrent(self):
        """ 
            Get current from dyno by calling sendReadCommand.
            
            Returns:
            False -- call to sendReadCommand was unsuccessful
            rsp -- result of sendReadCommand when call was successful
            
        """
        rsp = self.__dS.sendReadCommand('i')
        if(rsp == False):
            return False
        return rsp

    def getTorque(self):
        """ 
            Get torque by using the current from dyno.
            
            Returns:
            False -- call to getCurrent was unsuccessful
            rsp -- calculated torque value if call to getCurrent was successful
            
        """
        global CURRENT_SCALING_FACTOR
        global TORQUE_CONSTANT
        rsp = self.getCurrent()
        if(rsp == False):
            return False
        t = float(rsp)/(CURRENT_SCALING_FACTOR*TORQUE_CONSTANT)
        if t <= self.__tlim:
            return t
        else:
            self.killDrive(2)
            return False

    def enableDrive(self):
        """ 
            Enable dyno drive by calling sendWriteCommand.
            
            Returns:
            True -- call to sendWriteCommand was successful
            rsp -- otherwise
            
        """
        rsp = self.__dS.sendWriteCommand('en')
        if(rsp == True):
            self.printStdOut("Drive enabled successfully.")
            return True
        self.printStdOut("There was an error enabling the drive.",True)
        return rsp

    def disableDrive(self):
        """ 
            Disable dyno drive by calling sendWriteCommand.
            
            Returns:
            True -- call to sendWriteCommand was successful
            False -- otherwise
            
        """
        rsp = self.__dS.sendWriteCommand('dis')
        if(rsp == True):
            self.printStdOut("Drive disabled succesfully.")
            return True
        self.printStdOut("There was an error disabling the drive.",True)
        return rsp

    def forceDisableDrive(self, attempts = -1):
        """
            Forces disable of dyno drive.
            
            Arguments:
            attempts -- (optional) number of times to attempt to disable drive
            
        """
        # Attempt to disable drive until successful or attempts is met
        try:
       	    # if attempts is not set, try forever
            if attempts != -1:
                while self.disableDrive() != True:
                    time.sleep(0.1)
                # if while loop has exited the drive has been disabled
                return True
            else:
            	for i in range(0,attemps+1):
            	    if self.disableDrive():
            	    	return True
            	    else:
            	    	time.sleep(0.1)
        except KeyboardInterrupt:
            sys.exit()
        return False

    def killDrive(self,mode = -1):
        if mode == 0:
            self.printStdOut("Velocity exceeds limit, disabling drive.",True)
        elif mode == 2:
            self.printStdOut("Torque exceeds limit, disabling drive.",True)
        else:
            self.printStdOut("System error, disabling drive.",True)
        rsp = self.forceDisableDrive(5)
        if rsp == True:
            return mode
        else:
            return False
            
            
    
    def __setOpmode(self,mode):
        """ 
            Set operation mode as velocity (0) or torque (2) by calling sendWriteCommand.
            
            Arguments:
            mode -- (required) the operation mode to put dyno in
            
            Returns:
            True -- successfully changed opmode
            False -- invalid opmode sent
            rsp -- call to sendWriteCommand was unsuccessful
            
        """
        if(mode == 0 or mode == 2):
            rsp = self.__dS.sendWriteCommand('opmode=' + str(mode))
            if(rsp == True):
                self.printStdOut("Opmode changed successfully.")
                return True
            else:
                printStdOut,("There was an error setting the opmode.",True)
                return rsp
        elif mode != -1:
            self.printStdOut("Valid opmodes are 0 or 2.",True)
        return False

    def setVelocity(self,velocity):
        """ 
            Set dyno velocity by calling sendWriteCommand.
            
            Arguments:
            velocity -- (required) the desired velocity to be set
            
            Returns:
            True -- successfully set velocity
            False -- dyno in invalid mode or invalid velocity command sent
            rsp -- call to sendWriteCommand was unsuccessful
            
        """
        if self.__mode == 0:
            if self.getTorque() <= self.__tlim:
                try:
                    if (float(velocity) >= 0 and float(velocity) <= self.__vlim):
                        rsp = self.__dS.sendWriteCommand('j=' + str(velocity))
                        if(rsp == True):
                            self.printStdOut("Velocity command sent successfully.")
                            return True
                        else:
                            self.printStdOut("There was an error commanding the specified velocity.",True)
                            return rsp
                    else:
                        self.printStdOut("Velocity must be between 0 and %d RPM." % (self.__vlim),True)
                except ValueError:
                    self.printStdOut("Invalid velocity command.",True)
            else:
                self.killDrive(2)
        else:
            self.printStdOut("Dynamometer is in an unknown mode.",True)
        return False

    def setTorque(self, torque):
        """ 
            Set dyno torque by converting to current then calling sendWriteCommand.
            
            Arguments:
            torque -- (required) the desired torque to be set
            
            Returns:
            True -- successfully set torque
            False -- dyno in invalid mode or invalid torque command sent
            rsp -- call to sendWriteCommand was unsuccessful
            
        """
        global CURRENT_SCALING_FACTOR
        global TORQUE_CONSTANT
        if self.__mode == 2:
            if self.getVelocity() <= self.__vlim:
                try:
                    if(float(torque) >= 0 and float(torque) <= self.__tlim):
                        # Convert to current as the drive torque command is actually a current command.
                        i = str(int(round(float(torque)*CURRENT_SCALING_FACTOR*TORQUE_CONSTANT,0)))
                        rsp = self.__dS.sendWriteCommand('t=' + i)
                        if(rsp == True):
                            self.printStdOut("Te: %0.2f Nm" % (torque))
                            return True
                        else:
                            self.printStdOut("There was an error commanding the specified torque.",True)
                            return rsp
                    else:
                        self.printStdOut("Torque must be between 0 and %0.2f N-m." % (self.__tlim),True)
                except ValueError:
                    self.printStdOut("Invalid torque command.",True)
            else:
                self.killDrive(0)
        else:
            self.printStdOut("Dynamometer is in an unknown mode.",True)
        return False

    def computeLossTorque(self,v):
        """
            Convert current loss to torque loss.
            
            Arguments:
            v -- (required) given velocity
            
            Returns:
            modified result of computeLossCurrent
            
        """
        global TORQUE_CONSTANT
        return self.computeLossCurrent(v)/TORQUE_CONSTANT;

    def computeLossCurrent(self,v):
        """
            Calculate amount of current lost due to velocity.
            
            Arguments:
            v -- (required) given velocity
            
            Returns:
            lost current
            
        """
        global CONSTANT_COEFFICIENT
        global LINEAR_COEFFICIENT
        global QUADRATIC_COEFFICIENT
        return CONSTANT_COEFFICIENT + v*LINEAR_COEFFICIENT + math.pow(v,2)*QUADRATIC_COEFFICIENT

    def setCurrentLimit(self, limit):
        """
            Send command to dyno to set current limit.
            
            Arguments:
            limit -- (required) desired current limit
            
            Returns:
            True -- current limit successfully set
            rsp -- call to sendWriteCommand was unsuccessful
            
        """
        rsp = self.__dS.sendWriteCommand('ilim=' + str(limit))
        if(rsp == True):
            self.printStdOut("Current limit successfully set.")
            return True
        self.printStdOut("There was an error setting the current limit.",True)
        return rsp

    def setTorqueLimit(self,limit):
        """
            Send command to dyno to set torque limit.
            
            Arguments:
            limit -- (required) desired torque limit
            
            Returns:
            True -- torque limit successfully set
            rsp -- call to setCurrentLimit was unsuccessful
            
        """
        rsp = self.setCurrentLimit(int(round(float(limit)*CURRENT_SCALING_FACTOR*TORQUE_CONSTANT)),False)
        if(rsp == True):
            self.printStdOut("Torque limit successfully set.")
            return True
        self.printStdOut("There was an error setting the torque limit.",True)
        return rsp

    def setDynoMode(self, mode = -1):
        """
            Enable dyno drive in velocity or torque mode.
            
            Arguments:
            torqueLimit -- (optional) maximum torque value
            testTorque -- (optional) whether or not to test the torque
            testVelocity -- (optional) whether or not to test the velocity
            
            Returns:
            rsp -- result of trying to enable a dyno mode
            recursivly call setDynoMode if easily fixable error found
            False -- invalid mode entered
            
        """
        # Ask for mode if none is given.
        if mode == -1:
            if self.__l != None:
                self.__l.acquire()
            inputMode = raw_input("Which mode should the dyno be placed in (0) velocity mode or (2) torque mode? ")
            if self.__l != None:
                self.__l.release()
            mode = int(inputMode)
        if mode == 0:
            # Attempt to enable velocity mode.
            rsp = self.__enableVelocityMode()
        elif mode == 2:
            # Attempt to enable torque mode.
            rsp = self.__enableTorqueMode()
        else:
            self.printStdOut("Invalid mode.",True)
            return False
        # Look for easily fixable errors.
        if rsp == 5:
            # A response code of 5 can usually be fixed by trying again.
            time.sleep(0.1)
            return self.setDynoMode(mode)
        elif rsp == 23:
            self.disableDrive()
            return self.setDynoMode(mode)
        else:
            return rsp
        

    def __enableVelocityMode(self):
        """
            Enable dyno for velocity commands in velocity mode after disabling dyno drive.
            
            Arguments:
            testVelocity -- (optional) option for testing drive for a given velocity
            
            Returns:
            True -- drive ready for velocity commands
            rsp -- error enabling velocity mode
            
        """
        rsp = self.disableDrive()
        if(rsp == True):
            time.sleep(0.1)
            rsp = self.__setOpmode(0)
            if(rsp == True):
                time.sleep(0.1)
                rsp = self.enableDrive()
                if(rsp == True):
                    active = self.checkActive()
                    time.sleep(0.1)
                    driveok = self.driveOk()
                    if active == True and driveok == True:
                        self.printStdOut("The drive should be ready to accept velocity commands.")
                        self.__mode = 0
                        return True
                    else:
                        rsp = "Active response: " + str(active) + "; Driveok response: " + str(driveok)
        self.printStdOut("There was an error enabling velocity mode.",True)
        return rsp

    def __enableTorqueMode(self):
        """
            Enable dyno for torque commands in torque mode after disabling dyno drive.
            
            Arguments:
            torqueLimit -- (optional) maximum amount of torque to be used
            testTorque -- (optional) option for testing drive for a given torque
            
            Returns:
            True -- drive ready for torque commands
            rsp -- error enabling torque mode
            
        """
        rsp = self.disableDrive()
        if(rsp == True):
            time.sleep(0.1)
            rsp = self.__setOpmode(2)
            if(rsp == True):
                time.sleep(0.1)
                rsp = self.enableDrive()
                if(rsp == True):
                    active = self.checkActive()
                    time.sleep(0.1)
                    driveok = self.driveOk()
                    if active == True and driveok == True:
                        self.printStdOut("The drive should be ready to accept torque commands.")
                        self.__mode = 2
                        return True
                    else:
                        rsp = "Active response: " + str(active) + "; Driveok response: " + str(driveok)
        self.printStdOut("There was an error enabling torque mode.",True)
        return rsp

    def testTorqueMode(self, t = None):
        """
            Get torque and check if it is valid and does not cause the system to fail.
            
            Arguments:
            t -- (optional) the torque to test
            
        """
        if t == None:
            t = raw_input(" What torque would you like to apply for the test? ")
            t = float(t)
        if t > 1:
            self.printStdOut("That seems a little high for a simple test. I recommend using a torque at or below 1 Nm.")
            self.testTorqueMode(None)
        rsp = self.setTorque(t)
        if rsp == True:
            self.printStdOut("Applying test torque for 10 seconds...")
            time.sleep(10)
            if(self.setTorque(0) == True):
                self.printStdOut("Test completed successfully.")

    def testVelocityMode(self, v = None):
        """
            Get velocity and check if it is valid and does not cause the system to fail.
            
            Arguments:
            v -- (optional) the velocity to test
            '/dev/Bluetooth-Modem'
        """
        if v == None:
            v = raw_input(" What velocity would you like to apply for the test? ")
            v = float(v)
        if v > 1000:
            self.printStdOut("That seems a little high for a simple test. I recommend using a velocity at or below 1000 rpm.")
            self.testVelocityMode(None)
        rsp = self.setVelocity(v)
        if rsp == True:
            printStdOut("Applying test velocity for 10 seconds...")
            time.sleep(10)
            if(self.setVelocity(0) == True):
                printStdOut("Test completed successfully.")


class dynoSerial(serial.Serial):
    """
        The serial connection.
        
        Methods:

        sendCommand
        verifyCommand
        sendWriteCommand
        sendReadCommand
        closeSerial
        scanSerial
        promptForPort
        promptForBaud
        
    """
# Variables
#   __l

    def __init__(self, port = None, baud = None, l = None):
		self.__l = l
		if(port == None):
			port = self.promptForPort()
		if(baud == None):
			baud = self.promptForBaud()
		serial.Serial.__init__(self,int(port),long(baud),timeout=5)
		time.sleep(0.5)
        
    def printStdOut(self, msg):
        """
            Print using StdOut for option of using multiprocessing
            
            Arguments:
            msg -- (required) the message to be printed
            l -- (optional) lock object to synchronize printing to StdOut when using multiprocessing
            cr -- (optional) print a carriage return after the message if True
            
        """
        if self.__l != None:
            self.__l.acquire()
        print msg
        if self.__l != None:
            self.__l.release()
        
    def promptForPort(self):
        """
            Obtain desired port to be used.
            
            Returns:
            recursively call promptForPort if invalid port selected
            port -- the port to be used
            
            """
        self.printStdOut("Serial ports available:")
        numPorts = self.scanSerial()
        port = raw_input("Select a serial port number: ")
        if port == "q":
            sys.exit()
        elif (int(port) < 0):
            self.printStdOut("Invalid port selected. Try again (or enter q to quit).")
            return self.promptForPort()
        else:
            port = port.strip()
            return port

    def scanSerial(self):
        """
            Scan serial
            
            Returns:
            next position to be checked
            
            """
        path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
        for i in itertools.count():
            try:
                val = winreg.EnumValue(key, i)
                portName = str(val[1])
                self.printStdOut(str(int(portName[-1])-1) + ": " + portName)
            except EnvironmentError:
                break
        return i-1

    def promptForBaud(self):
        """
            Obtain desired baud rate to be used.
            
            Returns:
            recursively call promptForBaud if invalid baud rate selected
            baud -- the baud rate to be used
            
            """
        availableBaud = ['9600','19200']
        self.printStdOut("Baud rates available:")
        for b in availableBaud:
            self.printStdOut(b)
        baud = raw_input("Select a baud rate: ")
        if baud == "q":
            sys.exit()
        elif not baud in availableBaud:
            self.printStdOut("Inavlid baud rate. Try again (or enter q to quit).")
            return self.promptForBaud()
        else:
            return baud
        
    def closeSerial(self):
        """
            Close and disconnect from serial port.
            
            Returns:
            True -- serial closed successfully
            False -- unable to close serial port or serial connection never opened
            
            """
        self.printStdOut("Attempting to disconnect serial connection...") 
        try:
            self.close()
            if(self.isOpen()):
                self.printStdOut("Unable to close serial port.")
            else:
                self.printStdOut("Serial closed successfully.")
                return True
        except AttributeError:
            self.printStdOut("Serial connection never opened.") 
        return False

    def sendWriteCommand(self,command):
        """ 
            Call verifyCommand function if a serial command was sent successfully to dyno.
            
            Arguments:
            command -- (required) a string containing the command to be sent
            
            Returns:
            False -- call to sendCommand was unsuccessful
            result of verifyCommand otherwise
            
        """
        if self.isOpen():
            if(self.sendCommand(command)):
                return self.verifyCommand()
        return False

    def sendReadCommand(self,command):
        """ 
            Send read command to dyno.
            
            Arguments:
            command -- (required) a string containing the command to be sent
            
            Returns:
            False -- serial connection not available or unexpected character read
            val -- the string that was read
            
        """
        try:
            if(self.isOpen()):
                if(self.sendCommand(command)):
                    # Check if new line character.
                    if(self.read() == chr(0x0A)):
                        val = str()
                        # Read a byte from the serial port until a carriage return is read.
                        while True:
                            b = self.read()
                            # Check if carriage return character.
                            if(b != chr(0x0D)):
                                val = val + b
                            else:
                                if(self.read() == chr(0x0A)):
                                    if(self.read() == '-'):
                                        if(self.read() == '-'):
                                            if(self.read() == '>'):
                                                return val                                    
                else:
                    self.printStdOut("Read command could not be sent.")
            else:
                self.printStdOut("Serial connection not open.")
        except AttributeError:
            self.printStdOut("Serial connection never opened.")
        return False
    
    def sendCommand(self,command):
        """ 
            Send serial command byte by byte to dyno.
            
            Arguments:
            command -- (required) a string containing the command to be sent
            
            Returns:
            True -- everything worked
            False -- otherwise
            
        """
        command = command + chr(0x0D)
        try:
            if(self.isOpen()):
                # Clear the serial output buffer.
                self.flushOutput
                self.flushInput
                # Clear anything that is in waiting.
                self.read(self.inWaiting())
                # Do for each character of command string.
                for char in command:
                    # Send character out serial port.
                    if(self.write(char) > 0):
                        try:
                            # Read a byte from the serial port.
                            c = self.read()
                            if(c != char):
                                self.printStdOut("Byte received (" + str(ord(c)) + ") does not matc byte sent (" + str(ord(char)) + ")")
                                return False
                        except self.SerialException:
                            self.printStdOut("Serial exception.")
                            return False
                    else:
                        self.printStdOut("Serial port error.")
                        return False
                return True
            else:
                self.printStdOut("Serial connection not open.")
        except AttributeError:
            self.printStdOut("Serial connection never opened.")
        return False

    def verifyCommand(self):
        """ 
            Verify serial command to dyno.
            
            Returns:
            True -- everything worked
            False -- serial connection not available or unexpected character read
            val -- err integer number returned by dyno if an error occurred
            
        """
        try:
            if(self.isOpen()):
                # Check for new line character.
                if(self.read() == chr(0x0A)):
                    # Read a byte from the serial port.
                    c = self.read()
                    if(c == '-'):
                        if(self.read() == '-'):
                            if(self.read() == '>'):
                                return True
                    # Check for bell character.
                    elif(c == chr(0x07)):
                        val = str()
                        while True:
                            b = self.read()
                            # Check for carriage return character.
                            if(b != chr(0x0D)):
                                val = val + b
                            else:
                                if(self.read() == chr(0x0A)):
                                    if(self.read() == '-'):
                                        if(self.read() == '-'):
                                            if(self.read() == '>'):
                                                if val[:3] == 'ERR':
                                                    self.printStdOut(val)
                                                    # Return err number if there is an error.
                                                    return int(val[4:6])
                                                break
            else:
                self.printStdOut("Serial connection not open.")
        except AttributeError:
            self.printStdOut("Serial connection never opened.")
        return False