"""
    Copyright 2012 Stanton T. Cady
    Copyright 2012 Hannah Hasken
    
<<<<<<< HEAD
    ServoStar_python  v0.2.4 -- March 30, 2012
=======
    ServoStar_python  v0.2.1 -- March 28, 2012
>>>>>>> bd77ab46f3374f761fc859f910875f11c9a46359
    
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
        setOpmode
        setVelocity
        setTorque
        computeLossTorque
        computeLossCurrent
        setCurrentLimit
        setTorqueLimit
        killSystem
        enableDyno
        enableVelocityMode
        enableTorqueMode
        testTorqueMode
        testVelocityMode
        checkSpeed
        setupDynoSerial
        startDyno
        printStdOut
        
    """
    # Variables
#    dSerial
#    mode
#    torque
#    velocity
#    ser
#    l
    
    
    def __init__(self, mode = -1, port = None, baud = None, torque = None, velocity = None, l = None):
    	self.l = l
		self.dSerial = self.dynoSerial(port,baud,l)
		if(self.dSerial != False):
			self.mode = self.setOpmode(mode)
			if(self.mode != False):
				if(torque == None):
					self.torque = self.setTorque(torque)
				else:
					self.torque = torque
				if(velocity == None):
					self.velocity = self.setVelocity(velocity)
				else:
					self.velocity = velocity
				printStdOut("Dyno object created successfully.",self.l)
			else:
				printStdOut("There was an error setting the opmode",self.l)
		else:
			printStdOut("There was an error opening the dyno serial port",self.l)
    
    class dynoSerial:
        """
            The serial connection.
            
            Methods:
            sendCommand
            verifyCommand
            sendWriteCommand
            sendReadCommand
            openSerial
            closeSerial
            scanSerial
            promptForPort
            promptForBaud
            
        """
        # Variables
#        ser
#        port
#        baud
#        l
        
        # Methods
        def __init__(self, port = None, baud = None, l = None):
            self.l = l
            if(port == None):
                self.port = self.promptForPort()
            else:
                self.port = port
            if(baud == None):
                self.baud = self.promptForBaud()
            else:
                self.baud = baud
            self.ser = self.openSerial()
        
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
                if(self.ser.isOpen()):
                    # Clear the serial output buffer.
                    self.ser.flushOutput
                    self.ser.flushInput
                    # Clear anything that is in waiting.
                    self.ser.read(self.ser.inWaiting())
                    # Do for each character of command string.
                    for char in command:
                        # Send character out serial port.
                        if(self.ser.write(char) > 0):
                            try:
                                # Read a byte from the serial port.
                                c = self.ser.read()
                                if(c != char):
                                    printStdOut("Byte received (" + str(ord(c)) + ") does not match byte sent (" + str(ord(char)) + ")",self.l)
                                    return False
                            except self.ser.SerialException:
                                printStdOut("Serial exception.",self.l)
                                return False
                        else:
                            printStdOut("Serial port error.",self.l)
                            return False
                    return True
                else:
                    printStdOut("Serial connection not open.",self.l)
            except AttributeError:
                printStdOut("Serial connection never opened.",self.l)
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
                if(self.ser.isOpen()):
                    # Check for new line character.
                    if(self.ser.read() == chr(0x0A)):
                        # Read a byte from the serial port.
                        c = self.ser.read()
                        if(c == '-'):
                            if(self.ser.read() == '-'):
                                if(self.ser.read() == '>'):
                                    return True
                        # Check for bell character.
                        elif(c == chr(0x07)):
                            val = str()
                            while True:
                                b = self.ser.read()
                                # Check for carriage return character.
                                if(b != chr(0x0D)):
                                    val = val + b
                                else:
                                    if(self.ser.read() == chr(0x0A)):
                                        if(self.ser.read() == '-'):
                                            if(self.ser.read() == '-'):
                                                if(self.ser.read() == '>'):
                                                    if val[:3] == 'ERR':
                                                        printStdOut(val,self.l)
                                                        # Return err number if there is an error.
                                                        return int(val[4:6])
                                                    break
                else:
                    printStdOut("Serial connection not open.",self.l)
            except AttributeError:
                printStdOut("Serial connection never opened.",self.l)
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
            if self.ser != False:
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
                if(self.ser.isOpen()):
                    if(self.sendCommand(command)):
                        # Check if new line character.
                        if(self.ser.read() == chr(0x0A)):
                            val = str()
                            # Read a byte from the serial port until a carriage return is read.
                            while True:
                                b = self.ser.read()
                                # Check if carriage return character.
                                if(b != chr(0x0D)):
                                    val = val + b
                                else:
                                    if(self.ser.read() == chr(0x0A)):
                                        if(self.ser.read() == '-'):
                                            if(self.ser.read() == '-'):
                                                if(self.ser.read() == '>'):
                                                    return val                                    
                    else:
                        printStdOut("Read command could not be sent.",self.l)
                else:
                    printStdOut("Serial connection not open.",self.l)
            except AttributeError:
                printStdOut("Serial connection never opened.",self.l)
            return False
                
        def openSerial(self):
            """
                Open and connect to serial port.
                
                Returns:
                ser -- the serial port object to use
                False -- unable to open serial port
                
                """
            if self.port == None:
                self.port = self.promptForPort()
            if self.baud == None:
                self.baud = self.promptForBaud()
            printStdOut("Connecting...",self.l)
            self.ser = serial.Serial(int(self.port),long(self.baud),timeout=5)
            if (self.ser.isOpen()):
                self.ser.flushInput()
                printStdOut("Serial opened successfully.",self.l)
                return self.ser
            else:
                printStdOut("Unable to open serial port.",self.l) 
            return False

        def closeSerial(self):
            """
                Close and disconnect from serial port.
                
                Returns:
                True -- serial closed successfully
                False -- unable to close serial port or serial connection never opened
                
                """
            printStdOut("Attempting to disconnect serial connection...",self.l) 
            try:
                self.ser.close()
                if(self.ser.isOpen()):
                    printStdOut("Unable to close serial port.",self.l)
                else:
                    printStdOut("Serial closed successfully.",self.l)
                    return True
            except AttributeError:
                printStdOut("Serial connection never opened.",self.l) 
            return False

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
                    printStdOut(str(int(portName[-1])-1) + ": " + portName,self.l)
                except EnvironmentError:
                    break
            return i-1

        def promptForPort(self):
            """
                Obtain desired port to be used.
                
                Returns:
                recursively call promptForPort if invalid port selected
                port -- the port to be used
                
                """
            printStdOut("Serial ports available:",self.l)
            numPorts = self.scanSerial()
            self.port = raw_input("Select a serial port number: ")
            if self.port == "q":
                sys.exit()
            elif (int(port) < 0):
                printStdOut("Invalid port selected. Try again (or enter q to quit).",self.l)
                return self.promptForPort()
            else:
                self.port = self.port.strip()
                return self.port

        def promptForBaud(self):
            """
                Obtain desired baud rate to be used.
                
                Returns:
                recursively call promptForBaud if invalid baud rate selected
                baud -- the baud rate to be used
                
                """
            availableBaud = ['9600','19200']
            printStdOut("Baud rates available:",self.l)
            for self.baud in availableBaud:
                printStdOut(self.baud,self.l)
            baud = raw_input("Select a baud rate: ")
            if self.baud == "q":
                sys.exit()
            elif not self.baud in availableBaud:
                printStdOut("Inavlid baud rate. Try again (or enter q to quit).",self.l)
                return self.promptForBaud()
            else:
                return self.baud

    def checkActive(self):
        """ 
            Check to see if dyno is active by calling sendReadCommand.
            
            Returns:
            True -- call to sendReadCommand was successful
            False -- call to sendReadCommand was unsuccessful
            rsp -- result of sendReadCommand when call returned unexpected result
            
        """
        rsp = self.dSerial.sendReadCommand('active')
        if(rsp == false):
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
        rsp = self.dSerial.sendReadCommand('driveok')
        if(rsp == false):
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
        rsp = self.dSerial.sendReadCommand('ready')
        if(rsp == false):
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
        rsp = self.dSerial.sendReadCommand('v')
        if(rsp == false):
            return False
        return int(rsp)

    def getCurrent(self):
        """ 
            Get current from dyno by calling sendReadCommand.
            
            Returns:
            False -- call to sendReadCommand was unsuccessful
            rsp -- result of sendReadCommand when call was successful
            
        """
        rsp = self.dSerial.sendReadCommand('i')
        if(rsp == false):
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
        if(rsp == false):
            return False
        return float(rsp)/(CURRENT_SCALING_FACTOR*TORQUE_CONSTANT)

    def enableDrive(self):
        """ 
            Enable dyno drive by calling sendWriteCommand.
            
            Returns:
            True -- call to sendWriteCommand was successful
            rsp -- otherwise
            
        """
        rsp = self.dSerial.sendWriteCommand('en')
        if(rsp == True):
            printStdOut("Drive enabled successfully.",self.l)
            return True
        printStdOut("There was an error enabling the drive.",self.l)
        return rsp

    def disableDrive(self):
        """ 
            Disable dyno drive by calling sendWriteCommand.
            
            Returns:
            True -- call to sendWriteCommand was successful
            False -- otherwise
            
        """
        rsp = self.dSerial.sendWriteCommand('dis')
        if(rsp == True):
            printStdOut("Drive disabled succesfully.",self.l)
            return True
        printStdOut("There was an error disabling the drive.",self.l)
        return rsp

    def setOpmode(self,mode):
        """ 
            Set operation mode as velocity (1) or torque (2) by calling sendWriteCommand.
            
            Arguments:
            mode -- (required) the operation mode to put dyno in
            
            Returns:
            True -- successfully changed opmode
            False -- invalid opmode sent
            rsp -- call to sendWriteCommand was unsuccessful
            
        """
        if(mode == 1 or mode == 2):
            rsp = self.dSerial.sendWriteCommand('opmode=' + str(mode))
            if(rsp == True):
                printStdOut("Opmode changed successfully.",self.l)
                return True
            else:
                printStdOut,("There was an error setting the opmode.",self.l)
                return rsp
        elif mode != -1:
            printStdOut("Valid opmodes are 1 or 2.",self.l)
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
        if self.mode == 1:
            try:
                if (float(velocity) >= 0 and float(velocity) <= 1300):
                    rsp = self.dSerial.sendWriteCommand('v=' + str(velocity))
                    if(rsp == True):
                        printStdOut("Velocity command sent successfully.",self.l)
                        return True
                    else:
                        printStdOut("There was an error commanding the specified velocity.",self.l)
                        return rsp
                else:
                    printStdOut("Velocity must be between 0 and 1300 RPM.",self.l)
            except ValueError:
                printStdOut("Invalid velocity command.",self.l)
        else:
            printStdOut("Dynamometer is in an unknown mode.",self.l)
        return False

    def setTorque(self,torque,quiet=False):
        """ 
            Set dyno torque by converting to current then calling sendWriteCommand.
            
            Arguments:
            torque -- (required) the desired torque to be set
            quiet -- (optional) print to StdOut if false
            
            Returns:
            True -- successfully set torque
            False -- dyno in invalid mode or invalid torque command sent
            rsp -- call to sendWriteCommand was unsuccessful
            
        """
        global CURRENT_SCALING_FACTOR
        global TORQUE_CONSTANT
        if self.mode == 2:
            try:
                if(float(torque) >= 0 and float(torque) <= 6):
                    # Convert to current as the drive torque command is actually a current command.
                    i = str(int(round(float(torque)*CURRENT_SCALING_FACTOR*TORQUE_CONSTANT,0)))
                    rsp = self.dSerial.sendWriteCommand('t=' + i)
                    if(rsp == True):
                        printStdOut("Te: %0.2f Nm" % (t),self.l) if not quiet else ''
                        return True
                    else:
                        printStdOut("There was an error commanding the specified torque.",self.l)
                        return rsp
                else:
                    printStdOut("Torque must be between 0 and 6 N-m.",self.l)
            except ValueError:
                printStdOut("Invalid torque command.",self.l)
        else:
            printStdOut("Dynamometer is in an unknown mode.",self.l)
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

    def setCurrentLimit(self,limit,quiet = True):
        """
            Send command to dyno to set current limit.
            
            Arguments:
            limit -- (required) desired current limit
            quiet -- (optional) print to StdOut if false
            
            Returns:
            True -- current limit successfully set
            rsp -- call to sendWriteCommand was unsuccessful
            
        """
        rsp = self.dSerial.sendWriteCommand('ilim=' + str(limit))
        if(rsp == True):
            printStdOut("Current limit successfully set.",self.l) if not quiet else sys.stdout.write('')
            return True
        printStdOut("There was an error setting the current limit.",self.l) if not quiet else sys.stdout.write('')
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
            printStdOut("Torque limit successfully set.",self.l)
            return True
        printStdOut("There was an error setting the torque limit.",self.l)
        return rsp

    def killSystem(self,dyno = None,exit = True):
        """
            Disable dyno drive.
            
            Arguments:
            dyno -- (optional) dyno object
            exit -- (optional) option to exit system
            
        """
        if dyno != None and dyno.isOpen():
            # Attempt to disable drive until successful.
            try:
                while self.disableDrive(dyno) != True:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                sys.exit()
            closeSerial(dyno)
        if exit == True:
            sys.exit()

    def enableDyno(self, torqueLimit = 6, testTorque = True, testVelocity = True, quiet = False):
        """
            Enable dyno drive in velocity or torque mode.
            
            Arguments:
            torqueLimit -- (optional) maximum torque value
            testTorque -- (optional) whether or not to test the torque
            testVelocity -- (optional) whether or not to test the velocity
            quiet -- (optional) print to StdOut if false
            
            Returns:
            rsp -- result of trying to enable a dyno mode
            recursivly call enableDyno if easily fixable error found
            False -- invalid mode entered
            
        """
        global dynoMode
        # Ask for mode if none is given.
        if self.mode == -1:
            if self.l != None:
                self.l.acquire()
            self.mode = raw_input("Which mode should the dyno be placed in (1) velocity mode or (2) torque mode? ")
            if self.l != None:
                self.l.release()
            self.mode = int(self.mode)
        if self.mode == 1:
            # Attempt to enable velocity mode.
            rsp = self.enableVelocityMode(testVelocity)
            if rsp == True:
                dynoMode = 1
            return rsp
        elif self.mode == 2:
            # Attempt to enable torque mode.
            rsp = self.enableTorqueMode(torqueLimit,testTorque,quiet)
            if rsp == True:
                dynoMode = 2
            # Look for easily fixable errors.
            elif rsp == 5:
                # A response code of 5 can usually be fixed by trying again.
                time.sleep(0.1)
                return self.enableDyno(torqueLimit,testTorque,testVelocity)
            elif rsp == 23:
                self.disableDrive()
                return self.enableDyno(torqueLimit,testTorque,testVelocity)
            return rsp
        else:
            printStdOut("Inavlid mode.",self.l)
        return False

    def enableVelocityMode(self, testVelocity = True):
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
            rsp = self.setOpmode(1)
            if(rsp == True):
                time.sleep(0.1)
                rsp = self.enableDrive()
                if(rsp == True):
                    active = self.checkActive()
                    time.sleep(0.1)
                    driveok = self.driveOk()
                    if active == True and driveok == True:
                        printStdOut("The drive should be ready to accept velocity commands.",self.l)
                        if testVelocity == True:
                            q = raw_input("Shall I test the drive? (y/n) ")
                            if q == 'y':
                                self.testVelocityMode(None)
                        return True
                    else:
                        rsp = "Active response: " + str(active) + "; Driveok response: " + str(driveok)
        printStdOut("There was an error enabling velocity mode.",self.l)
        return rsp

    def enableTorqueMode(self, torqueLimit = None, testTorque = True, quiet = False):
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
            rsp = self.setOpmode(2)
            if(rsp == True):
                time.sleep(0.1)
                rsp = self.enableDrive()
                if(rsp == True):
                    if(torqueLimit != None):
                        time.sleep(0.1)
                        if(self.setTorqueLimit(torqueLimit == True,quiet)):
                            printStdOut("The torque limit has been set.",self.l)
                    active = self.checkActive()
                    time.sleep(0.1)
                    driveok = self.driveOk()
                    if active == True and driveok == True:
                        printStdOut("The drive should be ready to accept torque commands.",self.l)
                        if testTorque == True:
                            q = raw_input("Shall I test the drive? (y/n) ")
                            if q == 'y':
                                self.testTorqueMode(None,quiet)
                        return True
                    else:
                        rsp = "Active response: " + str(active) + "; Driveok response: " + str(driveok)
        printStdOut("There was an error enabling torque mode.",self.l)
        return rsp

    def testTorqueMode(self, t = None, quiet = False):
        """
            Get torque and check if it is valid and does not cause the system to fail.
            
            Arguments:
            t -- (optional) the torque to test
            quiet -- (optinal) print to StdOut if false
            
        """
        if t == None:
            t = raw_input(" What torque would you like to apply for the test? ")
            t = float(t)
        if t > 1:
            printStdOut("That seems a little high for a simple test. I recommend using a torque at or below 1 Nm.",self.l)
            self.testTorqueMode(None,quiet)
        rsp = self.setTorque(t,quiet)
        if rsp == True:
            printStdOut("Applying test torque for 10 seconds...",self.l)
            time.sleep(10)
            if(self.setTorque(0,quiet) == True):
                printStdOut("Test completed successfully.",self.l)

    def testVelocityMode(self, v = None):
        """
            Get velocity and check if it is valid and does not cause the system to fail.
            
            Arguments:
            v -- (optional) the velocity to test
            
        """
        if v == None:
            v = raw_input(" What velocity would you like to apply for the test? ")
            v = float(v)
        if v > 1000:
            printStdOut("That seems a little high for a simple test. I recommend using a velocity at or below 1000 rpm.",self.l)
            self.testVelocityMode(None)
        rsp = self.setVelocity(v)
        if rsp == True:
            printStdOut("Applying test velocity for 10 seconds...",self.l)
            time.sleep(10)
            if(self.setVelocity(0) == True):
                printStdOut("Test completed successfully.",self.l)

    def checkSpeed(self, t):
        """
            Check that the given velocity is valid.
            
            Arguments:
            t -- (reqired) 
            
            Returns:
            v -- the given, valid speed
            killSystem if high invalid speed given
            
        """
        v = float(getVelocity())
        if v > CUT_OUT_SPEED:
            printStdOut("Speed of " + str(v) + " rpm exceeds cut out speed of " + str(CUT_OUT_SPEED) + " rpm.  Killing system.",self.l)
            self.killSystem()
        else:
            return v

    def setupDynoSerial(self):
        """
            Obtain port and baud parameters and attempt to open a serial connection to dyno.
            
            Returns:
            call to openSerial
            
        """
        # Check if parameters for serial port and baud rate have been passed in.
        if self.dSerial.port == None or self.dSerial.baud == None:
            # Missing port or baud so ask for them.
            q = raw_input("Port and/or baud rate not provided. Shall I use the default settings? (y/n): ")
            if self.dSerial.port == None and q == 'y':
                    self.dSerial.port = 0
            if self.dSerial.baud == None and q == 'y':
                    self.dSerial.baud = 9600
        printStdOut("Attempting to setup the dynamometer.",self.l)
        # Attempt to open a serial connection to the dyno.
        return self.dSerial.openSerial()

    def startDyno(self, initialTorque = None, vref = None, initialDelay = None, quiet = False):
        """
            Attempt to start the dyno with given velocity and torque values.
            
            Arguments:
            initialTorque -- (optional) how much torque to initially apply
            vref -- (optional) velocity to set dyno to in rpm
            initialDelay -- (optional) how long to let the drive accelerate in seconds
            
        """
        if vref == None:
            vref = 1000
        #initialSpeed = int(raw_input("What velocity in rpm would like to use? "))
        if vref < 1300:
            printStdOut("Starting dyno to " + str(vref) + " rpm.",self.l)
            if initialTorque == None:
                t = 0
            else:
                t = initialTorque
                if initialDelay == None:
                    initialDelay = 15
                if initialDelay > 0:
                    self.setTorque(t,quiet)
                    samplePeriod = 0.2
                    try:
                        for i in range(int(initialDelay/samplePeriod)):
                            v = self.checkSpeed(t)
                            printStdOut("Letting drive accelerate for %d seconds.  Current speed: %d rpm" % (initialDelay,v),True,self.l)
                            time.sleep(samplePeriod)
                        sys.stdout.write("\n")
                        return True
                    except KeyboardInterrupt:
                        self.killSystem()
        else:
            printStdOut("Please choose a speed below 1300 rpm.",self.l)
            self.startDyno(initialTorque,vref,initialDelay,quiet)

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
