"""
    Copyright 2012 Stanton T. Cady
    Copyright 2012 Hannah Hasken
    
    ServoStar_python  v0.2.7 -- April 27, 2012
    
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
    
    
    def __init__(self, mode = -1, port = None, baud = None, initial = None, tlim = 6, vlim = 1300, l = None):
        self.__l = l
        self.__mode = -1
        self.__torque = 0
        self.__velocity = 0
        self.__tlim = tlim
        self.__vlim = vlim
        self.__dS = dynoSerial(port,baud,l)
        if self.setTorqueLimit(tlim):
            rsp = self.setDynoMode(mode)
            if(rsp == True):
                if(initial != None and self.__mode == 2):
                    printStdOut("Commanding initial torque.",self.__l)
                    self.setTorque(torque)
                elif(initial != None and self.__mode == 1):
                    printStdOut("Commanding initial velocity.",self.__l)
                    self.setVelocity(velocity)
                printStdOut("Dyno object created successfully.",self.__l)
            else:
                printStdOut("There was an error enabling the drive: " + str(rsp),self.__l)
        else:
            printStdOut("Could not set torque limit, aborting drive enable",self.__l)

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
        return int(rsp)

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
        return float(rsp)/(CURRENT_SCALING_FACTOR*TORQUE_CONSTANT)

    def enableDrive(self):
        """ 
            Enable dyno drive by calling sendWriteCommand.
            
            Returns:
            True -- call to sendWriteCommand was successful
            rsp -- otherwise
            
        """
        rsp = self.__dS.sendWriteCommand('en')
        if(rsp == True):
            printStdOut("Drive enabled successfully.")
            return True
        printStdOut("There was an error enabling the drive.",self.__l)
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
            printStdOut("Drive disabled succesfully.",self.__l)
            return True
        printStdOut("There was an error disabling the drive.",self.__l)
        return rsp

    def __setOpmode(self,mode):
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
            rsp = self.__dS.sendWriteCommand('opmode=' + str(mode))
            if(rsp == True):
                printStdOut("Opmode changed successfully.",self.__l)
                return True
            else:
                printStdOut,("There was an error setting the opmode.",self.__l)
                return rsp
        elif mode != -1:
            printStdOut("Valid opmodes are 1 or 2.",self.__l)
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
        if self.__mode == 1:
            try:
                if (float(velocity) >= 0 and float(velocity) <= self.__vlim):
                    rsp = self.__dS.sendWriteCommand('v=' + str(velocity))
                    if(rsp == True):
                        printStdOut("Velocity command sent successfully.",self.__l)
                        return True
                    else:
                        printStdOut("There was an error commanding the specified velocity.",self.__l)
                        return rsp
                else:
                    printStdOut("Velocity must be between 0 and %d RPM." % (self.__vlim),self.__l)
            except ValueError:
                printStdOut("Invalid velocity command.",self.__l)
        else:
            printStdOut("Dynamometer is in an unknown mode.",self.__l)
        return False

    def setTorque(self, torque, quiet=False):
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
        if self.__mode == 2:
            try:
                if(float(torque) >= 0 and float(torque) <= self.__tlim):
                    # Convert to current as the drive torque command is actually a current command.
                    i = str(int(round(float(torque)*CURRENT_SCALING_FACTOR*TORQUE_CONSTANT,0)))
                    rsp = self.__dS.sendWriteCommand('t=' + i)
                    if(rsp == True):
                        printStdOut("Te: %0.2f Nm" % (torque),self.__l) if not quiet else ''
                        return True
                    else:
                        printStdOut("There was an error commanding the specified torque.",self.__l)
                        return rsp
                else:
                    printStdOut("Torque must be between 0 and %0.2f N-m." % (self.__tlim),self.__l)
            except ValueError:
                printStdOut("Invalid torque command.",self.__l)
        else:
            printStdOut("Dynamometer is in an unknown mode.",self.__l)
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
        rsp = self.__dS.sendWriteCommand('ilim=' + str(limit))
        if(rsp == True):
            printStdOut("Current limit successfully set.",self.__l) if not quiet else sys.stdout.write('')
            return True
        printStdOut("There was an error setting the current limit.",self.__l) if not quiet else sys.stdout.write('')
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
            printStdOut("Torque limit successfully set.",self.__l)
            return True
        printStdOut("There was an error setting the torque limit.",self.__l)
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
                while self.disableDrive() != True:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                sys.exit()
            closeSerial()
        if exit == True:
            sys.exit()

    def setDynoMode(self, mode = -1):
        """
            Enable dyno drive in velocity or torque mode.
            
            Arguments:
            torqueLimit -- (optional) maximum torque value
            testTorque -- (optional) whether or not to test the torque
            testVelocity -- (optional) whether or not to test the velocity
            quiet -- (optional) print to StdOut if false
            
            Returns:
            rsp -- result of trying to enable a dyno mode
            recursivly call setDynoMode if easily fixable error found
            False -- invalid mode entered
            
        """
        # Ask for mode if none is given.
        if mode == -1:
            if self.__l != None:
                self.__l.acquire()
            inputMode = raw_input("Which mode should the dyno be placed in (1) velocity mode or (2) torque mode? ")
            if self.__l != None:
                self.__l.release()
            mode = int(inputMode)
        if mode == 1:
            # Attempt to enable velocity mode.
            rsp = self.__enableVelocityMode()
        elif mode == 2:
            # Attempt to enable torque mode.
            rsp = self.__enableTorqueMode()
        else:
            printStdOut("Inavlid mode.",self.__l)
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
            rsp = self.__setOpmode(1)
            if(rsp == True):
                time.sleep(0.1)
                rsp = self.enableDrive()
                if(rsp == True):
                    active = self.checkActive()
                    time.sleep(0.1)
                    driveok = self.driveOk()
                    if active == True and driveok == True:
                        printStdOut("The drive should be ready to accept velocity commands.",self.__l)
                        self.__mode = 1
                        return True
                    else:
                        rsp = "Active response: " + str(active) + "; Driveok response: " + str(driveok)
        printStdOut("There was an error enabling velocity mode.",self.__l)
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
                        printStdOut("The drive should be ready to accept torque commands.",self.__l)
                        self.__mode = 2
                        return True
                    else:
                        rsp = "Active response: " + str(active) + "; Driveok response: " + str(driveok)
        printStdOut("There was an error enabling torque mode.",self.__l)
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
            printStdOut("That seems a little high for a simple test. I recommend using a torque at or below 1 Nm.",self.__l)
            self.testTorqueMode(None,quiet)
        rsp = self.setTorque(t,quiet)
        if rsp == True:
            printStdOut("Applying test torque for 10 seconds...",self.__l)
            time.sleep(10)
            if(self.setTorque(0,quiet) == True):
                printStdOut("Test completed successfully.",self.__l)

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
            printStdOut("That seems a little high for a simple test. I recommend using a velocity at or below 1000 rpm.",self.__l)
            self.testVelocityMode(None)
        rsp = self.setVelocity(v)
        if rsp == True:
            printStdOut("Applying test velocity for 10 seconds...",self.__l)
            time.sleep(10)
            if(self.setVelocity(0) == True):
                printStdOut("Test completed successfully.",self.__l)

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
            printStdOut("Speed of " + str(v) + " rpm exceeds cut out speed of " + str(CUT_OUT_SPEED) + " rpm.  Killing system.",self.__l)
            self.killSystem()
        else:
            return v

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
            printStdOut("Starting dyno to " + str(vref) + " rpm.",self.__l)
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
                            printStdOut("Letting drive accelerate for %d seconds.  Current speed: %d rpm" % (initialDelay,v),True,self.__l)
                            time.sleep(samplePeriod)
                        sys.stdout.write("\n")
                        return True
                    except KeyboardInterrupt:
                        self.killSystem()
        else:
            printStdOut("Please choose a speed below 1300 rpm.",self.__l)
            self.startDyno(initialTorque,vref,initialDelay,quiet)


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
        
    def promptForPort(self):
        """
            Obtain desired port to be used.
            
            Returns:
            recursively call promptForPort if invalid port selected
            port -- the port to be used
            
            """
        printStdOut("Serial ports available:",self.__l)
        numPorts = self.scanSerial()
        port = raw_input("Select a serial port number: ")
        if port == "q":
            sys.exit()
        elif (int(port) < 0):
            printStdOut("Invalid port selected. Try again (or enter q to quit).",self.__l)
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
                printStdOut(str(int(portName[-1])-1) + ": " + portName,self.__l)
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
        printStdOut("Baud rates available:",self.__l)
        for b in availableBaud:
            printStdOut(b,self.__l)
        baud = raw_input("Select a baud rate: ")
        if baud == "q":
            sys.exit()
        elif not baud in availableBaud:
            printStdOut("Inavlid baud rate. Try again (or enter q to quit).",self.__l)
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
        printStdOut("Attempting to disconnect serial connection...",self.__l) 
        try:
            self.close()
            if(self.isOpen()):
                printStdOut("Unable to close serial port.",self.__l)
            else:
                printStdOut("Serial closed successfully.",self.__l)
                return True
        except AttributeError:
            printStdOut("Serial connection never opened.",self.__l) 
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
                    printStdOut("Read command could not be sent.",self.__l)
            else:
                printStdOut("Serial connection not open.",self.__l)
        except AttributeError:
            printStdOut("Serial connection never opened.",self.__l)
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
                                printStdOut("Byte received (" + str(ord(c)) + ") does not matc byte sent (" + str(ord(char)) + ")",self.__l)
                                return False
                        except self.SerialException:
                            printStdOut("Serial exception.",self.__l)
                            return False
                    else:
                        printStdOut("Serial port error.",self.__l)
                        return False
                return True
            else:
                printStdOut("Serial connection not open.",self.__l)
        except AttributeError:
            printStdOut("Serial connection never opened.",self.__l)
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
                                                    printStdOut(val,self.__l)
                                                    # Return err number if there is an error.
                                                    return int(val[4:6])
                                                break
            else:
                printStdOut("Serial connection not open.",self.__l)
        except AttributeError:
            printStdOut("Serial connection never opened.",self.__l)
        return False


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