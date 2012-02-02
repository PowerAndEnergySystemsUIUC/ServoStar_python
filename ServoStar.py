'''
Copyright 2012 Stanton T. Cady
		
ServoStart_python  v0.1 -- January 26, 2012
		
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
	
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

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
global currentScalingFactor
currentScalingFactor = 50 # drive command/amps
global torqueConstant
torqueConstant = 1.3975964 # Amps/Nm
global constantCoefficent
constantCoefficient = 0.1262042969
global linearCoefficient
linearCoefficient = 0.0000535541
global quadraticCoefficient
quadraticCoefficient = -0.0000000074
global cutOutSpeed
cutOutSpeed = 1350
global torqueLimit
torqueLimit = 6

def sendCommand(ser,command):
	command = command + chr(0x0D)
	try:
		if(ser.isOpen()):
			ser.flushOutput						# clear the serial output buffer
			ser.flushInput
			ser.read(ser.inWaiting())			# clear anything that is in waiting
			for char in command:				# do for each character of command string
				if(ser.write(char) > 0):		# send character out serial port
					try:
						c = ser.read()
						if(c != char):		# read a byte from the serial port
							print "Byte received (" + str(ord(c)) + ") does not match byte sent (" + str(ord(char)) + ")"
							return False
					except ser.SerialException:
						print "Serial exception."
						return False
				else:
					print "Serial port error."
					return False
			return True
		else:
			print "Serial connection not open."
	except AttributeError:
		print "Serial connection never opened."
	return False
	
def verifyCommand(ser):
	try:
		if(ser.isOpen()):
			if(ser.read() == chr(0x0A)):
				c = ser.read()
				if(c == '-'):
					if(ser.read() == '-'):
						if(ser.read() == '>'):
							return True
				elif(c == chr(0x07)):
					val = str()
					while True:
						b = ser.read()
						if(b != chr(0x0D)):
							val = val + b
						else:
							if(ser.read() == chr(0x0A)):
								if(ser.read() == '-'):
									if(ser.read() == '-'):
										if(ser.read() == '>'):
											if val[:3] == 'ERR':
												print val
												# return err number if there is an error
												return int(val[4:6])
											break
		else:
			print "Serial connection not open."
	except AttributeError:
		print "Serial connection never opened."
	return False
	
def sendWriteCommand(l,ser,command):
	if(sendCommand(l,ser,command)):
		return verifyCommand(l,ser)
		
def sendReadCommand(l,ser,command):
	try:
		if(ser.isOpen()):
			if(sendCommand(l,ser,command)):
				if(ser.read() == chr(0x0A)):
					val = str()
					while True:
						b = ser.read()
						if(b != chr(0x0D)):
							val = val + b
						else:
							if(ser.read() == chr(0x0A)):
								if(ser.read() == '-'):
									if(ser.read() == '-'):
										if(ser.read() == '>'):
											return val                                    
			else:
				printStdOut(l,"Read command could not be sent.")
		else:
			printStdOut(l,"Serial connection not open.")
	except AttributeError:
		printStdOut(l,"Serial connection never opened.")
	return False

def getTwoByteNum(l,ser):
	b = ser.read()
	val = int(ord(b)) << 8
	b = ser.read()
	val = int(ord(b)) + val
	b = ser.read()
	if b != chr(0x0D):
		printStdOut(l,"Did not receive carriage return.")
		return False
	else:
		return float(val)
		
def sendTwoByteNum(l,ser,payload):
	try:
		byteOne = chr(int(payload/256))
		byteTwo = chr(int(payload % 256))
		ser.write(byteOne)
		ser.write(byteTwo)
		#printStdOut(l,"Payload sent successfully. Byte one: " + str(ord(byteOne)) + "; Byte two: " + str(ord(byteTwo)))
	except ValueError:
		printStdOut(l,"Could not send data, value of payload out of range.")

def sendOneChar(l,ser,payload):
	try:
		ser.write(payload)
	except ValueError:
		printStdOut(l,"Could not send data, value of payload out of range.")	

def checkActive(l,ser):
	rsp = sendReadCommand(l,ser,'active')
	if(int(rsp) == 1):
		return True
	return rsp

def driveOk(l,ser):
	rsp = sendReadCommand(l,ser,'driveok')
	if(int(rsp) == 1):
		return True
	return rsp
	
def driveReady(l,ser):
	rsp = sendReadCommand(l,ser,'ready')
	if(int(rsp) == 1):
		return True
	return rsp
	
def getVelocity(l,ser):
	return int(sendReadCommand(l,ser,'v'))
	
def getCurrent(l,ser):
	return sendReadCommand(l,ser,'i')
	
def getTorque(l,ser):
	global currentScalingFactor
	global torqueConstant
	return float(getCurrent(l,ser))/(currentScalingFactor*torqueConstant)
		
def enableDrive(l,ser):
	rsp = sendWriteCommand(l,ser,'en')
	if(rsp == True):
		printStdOut(l,"Drive enabled successfully.")
		return True
	printStdOut(l,"There was an error enabling the drive.")
	return rsp

def disableDrive(l,ser):
	rsp = sendWriteCommand(l,ser,'dis')
	if(rsp == True):
		printStdOut(l,"Drive disabled succesfully.")
		return True
	printStdOut(l,"There was an error disabling the drive.")
	return rsp
		
def setOpmode(l,ser,mode):
	if(mode == 0 or mode == 2):
		rsp = sendWriteCommand(l,ser,'opmode=' + str(mode))
		if(rsp == True):
			printStdOut(l,"Opmode changed successfully.")
			return True
		else:
			printStdOut,("There was an error setting the opmode.")
			return rsp
	else:
		printStdOut(l,"Valid opmodes are 0 or 2.")
	return False
	
def setVelocity(l,ser,velocity):
	if dynoMode == 1:
		try:
			if (float(velocity) >= 0 or float(velocity) <= 1300):
				rsp = sendWriteCommand(l,ser,'j=' + str(velocity))
				if(rsp == True):
					printStdOut(l,"Velocity command sent successfully.")
					return True
				else:
					printStdOut(l,"There was an error commanding the specified velocity.")
					return rsp
			else:
				printStdOut(l,"Velocity must be between 0 and 1300 RPM.")
		except ValueError:
			printStdOut(l,"Invalid velocity command.")
	return False
	
def setTorque(l,ser,torque):
	global currentScalingFactor
	global torqueConstant
	if dynoMode == 2:
		try:
			if(float(torque) >= 0 or float(torque) <= 6):
				# the drive torque command is actually a current command
				i = str(int(round(float(torque)*currentScalingFactor*torqueConstant,0)))
				rsp = sendWriteCommand(l,ser,'t=' + i)
				if(rsp == True):
					return True
				else:
					printStdOut(l,"There was an error commanding the specified torque.")
					return rsp
			else:
				printStdOut(l,"Torque must be between 0 and 6 N-m.")
		except ValueError:
			printStdOut(l,"Invalid torque command.")
	else:
		printStdOut(l,"Dynamometer is in an unknown mode.")
	return False
	
def changeTorque(l, f, ser, t, v, msg = ""):
	global torqueLimit
	if t > 0:
		# attempt to change torque command
		if setTorque(l,ser,t) == True:
			# print speed and torque command to stdout
			printStdOut(l,"Speed: %i rpm; Te: %0.2f Nm; Tm: %0.2f Nm" % (v,t,t-round(computeLossTorque(v),2)),True)
			# log torque command and velocity
			# parameters: lock, file, torque, velocity
			logTorqueVelocity(l,f,t,v,msg)
			return True
		else:
			printStdOut(l,"There was an error changing the torque.")
	else:
		printStdOut(l,"Negative torques cannot be applied right now.")
	return False
	
def computeLossTorque(v):
	global torqueConstant
	return computeLossCurrent(v)/torqueConstant;
	
def computeLossCurrent(v):
	global constantCoefficent
	global linearCoefficient
	global quadraticCoefficient
	return constantCoefficient + v*linearCoefficient + math.pow(v,2)*quadraticCoefficient

def setCurrentLimit(l,ser,limit):
	rsp = sendWriteCommand(l,ser,'ilim=' + str(limit))
	if(rsp == True):
		printStdOut(l,"Current limit successfully set.")
	printStdOut(l,"There was an error setting the current limit.")
	return rsp

def setTorqueLimit(l,ser,limit):
	rsp = sendWriteCommand(l,ser,'ilim=' + str(int(round(float(limit)*currentScalingFactor*torqueConstant))))
	if(rsp == True):
		printStdOut(l,"Torque limit successfully set.")
		return True
	printStdOut(l,"There was an error setting the torque limit.")
	return rsp
	
def openSerial(l,port,baud):
	printStdOut(l,"Connecting...")
	ser = serial.Serial(int(port),long(baud),timeout=5)
	if (ser.isOpen()):
		ser.flushInput()
		printStdOut(l,"Serial opened successfully.")
		return ser
	else:
		printStdOut(l,"Unable to open serial port.") 
	return False

def closeSerial(l,ser):
	printStdOut(l,"Attempting to disconnect serial connection...") 
	try:
		ser.close()
		if(ser.isOpen()):
			printStdOut(l,"Unable to close serial port.")
		else:
			printStdOut(l,"Serial closed successfully.")
			return True
	except AttributeError:
		printStdOut(l,"Serial connection never opened.") 
	return False

def scanSerial():
	path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
	key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
	for i in itertools.count():
		try:
			val = winreg.EnumValue(key, i)
			portName = str(val[1])
			print str(int(portName[-1])-1) + ": " + portName
		except EnvironmentError:
			break
	return i-1

def promptForSerial():
	print "Serial ports available:"
	numPorts = scanSerial()
	port = raw_input("Select a serial port number: ")
	if port == "q":
		sys.exit()
	elif (int(port) < 0):
		print "Invalid port selected. Try again (or enter q to quit)."
		return promptSorSerial()
	else:
		port = port.strip()
		return port

def promptForBaud(device):
	if device == "dyno":
		availableBaud = ['9600','19200']
	elif device == "arduino":
		availableBaud = ['9600','19200','28800','38400','57600','115200']
	else:
		availableBaud = ['9600']
	print "Baud rates available:"
	for baud in availableBaud:
		print baud
	baud = raw_input("Select a baud rate: ")
	if baud == "q":
		sys.exit()
	elif not baud in availableBaud:
		print "Inavlid baud rate.  Try again (or enter q to quit)."
		return promptForBaud()
	else:
		return baud

def killSystem(l, dyno = None, arduino = None, exit = True):
	if dyno != None and dyno.isOpen():
		# attempt to disable drive until successful
		try:
			while disableDrive(l,dyno) != True:
				time.sleep(0.1)
		except KeyboardInterrupt:
			sys.exit()
		closeSerial(l,dyno)
	if arduino != None:
		closeSerial(l,arduino)
	if exit == True:
		sys.exit()

def enableDyno(l, ser, mode = -1, torqueLimit = 6, testTorque = True):
	global dynoMode
	# if no mode is given, ask for one
	if mode == -1:
		l.acquire()
		mode = raw_input("Which mode should the dyno be placed in (1) velocity mode or (2) torque mode? ")
		l.release()
		mode = int(mode)
	if mode == 1:
		return enableVelocityMode(l,ser)
	elif mode == 2:
		# attempt to enable torque mode
		rsp = enableTorqueMode(l,ser,torqueLimit,testTorque)
		if rsp == True:
			dynoMode = 2
		# look for easily fixable errors
		elif rsp == 5:
			# a response code of 5 can usually be fixed by trying again
			time.sleep(0.1)
			return enableDyno(l,ser,mode,torqueLimit,testTorque)
		elif rsp == 23:
			disableDrive(l,ser)
			return enableDyno(l,ser,mode,torqueLimit,testTorque)
		return rsp
	else:
		printStdOut(l,"Inavlid mode.")
	return False
	
def enableVelocityMode(l,ser):
	rsp = disableDrive(l,ser)
	if(rsp == True):
		time.sleep(0.1)
		rsp = setOpmode(l,ser,1)
		if(rsp == True):
			time.sleep(0.1)
			rsp = enableDrive(l,ser)
			if(rsp == True):
				# change global variable storing dyno mode to 1 (velocity mode)
				global dynoMode
				dynoMode = 1
				testVelocityMode = raw_input("Dynamometer set to velocity mode.  Shall I test the drive? (y/n) ")
				if testVelocityMode == "y":
					testVelocity = raw_input("What velocity would you like to use for the test? ")
					if(float(testVelocity) > 1000):
						printStdOut(l,"That seems a little high for a simple test. I recommend using a velocity at or below 1000 rpm.")
						testVelocity = raw_input("Enter a new velocity to test: ")
					if(setVelocity(l,ser,testVelocity)):
						printStdOut(l,"Spinning the drive at the test velocity for 10 seconds...")
						time.sleep(10)
						if(setVelocity(l,ser,0)):
							printStdOut(l,"Test completed successfully.")
				printStdOut(l,"The drive should be ready to accept velocity commands.")
				return True
	printStdOut(l,"There was an error enabling velocity mode.")
	return False
	
def enableTorqueMode(l,ser, torqueLimit = None, testTorque = True):
	rsp = disableDrive(l,ser)
	if(rsp == True):
		time.sleep(0.1)
		rsp = setOpmode(l,ser,2)
		if(rsp == True):
			time.sleep(0.1)
			rsp = enableDrive(l,ser)
			if(rsp == True):
				if(torqueLimit != None):
					time.sleep(0.1)
					if(setTorqueLimit(l,ser,torqueLimit) == True):
						printStdOut(l,"The torque limit has been set.")
				active = checkActive(l,ser)
				time.sleep(0.1)
				driveok = driveOk(l,ser)
				if active == True and driveok == True:
					printStdOut(l,"The drive should be ready to accept torque commands.")
					if testTorque == True:
						q = raw_input("Shall I test the drive? (y/n) ")
						if q == 'y':
							testTorqueMode(l,ser)
					return True
				else:
					rsp = "Active response: " + str(active) + "; Driveok response: " + str(driveok)
	printStdOut(l,"There was an error enabling torque mode.")
	return rsp
	
def testTorqueMode(l,ser, t = None):
	if t == None:
		t = raw_input(" What torque would you like to apply for the test? ")
		t = float(t)
	if t > 1:
		printStdOut(l,"That seems a little high for a simple test. I recommend using a torque at or below 1 Nm.")
		testTorqueMode(l,ser)
	rsp = setTorque(l,ser,t)
	if rsp == True:
		printStdOut(l,"Applying test torque for 10 seconds...")
		time.sleep(10)
		if(setTorque(l,ser,0) == True):
			printStdOut(l,"Test completed successfully.")

def checkSpeed(l,f,ser,va,t):
	v = float(getVelocity(l,ser))
	if v > cutOutSpeed:
		printStdOut(l,"Speed of " + str(v) + " rpm exceeds cut out speed of " + str(cutOutSpeed) + " rpm.  Killing system.")
		killSystem(l,ser)
	else:
		va.value = int(v)
		logTorqueVelocity(l,f,t,v)
		return v
	
def setupDynoSerial(l, port = None, baud = None):
	# check if parameters for serial port and baud rate have been passed in
	if port == None or baud == None:
		# missing port or baud so ask for them
		print "Attempting to setup the dynamometer."
		q = raw_input("Shall I attempt to connect to the drive using default settings? (y/n): ")
		if port == None:
			if q == 'y':
				port = 0
			else:
				port = promptForSerial()
		if baud == None:
			if q == 'y':
				baud = 9600
			else:
				baud = promptForBaud("dyno")
	printStdOut(l,"Attempting to setup the dynamometer.")
	# attempt to open a serial connection to the dyno
	return openSerial(l,port,baud)

def startDyno(l, f, ser, va, initialTorque = None, vref = None, initialDelay = None):
	if vref == None:
		vref = 1000
		#initialSpeed = int(raw_input("What velocity in rpm would like to use? "))
	if vref < 1300:
		printStdOut(l,"Starting dyno to " + str(vref) + " rpm.")
		if initialTorque == None:
			t = 0
		else:
			t = initialTorque
			if initialDelay == None:
				initialDelay = 15
			if initialDelay > 0:
				changeTorque(l,f,ser,t,0,"START")
				samplePeriod = 0.2
				try:
					for i in range(int(initialDelay/samplePeriod)):
						v = checkSpeed(l,f,ser,va,t)
						printStdOut(l,"Letting drive accelerate for %d seconds.  Current speed: %d rpm" % (initialDelay,v),True)
						time.sleep(samplePeriod)
					sys.stdout.write("\n")
					return True
				except KeyboardInterrupt:
					killSystem(l,ser)
	else:
		print "Please choose a speed below 1300 rpm."
		startDyno(l,ser)

def printStdOut(l, msg, cr = False):
	global crLast
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
	l.release()