##Kollmorgen ServoStar Servo Drive##

###Introduction and Purpose###
The intention of this library is to simplify the control and monitoring of Kollmorgen ServoStar Servo Drives. Originally, this code was written to faciliate research at the University of Illinois at Urbana-Champaign (UIUC), but, as it may prove to be a valuable resource to others at UIUC and elsewhere, it has been released here under the GNU General Public License version 3. While the current version of the code is not feature complete, i.e., it is not capable of utilizing all commands available on the ServoStar drive, the methods available should be sufficient for almost any application.

###Using the Library###
The ServoStar_python library uses an object-oriented programming paradigm.  If you are not familiar with objects in Python, it may be helpful to review the [Classes module](http://docs.python.org/tutorial/classes.html) in the Python tutorial before proceeding.

####Initialization####
Like any other Python library, you must first import the ServoStar library as follows (note that [pyserial](http://pyserial.sourceforge.net/) must be installed on your computer).
````python
import ServoStar
````

Once imported, to begin controlling and monitoring a servo drive, simply create an instance of the ServoDrive object.  The constructor has 9 arguments, all of which are optional.  An example of creating a new instance which places the drive in torque mode using serial port COM1 at 9600 baud is given below.  A detailed description of the arguments is given after the example.
````python
myDrive = ServoStar.ServoDrive(0,9600,2)
````

The arguments and their descriptions are given below in the order which they must appear.  Note that if you wish you pass a value as the last optional parameter, all previous parameters must be given as well, i.e., even if you wish to take the default values for parameters 1-8 but would like to specify the 9th, you must provide something in spaces 1-8.