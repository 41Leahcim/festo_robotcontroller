# Examples
The examples are applications used to test the code and show the user how they can use the library.

## Device search
This is the application you should run first on the Ethercat network. It displays properties of the devices on the network that can be used to control those devices with the other example. The devices are separated with an empty line.

### Usage
This application needs the to the Ethercat network connected interface to be passed as argument. As this application needs to perform direct IO on the interface, it may need administrator or root priviledge to run.

## Single servo
This application can be used to control a servo or stepper motor of choice. This application is currently limited to homing and moving the servo a little bit in the positive direction and back.

### Usage
Like the device search application, this application needs the Ethercat interface to be passed as argument. To select a device, the device number, alias address, configured address, and/or description can be passed as argument. The first found device conforming to those requirements will be controlled. This means that the first device will be used if no device selection arguments have been passed. The argument flags for this application are:
- "-n" for the device number (32-bit or 64-bit integer)
- "-a" for alias address (16-bit integer)
- "-c" for configured address (16-bit integer)
- "-d" for description (text)
- "-h" or "--help" for a help message about how to use the application

The interface will be the argument not preceded by a flag.