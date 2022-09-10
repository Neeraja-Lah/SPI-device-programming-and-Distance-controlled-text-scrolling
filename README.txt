Assignment 4: SPI Device Programming and Distance-controlled Text Scrolling


#############################################################

Overview
********

This is an application which demonstrates distance-controlled text scrolling.
It uses the RPi's hardware SPI channel 0 for display and GPIO and IRQ pin for 
HCSR-04. The display device is configured and used in user space where as the 
HCSR-04 device is a device driver which is a loadable kernel module. It uses 
the gpio class driver to implement GPIO functionality and hardware interrupt 
to generate rising and falling edge interrupts. The use can set the display 
intensity using keyboard keys from '0-8' and also toggle blinking mode by 
pressing key 'b'. The scrolling speed is controlled dpending on the distance. 
Short distance leads to fast scrolling speed and vice versa.

Requirements
************

Raspberry Pi Model 3B+ or higher
GCC compiler
make
Raspberry Pi Kernel headers
8x8 Dot Matrix LCD with MAX7219 display driver
HC-SR04 Ultrasonic Distance Sensor
Jumper Wires
Breadboard

Connections
***********
: Connect VCC, GND, DIN, CS, CLK from display device to 3.3V (pin 1), GND (pin 6), GPIO10 MOSI (pin 19), GPIO8 CE0 (pin 24), GPIO11 SCLK (pin 23) respectively of the RPi header
: Connect VCC, TRIG, ECHO, GND from HC-SR04 distance sensor to 5V (pin 2), GPIO22 (pin 15), GPIO23 (pin 16), GND (pin 9) respectively of the RPi header

Building and Running
********************

To Build:
: Open the RPi's terminal and navigate to project folder
: Make sure raspberry pi kernel headers are installed. If not, run command 'sudo apt-get install raspberrypi-kernel-headers' to install it
: If SPI is not enabled, type command sudo raspi-config, then navigate to interface options. Go to SPI and enable it. Reboot the system after doing above steps.
: Use command 'make'. This will compile assignment4.c file and create an executable assignment4 along with kernel object files
: Use command 'sudo insmod hcsr_drv.ko' to load the HCSR-04 driver into the kernel module.

To Run:
: Use command 'sudo ./assignment4' to run the program.
: We need root access to open system files, so don't forget to use sudo.
: Press 'Ctlr+C' to terminate the program gracefully.
: Use command 'sudo rmmod hcsr_drv' to remove the kernel module.
: Use command 'make clean' to remove all build files.
: If the display does not start scrolling for the fisrt time. Terminate program using 'Ctlr+C' and run 'sudo ./assignment4' again.
: Make sure to set the object at a considerable distance before starting the program to get best results.
