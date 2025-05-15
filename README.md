# PiRaTe - The Pi Radio Telescope

## Overview
This is an all-in-one control system for an Az/Alt mount radio telescope originally written for the 3m radio antenna at the astronomical observatory in Radebeul/Dresden (Germany) but may be utilized for similar instruments. 
The main features are:
- Based on Raspberry Pi (tested for model 4) connecting via GPIOs to easy-to-acquire ubiquitous periphery modules for motor control, position sensing, GPS reception, I/O, ADCs etc. The emplyed components and connections are documented here: https://oshwlab.com/antrares/pirate-mainboard
- Abstraction to the hardware is managed through a custom Instrument-Neutral-Device-Interface (INDI) driver, which allows access to the scope through the quasi-standard XML-message-based INDI protocol. INDI is utilized in many remote observatories.
- An observation task manager (Radio Telescope Task Scheduler - RaTSche) which runs as daemon and manages tasks such as measurements, parking, maintenance etc. New tasks are added through a message queue interface e.g. through the included command-line-interface (CLI) client which allows easy integration into web-based interfaces.
- A collection of bash scripts which allow for more complex measurement tasks, i.e. 2d scans in horizontal and equatorial coordinates, transit scans, tracking measurements. The scripts are installed through CMake in the system's binary location together with indi-pirt and ratsche and are invoked by the ratsche daemon when required.
- MQTT bridges which translate the indi and ratsche messages into an MQTT telemetry stream and vice versa: indi<->MQTT, ratsche->MQTT and MQTT->ratsche
- systemd service files for indiserver, ratsche daemon and the MQTT bridges to have the complete control chain started at boot time. These are installed into the according system locations by the cmake scripts, too.

## Howto build and install PiRaTe
First, you need all the packages installed, that the driver depends on. Copy&Pasting following command line should do this job:

`sudo apt install libindi-dev libgpiod-dev libnova-dev libgsl-dev libudev-dev`

Now, build and install the project:
- checkout the main branch of this git repository: `git clone https://github.com/hangeza/indi-rpi-radiotelescope.git`
- in the project dir `mkdir build && cd build`
- `cmake ../`
- `make`
- `sudo make install`

The binaries, shell scripts and systemd unit files should then be installed in the appropriate system locations.

## Prerequisites
The driver does not depend on another service. However, to be fully functional, several udev devices have to be made available through dtoverlay. Make sure to have the following lines added to /boot/config.txt:
```
#enable primary i2c bus
dtparam=i2c_arm=on
#disable standard spi configuration
dtparam=spi=off
#enable spi0 master with cs pins diverted to unused and unconnected gpio pins
dtoverlay=spi0-cs,cs0_pin=46,cs1_pin=47
#enable spi6 instead of mini-spi spi1 which can not handle spi mode 3
dtoverlay=spi6-1cs,cs0_pin=48
#Switch off usage of ID EEPROM (GPIO0 and GPIO1)
force_eeprom_read=0
#enable 1Wire
dtoverlay=w1-gpio,gpiopin=4,pullup=on
#enable UART0 for GNSS communication
enable_uart=1
init_uart_baud=9600
#enable GPS PPS on GPIO18
dtoverlay=pps-gpio,gpiopin=18
#preset Relay GPIOs
gpio=17,27,22,16=op,dh
#enable hardware pwm on pins 12 and 13
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4

```

## Activate and Start PiRaTe
Now activate the driver and ratsche via systemd services:

    sudo systemctl enable --now indiserver.service
    sudo systemctl enable --now ratsche.service

These steps have to be executed only once. The indi driver as well as the task scheduler should now be running which may be checked by:

    systemctl status indiserver.service
    systemctl status ratsche.service

The output should not contain any critical messages up to this point. Also, when installing the software suite through the make install command (see above), the peripheral service scripts for mqtt communication should be invoked automatically. Check with 

    systemctl status rt_mqtt.service|rt_mqtt_to_ratsche.service|rt_ratsche_to_mqtt.service


