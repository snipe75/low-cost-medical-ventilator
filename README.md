# low-cost-medical-ventilator
A low cost implementation of medical ventilator for EMERGENCY use. unfinished

Using an EDF (electric ducted fan) from an RC plane is a cheap method that allows higher airflows and pressures 
than ambu-bag approaches. 

The electronic speed controller can be powered using a computer PSU which provides 12 volts ( similar to 4s lipo battery)

For pressure measurement, a BMP280 sensor is used.
For airflow measurement, a venturi tube made from coke bottle is used, and then a MPX5010DP pressure difference sensor.
All these components are low cost, so they can be doubled-up. The program can then compare two outputs of 
same type sensor to determine if one of them is malfunctioning.

Before real use, the accuracy of the sensors must be determined with more professional tools.

![Basic idea](https://github.com/snipe75/low-cost-medical-ventilator/blob/master/Making_ventilator.jpg)

## ARDUINO PROGRAM
A program for arduino that contains PID for controlling the turbine based on desired airflow or relative
pressure. Also outputs sensor info to serial connection, for python script reading and live plotting.

## PYTHON PROGRAM
For live plotting, a python script is included. I imagine cheap old laptops could be used since the program
isn't resource intensive. 

## BEFORE REAL USE
Several components are missing for real use :
- If high pressure levels are desired, a pneumatic valve is required that CLOSES EXHALATION HOLE on patient inhalation. 
(currently, there is an EXHALATION HOLE drilled AFTER the one-way valve. So when patient exhales, air goes out there. 
However this limits maximum pressure in patient lungs since air is escaping through the EXHALATION HOLE.
- an overpressure relief valve, in case the turbine goes full throttle due to whatever reason (maybe arduino restart?)
- Alarms that trigger on overpressure, low tidal volume, long time no breath-in, power supply issues, and sensor malfunctions.
- Probably some electronic circuitry to protect arduino from whatever could happen to it.
- A battery backup in case the power goes out or PSU stops working.
