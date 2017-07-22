EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATTINY85-P IC?
U 1 1 590CE8E1
P 6950 3800
F 0 "IC?" H 5800 4200 50  0000 C CNN
F 1 "ATTINY85-P" H 7950 3400 50  0000 C CNN
F 2 "DIP8" H 7950 3800 50  0000 C CIN
F 3 "" H 6950 3800 50  0000 C CNN
	1    6950 3800
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 590CEA45
P 6050 2750
F 0 "R?" V 6130 2750 50  0000 C CNN
F 1 "R" V 6050 2750 50  0000 C CNN
F 2 "" V 5980 2750 50  0000 C CNN
F 3 "" H 6050 2750 50  0000 C CNN
	1    6050 2750
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 590CEAAA
P 6500 2750
F 0 "R?" V 6580 2750 50  0000 C CNN
F 1 "R" V 6500 2750 50  0000 C CNN
F 2 "" V 6430 2750 50  0000 C CNN
F 3 "" H 6500 2750 50  0000 C CNN
	1    6500 2750
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 590CEB9F
P 5650 2750
F 0 "R?" V 5730 2750 50  0000 C CNN
F 1 "R" V 5650 2750 50  0000 C CNN
F 2 "" V 5580 2750 50  0000 C CNN
F 3 "" H 5650 2750 50  0000 C CNN
	1    5650 2750
	1    0    0    -1  
$EndComp
$Comp
L Battery_Cell BT?
U 1 1 590CECF4
P 8000 3150
F 0 "BT?" H 8100 3250 50  0000 L CNN
F 1 "Battery_Cell" H 8100 3150 50  0000 L CNN
F 2 "" V 8000 3210 50  0000 C CNN
F 3 "" V 8000 3210 50  0000 C CNN
	1    8000 3150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5650 2900 6500 2900
Connection ~ 6050 2900
Wire Wire Line
	6050 2900 6050 3000
Wire Wire Line
	6050 3000 8100 3000
Wire Wire Line
	8100 3000 8100 3150
Wire Wire Line
	8100 3150 8500 3150
Wire Wire Line
	8500 3150 8500 4050
Wire Wire Line
	8500 4050 8300 4050
Wire Wire Line
	8300 3300 8300 3550
Wire Wire Line
	7800 3300 8300 3300
Wire Wire Line
	7800 3300 7800 3150
Wire Wire Line
	7800 3150 5000 3150
Wire Wire Line
	5000 3150 5000 2150
$EndSCHEMATC