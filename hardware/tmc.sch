EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
Title "Silent Stepper V2 Bricklet"
Date "2020-01-23"
Rev "2.0"
Comp "Tinkerforge GmbH"
Comment1 "Licensed under CERN OHL v.1.1"
Comment2 "Copyright (©) 2020, T.Schneidermann <tim@tinkerforge.com>"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L tinkerforge:MOSFET_N_CH Q?
U 1 1 5E2A2153
P 6550 2000
AR Path="/5E2A2153" Ref="Q?"  Part="1" 
AR Path="/5E2859D2/5E2A2153" Ref="Q?"  Part="1" 
AR Path="/5E2C943B/5E2A2153" Ref="Q1"  Part="1" 
F 0 "Q1" H 6741 2046 50  0000 L CNN
F 1 "CSD19533Q5A" H 6250 1850 31  0000 L CNN
F 2 "kicad-libraries:SON_5x6mm" H 6750 2100 50  0001 C CNN
F 3 "" H 6550 2000 50  0000 C CNN
	1    6550 2000
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:CONN_2 P1
U 1 1 5E2B5774
P 10950 1000
F 0 "P1" H 11078 1028 40  0000 L CNN
F 1 "VIN" H 11078 952 40  0000 L CNN
F 2 "kicad-libraries:OQ_2P_5mm_Vertical" H 10950 1000 60  0001 C CNN
F 3 "" H 10950 1000 60  0000 C CNN
	1    10950 1000
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:CONN_4 P2
U 1 1 5E2B6548
P 10800 3600
F 0 "P2" H 10928 3641 50  0000 L CNN
F 1 "OUT" H 10928 3550 50  0000 L CNN
F 2 "kicad-libraries:OQ_4P_5mm_Vertical" H 10800 3600 60  0001 C CNN
F 3 "" H 10800 3600 60  0000 C CNN
	1    10800 3600
	1    0    0    1   
$EndComp
$Comp
L tinkerforge:MOSFET_N_CH Q?
U 1 1 5E2B8D65
P 7650 2000
AR Path="/5E2B8D65" Ref="Q?"  Part="1" 
AR Path="/5E2859D2/5E2B8D65" Ref="Q?"  Part="1" 
AR Path="/5E2C943B/5E2B8D65" Ref="Q5"  Part="1" 
F 0 "Q5" H 7841 2046 50  0000 L CNN
F 1 "CSD19533Q5A" H 7350 1850 31  0000 L CNN
F 2 "kicad-libraries:SON_5x6mm" H 7850 2100 50  0001 C CNN
F 3 "" H 7650 2000 50  0000 C CNN
	1    7650 2000
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:MOSFET_N_CH Q?
U 1 1 5E2BDFF1
P 6550 2800
AR Path="/5E2BDFF1" Ref="Q?"  Part="1" 
AR Path="/5E2859D2/5E2BDFF1" Ref="Q?"  Part="1" 
AR Path="/5E2C943B/5E2BDFF1" Ref="Q2"  Part="1" 
F 0 "Q2" H 6741 2846 50  0000 L CNN
F 1 "CSD19533Q5A" H 6250 2650 31  0000 L CNN
F 2 "kicad-libraries:SON_5x6mm" H 6750 2900 50  0001 C CNN
F 3 "" H 6550 2800 50  0000 C CNN
	1    6550 2800
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:MOSFET_N_CH Q?
U 1 1 5E2BDFF7
P 7650 2800
AR Path="/5E2BDFF7" Ref="Q?"  Part="1" 
AR Path="/5E2859D2/5E2BDFF7" Ref="Q?"  Part="1" 
AR Path="/5E2C943B/5E2BDFF7" Ref="Q6"  Part="1" 
F 0 "Q6" H 7841 2846 50  0000 L CNN
F 1 "CSD19533Q5A" H 7350 2650 31  0000 L CNN
F 2 "kicad-libraries:SON_5x6mm" H 7850 2900 50  0001 C CNN
F 3 "" H 7650 2800 50  0000 C CNN
	1    7650 2800
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:MOSFET_N_CH Q?
U 1 1 5E2C4229
P 6550 4450
AR Path="/5E2C4229" Ref="Q?"  Part="1" 
AR Path="/5E2859D2/5E2C4229" Ref="Q?"  Part="1" 
AR Path="/5E2C943B/5E2C4229" Ref="Q3"  Part="1" 
F 0 "Q3" H 6741 4496 50  0000 L CNN
F 1 "CSD19533Q5A" H 6250 4300 31  0000 L CNN
F 2 "kicad-libraries:SON_5x6mm" H 6750 4550 50  0001 C CNN
F 3 "" H 6550 4450 50  0000 C CNN
	1    6550 4450
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:MOSFET_N_CH Q?
U 1 1 5E2C422F
P 7650 4450
AR Path="/5E2C422F" Ref="Q?"  Part="1" 
AR Path="/5E2859D2/5E2C422F" Ref="Q?"  Part="1" 
AR Path="/5E2C943B/5E2C422F" Ref="Q7"  Part="1" 
F 0 "Q7" H 7841 4496 50  0000 L CNN
F 1 "CSD19533Q5A" H 7350 4300 31  0000 L CNN
F 2 "kicad-libraries:SON_5x6mm" H 7850 4550 50  0001 C CNN
F 3 "" H 7650 4450 50  0000 C CNN
	1    7650 4450
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:MOSFET_N_CH Q?
U 1 1 5E2C4235
P 6550 5250
AR Path="/5E2C4235" Ref="Q?"  Part="1" 
AR Path="/5E2859D2/5E2C4235" Ref="Q?"  Part="1" 
AR Path="/5E2C943B/5E2C4235" Ref="Q4"  Part="1" 
F 0 "Q4" H 6741 5296 50  0000 L CNN
F 1 "CSD19533Q5A" H 6250 5100 31  0000 L CNN
F 2 "kicad-libraries:SON_5x6mm" H 6750 5350 50  0001 C CNN
F 3 "" H 6550 5250 50  0000 C CNN
	1    6550 5250
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:MOSFET_N_CH Q?
U 1 1 5E2C423B
P 7650 5250
AR Path="/5E2C423B" Ref="Q?"  Part="1" 
AR Path="/5E2859D2/5E2C423B" Ref="Q?"  Part="1" 
AR Path="/5E2C943B/5E2C423B" Ref="Q8"  Part="1" 
F 0 "Q8" H 7841 5296 50  0000 L CNN
F 1 "CSD19533Q5A" H 7350 5100 31  0000 L CNN
F 2 "kicad-libraries:SON_5x6mm" H 7850 5350 50  0001 C CNN
F 3 "" H 7650 5250 50  0000 C CNN
	1    7650 5250
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:Rs R2
U 1 1 5E2C5261
P 6250 2000
F 0 "R2" V 6200 2000 31  0000 C CNN
F 1 "10" V 6250 2000 31  0000 C CNN
F 2 "kicad-libraries:R0603F" H 6250 2000 60  0001 C CNN
F 3 "" H 6250 2000 60  0000 C CNN
	1    6250 2000
	0    1    1    0   
$EndComp
$Comp
L tinkerforge:Rs R12
U 1 1 5E2CC0E7
P 7350 2000
F 0 "R12" V 7300 2000 31  0000 C CNN
F 1 "10" V 7350 2000 31  0000 C CNN
F 2 "kicad-libraries:R0603F" H 7350 2000 60  0001 C CNN
F 3 "" H 7350 2000 60  0000 C CNN
	1    7350 2000
	0    1    1    0   
$EndComp
$Comp
L tinkerforge:Rs R13
U 1 1 5E2CC86B
P 7350 2800
F 0 "R13" V 7300 2800 31  0000 C CNN
F 1 "10" V 7350 2800 31  0000 C CNN
F 2 "kicad-libraries:R0603F" H 7350 2800 60  0001 C CNN
F 3 "" H 7350 2800 60  0000 C CNN
	1    7350 2800
	0    1    1    0   
$EndComp
$Comp
L tinkerforge:Rs R3
U 1 1 5E2CCD41
P 6250 2800
F 0 "R3" V 6200 2800 31  0000 C CNN
F 1 "10" V 6250 2800 31  0000 C CNN
F 2 "kicad-libraries:R0603F" H 6250 2800 60  0001 C CNN
F 3 "" H 6250 2800 60  0000 C CNN
	1    6250 2800
	0    1    1    0   
$EndComp
$Comp
L tinkerforge:Rs R4
U 1 1 5E2CD209
P 6250 4450
F 0 "R4" V 6200 4450 31  0000 C CNN
F 1 "10" V 6250 4450 31  0000 C CNN
F 2 "kicad-libraries:R0603F" H 6250 4450 60  0001 C CNN
F 3 "" H 6250 4450 60  0000 C CNN
	1    6250 4450
	0    1    1    0   
$EndComp
$Comp
L tinkerforge:Rs R5
U 1 1 5E2CE1B3
P 6250 5250
F 0 "R5" V 6200 5250 31  0000 C CNN
F 1 "10" V 6250 5250 31  0000 C CNN
F 2 "kicad-libraries:R0603F" H 6250 5250 60  0001 C CNN
F 3 "" H 6250 5250 60  0000 C CNN
	1    6250 5250
	0    1    1    0   
$EndComp
$Comp
L tinkerforge:Rs R15
U 1 1 5E2CE704
P 7350 5250
F 0 "R15" V 7300 5250 31  0000 C CNN
F 1 "10" V 7350 5250 31  0000 C CNN
F 2 "kicad-libraries:R0603F" H 7350 5250 60  0001 C CNN
F 3 "" H 7350 5250 60  0000 C CNN
	1    7350 5250
	0    1    1    0   
$EndComp
$Comp
L tinkerforge:Rs R14
U 1 1 5E2D0BD1
P 7350 4450
F 0 "R14" V 7300 4450 31  0000 C CNN
F 1 "10" V 7350 4450 31  0000 C CNN
F 2 "kicad-libraries:R0603F" H 7350 4450 60  0001 C CNN
F 3 "" H 7350 4450 60  0000 C CNN
	1    7350 4450
	0    1    1    0   
$EndComp
$Comp
L tinkerforge:MOSFET_N_CH Q?
U 1 1 5E2D2AEF
P 9850 1600
AR Path="/5E2D2AEF" Ref="Q?"  Part="1" 
AR Path="/5E2859D2/5E2D2AEF" Ref="Q?"  Part="1" 
AR Path="/5E2C943B/5E2D2AEF" Ref="Q9"  Part="1" 
F 0 "Q9" H 10041 1646 50  0000 L CNN
F 1 "CSD19533Q5A" H 9550 1450 31  0000 L CNN
F 2 "kicad-libraries:SON_5x6mm" H 10050 1700 50  0001 C CNN
F 3 "" H 9850 1600 50  0000 C CNN
	1    9850 1600
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:Rs R16
U 1 1 5E2D47EC
P 9250 1300
F 0 "R16" V 9200 1300 31  0000 C CNN
F 1 "100k" V 9250 1300 20  0000 C CNN
F 2 "kicad-libraries:R0603F" H 9250 1300 60  0001 C CNN
F 3 "" H 9250 1300 60  0000 C CNN
	1    9250 1300
	-1   0    0    1   
$EndComp
$Comp
L tinkerforge:ZENER D1
U 1 1 5E2D7C21
P 9250 1600
F 0 "D1" V 9219 1679 50  0000 L CNN
F 1 "Z12V" V 9296 1679 31  0000 L CNN
F 2 "kicad-libraries:SOD-123" H 9250 1600 50  0001 C CNN
F 3 "" H 9250 1600 50  0000 C CNN
	1    9250 1600
	0    1    1    0   
$EndComp
Wire Wire Line
	9650 1450 9650 1600
Wire Wire Line
	9250 1800 9250 1950
Wire Wire Line
	9250 1950 9600 1950
Wire Wire Line
	9950 1950 9950 1800
$Comp
L tinkerforge:GND #PWR023
U 1 1 5E2DDFAF
P 9600 1950
F 0 "#PWR023" H 9600 1700 50  0001 C CNN
F 1 "GND" H 9605 1777 50  0000 C CNN
F 2 "" H 9600 1950 50  0000 C CNN
F 3 "" H 9600 1950 50  0000 C CNN
	1    9600 1950
	1    0    0    -1  
$EndComp
Connection ~ 9600 1950
Wire Wire Line
	9600 1950 9950 1950
$Comp
L tinkerforge:CPs C21
U 1 1 5E2E38B0
P 8650 1300
F 0 "C21" H 8550 1350 31  0000 L CNN
F 1 "220uF/100V/LowESR" V 8750 1100 20  0000 L CNN
F 2 "kicad-libraries:ELKO_103" H 8650 1300 60  0001 C CNN
F 3 "" H 8650 1300 60  0000 C CNN
	1    8650 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 900  7750 1800
Wire Wire Line
	6650 900  6650 1800
Connection ~ 7750 900 
Wire Wire Line
	7950 4100 7750 4100
Wire Wire Line
	7750 4100 7750 4250
Wire Wire Line
	7750 4100 6650 4100
Wire Wire Line
	6650 4100 6650 4250
Connection ~ 7750 4100
Wire Wire Line
	6650 2200 6650 2300
Wire Wire Line
	7750 2200 7750 2500
Wire Wire Line
	6650 2300 9900 2300
Connection ~ 6650 2300
Wire Wire Line
	6650 2300 6650 2450
Wire Wire Line
	7750 2500 9700 2500
Connection ~ 7750 2500
Wire Wire Line
	7750 2500 7750 2550
$Comp
L tinkerforge:Rs R10
U 1 1 5E2F7C4E
P 7200 3450
F 0 "R10" V 7150 3450 31  0000 C CNN
F 1 "WSHM2818R0220FEA" V 7100 3450 20  0000 C CNN
F 2 "kicad-libraries:R2818" H 7200 3450 60  0001 C CNN
F 3 "" H 7200 3450 60  0000 C CNN
	1    7200 3450
	-1   0    0    1   
$EndComp
Wire Wire Line
	6650 3000 6650 3100
Wire Wire Line
	7750 3100 7750 3000
Wire Wire Line
	7750 900  7950 900 
Wire Wire Line
	6650 3100 7200 3100
Wire Wire Line
	7950 900  7950 4100
Connection ~ 7950 900 
Wire Wire Line
	9900 2300 9900 3450
Wire Wire Line
	9900 3750 9900 4950
Wire Wire Line
	9700 4750 9700 3650
Wire Wire Line
	9700 3550 9700 2500
Wire Wire Line
	6650 4650 6650 4750
Wire Wire Line
	7750 4650 7750 4700
Wire Wire Line
	6650 4750 9700 4750
Connection ~ 6650 4750
Wire Wire Line
	6650 4750 6650 4850
Wire Wire Line
	9900 4950 7750 4950
Connection ~ 7750 4950
Wire Wire Line
	7750 4950 7750 5050
$Comp
L tinkerforge:Rs R6
U 1 1 5E338026
P 6600 3350
F 0 "R6" V 6550 3350 31  0000 C CNN
F 1 "47" V 6600 3350 31  0000 C CNN
F 2 "kicad-libraries:R0603F" H 6600 3350 60  0001 C CNN
F 3 "" H 6600 3350 60  0000 C CNN
	1    6600 3350
	0    1    1    0   
$EndComp
$Comp
L tinkerforge:Rs R7
U 1 1 5E3386B3
P 6600 3550
F 0 "R7" V 6550 3550 31  0000 C CNN
F 1 "47" V 6600 3550 31  0000 C CNN
F 2 "kicad-libraries:R0603F" H 6600 3550 60  0001 C CNN
F 3 "" H 6600 3550 60  0000 C CNN
	1    6600 3550
	0    1    1    0   
$EndComp
$Comp
L tinkerforge:Rs R8
U 1 1 5E33B24C
P 6600 5800
F 0 "R8" V 6550 5800 31  0000 C CNN
F 1 "47" V 6600 5800 31  0000 C CNN
F 2 "kicad-libraries:R0603F" H 6600 5800 60  0001 C CNN
F 3 "" H 6600 5800 60  0000 C CNN
	1    6600 5800
	0    1    1    0   
$EndComp
$Comp
L tinkerforge:Rs R9
U 1 1 5E33B252
P 6600 6000
F 0 "R9" V 6550 6000 31  0000 C CNN
F 1 "47" V 6600 6000 31  0000 C CNN
F 2 "kicad-libraries:R0603F" H 6600 6000 60  0001 C CNN
F 3 "" H 6600 6000 60  0000 C CNN
	1    6600 6000
	0    1    1    0   
$EndComp
Wire Wire Line
	6650 5450 6650 5650
Wire Wire Line
	6650 5650 7200 5650
Wire Wire Line
	7750 5650 7750 5450
Wire Wire Line
	7200 5650 7200 5800
Connection ~ 7200 5650
Wire Wire Line
	7200 5650 7750 5650
Wire Wire Line
	7200 6000 7200 6100
$Comp
L tinkerforge:GND #PWR021
U 1 1 5E33F095
P 7200 6100
F 0 "#PWR021" H 7200 5850 50  0001 C CNN
F 1 "GND" H 7205 5927 50  0000 C CNN
F 2 "" H 7200 6100 50  0000 C CNN
F 3 "" H 7200 6100 50  0000 C CNN
	1    7200 6100
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:GND #PWR020
U 1 1 5E33F7A9
P 7200 3750
F 0 "#PWR020" H 7200 3500 50  0001 C CNN
F 1 "GND" H 7205 3577 50  0000 C CNN
F 2 "" H 7200 3750 50  0000 C CNN
F 3 "" H 7200 3750 50  0000 C CNN
	1    7200 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 3750 7200 3550
Wire Wire Line
	7200 3350 7200 3100
Connection ~ 7200 3100
Wire Wire Line
	7200 3100 7750 3100
Wire Wire Line
	6700 3350 7200 3350
Connection ~ 7200 3350
Wire Wire Line
	7200 3550 6700 3550
Connection ~ 7200 3550
Wire Wire Line
	6700 5800 7200 5800
Wire Wire Line
	7200 6000 6700 6000
Wire Wire Line
	6500 6000 5000 6000
Wire Wire Line
	5000 6000 5000 4400
Wire Wire Line
	5000 4400 4800 4400
Wire Wire Line
	4800 4300 5200 4300
Wire Wire Line
	5200 4300 5200 5800
Wire Wire Line
	5200 5800 6500 5800
Wire Wire Line
	7250 5250 7250 5500
Wire Wire Line
	7250 5500 5400 5500
Wire Wire Line
	5400 5500 5400 4200
Wire Wire Line
	5400 4200 4800 4200
Wire Wire Line
	4800 4100 5600 4100
Wire Wire Line
	5600 4100 5600 5250
Wire Wire Line
	5600 5250 6150 5250
Wire Wire Line
	7250 4450 7250 4000
Wire Wire Line
	7250 4000 6200 4000
Wire Wire Line
	6200 4000 6200 3600
Wire Wire Line
	6200 3600 4800 3600
Wire Wire Line
	4800 3800 6050 3800
Wire Wire Line
	6050 3800 6050 4450
Wire Wire Line
	6050 4450 6150 4450
Wire Wire Line
	4800 3500 5050 3500
Wire Wire Line
	5050 3700 4800 3700
Wire Wire Line
	7750 4700 5950 4700
Wire Wire Line
	5950 4700 5950 4000
Wire Wire Line
	5950 4000 4800 4000
Connection ~ 7750 4700
Wire Wire Line
	7750 4700 7750 4950
Wire Wire Line
	5950 4000 5950 3500
Wire Wire Line
	5950 3500 5250 3500
Connection ~ 5950 4000
Wire Wire Line
	5250 3700 5800 3700
Wire Wire Line
	5800 3700 5800 3900
Wire Wire Line
	5800 4850 6650 4850
Connection ~ 6650 4850
Wire Wire Line
	6650 4850 6650 5050
Wire Wire Line
	4800 3900 5800 3900
Connection ~ 5800 3900
Wire Wire Line
	5800 3900 5800 4850
Wire Wire Line
	4800 3000 5900 3000
Wire Wire Line
	5900 3000 5900 3350
Wire Wire Line
	5900 3350 6500 3350
Wire Wire Line
	6500 3550 6300 3550
Wire Wire Line
	6300 3550 6300 3450
Wire Wire Line
	5800 3100 4800 3100
Wire Wire Line
	7250 2800 7100 2800
Wire Wire Line
	7100 2800 7100 3050
Wire Wire Line
	7100 3050 6050 3050
Wire Wire Line
	6050 3050 6050 2900
Wire Wire Line
	6050 2900 4800 2900
Wire Wire Line
	4800 2800 6150 2800
$Comp
L tinkerforge:Cs C13
U 1 1 5E38E080
P 5150 2200
F 0 "C13" V 5100 2100 31  0000 C CNN
F 1 "470nF/25V" V 5200 2350 20  0000 C CNN
F 2 "kicad-libraries:C0603F" H 5150 2200 60  0001 C CNN
F 3 "" H 5150 2200 60  0000 C CNN
	1    5150 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	4800 2300 5700 2300
Wire Wire Line
	5700 2300 5700 1750
Wire Wire Line
	5700 1750 7250 1750
Wire Wire Line
	7250 1750 7250 2000
Wire Wire Line
	6150 2500 6150 2000
Wire Wire Line
	4800 2500 6150 2500
Wire Wire Line
	4800 2400 5050 2400
Wire Wire Line
	5050 2200 4800 2200
Wire Wire Line
	4800 2600 5550 2600
Wire Wire Line
	6250 2600 6250 2450
Wire Wire Line
	6250 2450 6650 2450
Connection ~ 6650 2450
Wire Wire Line
	6650 2450 6650 2600
Wire Wire Line
	4800 2700 5650 2700
Wire Wire Line
	6350 2700 6350 2550
Wire Wire Line
	6350 2550 7750 2550
Connection ~ 7750 2550
Wire Wire Line
	7750 2550 7750 2600
Wire Wire Line
	5250 2400 5550 2400
Wire Wire Line
	5550 2400 5550 2600
Connection ~ 5550 2600
Wire Wire Line
	5550 2600 6250 2600
Wire Wire Line
	5250 2200 5650 2200
Wire Wire Line
	5650 2200 5650 2700
Connection ~ 5650 2700
Wire Wire Line
	5650 2700 6350 2700
Wire Wire Line
	5800 3450 6300 3450
Wire Wire Line
	5800 3100 5800 3450
$Comp
L tinkerforge:Cs C17
U 1 1 5E3C589F
P 5450 1000
F 0 "C17" H 5350 1050 31  0000 C CNN
F 1 "2,2uF/100V" H 5300 900 31  0000 C CNN
F 2 "kicad-libraries:C0603F" H 5450 1000 60  0001 C CNN
F 3 "" H 5450 1000 60  0000 C CNN
	1    5450 1000
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:Cs C12
U 1 1 5E3D1ECA
P 5050 1000
F 0 "C12" H 4950 1050 31  0000 C CNN
F 1 "100nF/100V" H 4900 900 31  0000 C CNN
F 2 "kicad-libraries:C0603F" H 5050 1000 60  0001 C CNN
F 3 "" H 5050 1000 60  0000 C CNN
	1    5050 1000
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:Cs C11
U 1 1 5E3D2E15
P 4300 900
F 0 "C11" V 4150 900 31  0000 C CNN
F 1 "100nF/100V" V 4200 900 31  0000 C CNN
F 2 "kicad-libraries:C0603F" H 4300 900 60  0001 C CNN
F 3 "" H 4300 900 60  0000 C CNN
	1    4300 900 
	0    1    1    0   
$EndComp
$Comp
L tinkerforge:Cs C10
U 1 1 5E3D48A8
P 3900 1450
F 0 "C10" V 4050 1450 31  0000 C CNN
F 1 "22nF/100V" V 4000 1450 31  0000 C CNN
F 2 "kicad-libraries:C0603F" H 3900 1450 60  0001 C CNN
F 3 "" H 3900 1450 60  0000 C CNN
	1    3900 1450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3800 1450 3800 1850
Wire Wire Line
	4000 1450 4000 1850
Wire Wire Line
	4400 1850 4400 900 
Connection ~ 4400 900 
Wire Wire Line
	4200 900  4200 1850
Wire Wire Line
	5050 900  5450 900 
Connection ~ 5050 900 
Wire Wire Line
	5050 1100 5050 1300
Wire Wire Line
	5450 1300 5450 1100
$Comp
L tinkerforge:GND #PWR019
U 1 1 5E409A95
P 5400 1300
F 0 "#PWR019" H 5400 1050 50  0001 C CNN
F 1 "GND" H 5405 1127 50  0000 C CNN
F 2 "" H 5400 1300 50  0000 C CNN
F 3 "" H 5400 1300 50  0000 C CNN
	1    5400 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 900  5800 900 
Connection ~ 5450 900 
Connection ~ 6650 900 
Wire Wire Line
	6650 900  7750 900 
Wire Wire Line
	9650 1450 9250 1450
Wire Wire Line
	9250 1450 9250 1400
Connection ~ 9250 1400
$Comp
L tinkerforge:GND #PWR022
U 1 1 5E46535F
P 8500 1650
F 0 "#PWR022" H 8500 1400 50  0001 C CNN
F 1 "GND" H 8505 1477 50  0000 C CNN
F 2 "" H 8500 1650 50  0000 C CNN
F 3 "" H 8500 1650 50  0000 C CNN
	1    8500 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 3450 10150 3450
Wire Wire Line
	9700 3550 10400 3550
Wire Wire Line
	9700 3650 10400 3650
Wire Wire Line
	10450 3750 10150 3750
Wire Wire Line
	3900 4900 3900 5100
Wire Wire Line
	3900 5100 4050 5100
Wire Wire Line
	4350 5100 4350 4900
Wire Wire Line
	4200 4900 4200 5100
Connection ~ 4200 5100
Wire Wire Line
	4200 5100 4350 5100
Wire Wire Line
	4050 4900 4050 5100
Connection ~ 4050 5100
Wire Wire Line
	4050 5100 4150 5100
Wire Wire Line
	4150 5100 4150 5250
Connection ~ 4150 5100
Wire Wire Line
	4150 5100 4200 5100
$Comp
L tinkerforge:GND #PWR018
U 1 1 5E4C9F41
P 4150 5250
F 0 "#PWR018" H 4150 5000 50  0001 C CNN
F 1 "GND" H 4155 5077 50  0000 C CNN
F 2 "" H 4150 5250 50  0000 C CNN
F 3 "" H 4150 5250 50  0000 C CNN
	1    4150 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 4200 3000 4200
Wire Wire Line
	3000 4200 3000 4400
Wire Wire Line
	3000 4400 3300 4400
Wire Wire Line
	3000 4400 3000 4950
Connection ~ 3000 4400
$Comp
L tinkerforge:GND #PWR017
U 1 1 5E4DCA8B
P 3000 5100
F 0 "#PWR017" H 3000 4850 50  0001 C CNN
F 1 "GND" H 3005 4927 50  0000 C CNN
F 2 "" H 3000 5100 50  0000 C CNN
F 3 "" H 3000 5100 50  0000 C CNN
	1    3000 5100
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:Cs C9
U 1 1 5E4F21E7
P 3200 4700
F 0 "C9" V 3050 4700 31  0000 C CNN
F 1 "100nF/100V" V 3100 4700 31  0000 C CNN
F 2 "kicad-libraries:C0603F" H 3200 4700 60  0001 C CNN
F 3 "" H 3200 4700 60  0000 C CNN
	1    3200 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 4600 3200 4550
Connection ~ 3200 4550
Wire Wire Line
	3200 4550 3300 4550
Wire Wire Line
	3200 4800 3200 4950
Wire Wire Line
	3200 4950 3000 4950
Connection ~ 3000 4950
Wire Wire Line
	3000 4950 3000 5100
Text GLabel 2450 2600 0    50   Input ~ 0
CS_TMC
Text GLabel 2450 2700 0    50   Input ~ 0
SCK_TMC
Text GLabel 2450 2800 0    50   Input ~ 0
MOSI_TMC
Text GLabel 2450 2900 0    50   Output ~ 0
MISO_TMC
Wire Wire Line
	3300 3450 3000 3450
Wire Wire Line
	3000 3450 3000 4200
Connection ~ 3000 4200
Text GLabel 900  3400 0    50   Input ~ 0
DRV_ENN
$Comp
L tinkerforge:VCC #PWR012
U 1 1 5E57730E
P 1350 2300
F 0 "#PWR012" H 1350 2400 40  0001 C CNN
F 1 "VCC" H 1359 2456 40  0000 C CNN
F 2 "" H 1350 2300 60  0000 C CNN
F 3 "" H 1350 2300 60  0000 C CNN
	1    1350 2300
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:R_PACK4 RP1
U 1 1 5E58BC7C
P 1450 2800
F 0 "RP1" V 1400 2650 50  0000 L CNN
F 1 "10k" V 1500 2650 50  0000 L CNN
F 2 "kicad-libraries:0603X4" H 1450 2800 50  0001 C CNN
F 3 "" H 1450 2800 50  0000 C CNN
	1    1450 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	1500 3000 1500 3100
Wire Wire Line
	1600 3000 1600 3200
Wire Wire Line
	1800 3000 1800 3350
Wire Wire Line
	1350 2300 1350 2600
Wire Wire Line
	1350 2600 1500 2600
Connection ~ 1500 2600
Wire Wire Line
	1500 2600 1600 2600
Connection ~ 1600 2600
Wire Wire Line
	1600 2600 1700 2600
Connection ~ 1700 2600
Wire Wire Line
	1700 2600 1800 2600
$Comp
L tinkerforge:VCC #PWR015
U 1 1 5E5DA32A
P 1400 4150
F 0 "#PWR015" H 1400 4250 40  0001 C CNN
F 1 "VCC" H 1409 4306 40  0000 C CNN
F 2 "" H 1400 4150 60  0000 C CNN
F 3 "" H 1400 4150 60  0000 C CNN
	1    1400 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 4150 1400 4300
Connection ~ 1400 4300
Wire Wire Line
	1400 4300 1400 4550
Wire Wire Line
	1400 4300 3300 4300
Wire Wire Line
	1400 4550 3200 4550
Text GLabel 900  3100 0    50   Output ~ 0
DIAG1
Text GLabel 900  3200 0    50   Output ~ 0
DIAG0
NoConn ~ 1700 3000
$Comp
L tinkerforge:Rs R11
U 1 1 5E2B4404
P 7200 5900
F 0 "R11" V 7150 5900 31  0000 C CNN
F 1 "WSHM2818R0220FEA" V 7100 5900 20  0000 C CNN
F 2 "kicad-libraries:R2818" H 7200 5900 60  0001 C CNN
F 3 "" H 7200 5900 60  0000 C CNN
	1    7200 5900
	-1   0    0    1   
$EndComp
Connection ~ 7200 6000
Connection ~ 7200 5800
$Comp
L tinkerforge:Cs C14
U 1 1 5E2B9458
P 5150 2400
F 0 "C14" V 5100 2300 31  0000 C CNN
F 1 "470nF/25V" V 5200 2550 20  0000 C CNN
F 2 "kicad-libraries:C0603F" H 5150 2400 60  0001 C CNN
F 3 "" H 5150 2400 60  0000 C CNN
	1    5150 2400
	0    1    1    0   
$EndComp
$Comp
L tinkerforge:Cs C15
U 1 1 5E2B9B93
P 5150 3500
F 0 "C15" V 5100 3400 31  0000 C CNN
F 1 "470nF/25V" V 5200 3650 20  0000 C CNN
F 2 "kicad-libraries:C0603F" H 5150 3500 60  0001 C CNN
F 3 "" H 5150 3500 60  0000 C CNN
	1    5150 3500
	0    1    1    0   
$EndComp
$Comp
L tinkerforge:Cs C16
U 1 1 5E2BA111
P 5150 3700
F 0 "C16" V 5100 3600 31  0000 C CNN
F 1 "470nF/25V" V 5200 3850 20  0000 C CNN
F 2 "kicad-libraries:C0603F" H 5150 3700 60  0001 C CNN
F 3 "" H 5150 3700 60  0000 C CNN
	1    5150 3700
	0    1    1    0   
$EndComp
$Comp
L tinkerforge:Cs C18
U 1 1 5E2BBC47
P 5800 1000
F 0 "C18" H 5700 1050 31  0000 C CNN
F 1 "2,2uF/100V" H 5650 900 31  0000 C CNN
F 2 "kicad-libraries:C0603F" H 5800 1000 60  0001 C CNN
F 3 "" H 5800 1000 60  0000 C CNN
	1    5800 1000
	1    0    0    -1  
$EndComp
Connection ~ 5800 900 
Wire Wire Line
	5800 900  6200 900 
Wire Wire Line
	5800 1100 5800 1300
Wire Wire Line
	5800 1300 5450 1300
Connection ~ 5450 1300
Connection ~ 5400 1300
Wire Wire Line
	5400 1300 5450 1300
Wire Wire Line
	5050 1300 5400 1300
$Comp
L tinkerforge:CPs C19
U 1 1 5E2CF02D
P 8100 1300
F 0 "C19" H 8000 1350 31  0000 L CNN
F 1 "220uF/100V/LowESR" V 8200 1100 20  0000 L CNN
F 2 "kicad-libraries:ELKO_103" H 8100 1300 60  0001 C CNN
F 3 "" H 8100 1300 60  0000 C CNN
	1    8100 1300
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:CPs C20
U 1 1 5E2CF502
P 8350 1300
F 0 "C20" H 8250 1350 31  0000 L CNN
F 1 "220uF/100V/LowESR" V 8450 1100 20  0000 L CNN
F 2 "kicad-libraries:ELKO_103" H 8350 1300 60  0001 C CNN
F 3 "" H 8350 1300 60  0000 C CNN
	1    8350 1300
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:CPs C22
U 1 1 5E2CF915
P 8900 1300
F 0 "C22" H 8800 1350 31  0000 L CNN
F 1 "220uF/100V/LowESR" V 9000 1100 20  0000 L CNN
F 2 "kicad-libraries:ELKO_103" H 8900 1300 60  0001 C CNN
F 3 "" H 8900 1300 60  0000 C CNN
	1    8900 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 1400 8100 1650
Wire Wire Line
	8100 1650 8350 1650
Wire Wire Line
	8500 1650 8650 1650
Wire Wire Line
	8900 1650 8900 1400
Connection ~ 8500 1650
Wire Wire Line
	8650 1400 8650 1650
Connection ~ 8650 1650
Wire Wire Line
	8650 1650 8900 1650
Wire Wire Line
	8350 1400 8350 1650
Connection ~ 8350 1650
Wire Wire Line
	8350 1650 8500 1650
Wire Wire Line
	8100 1200 8100 900 
Connection ~ 8100 900 
Wire Wire Line
	8100 900  8350 900 
Wire Wire Line
	8350 900  8350 1200
Connection ~ 8350 900 
Wire Wire Line
	8350 900  8650 900 
Wire Wire Line
	8650 1200 8650 900 
Connection ~ 8650 900 
Wire Wire Line
	8650 900  8900 900 
Wire Wire Line
	8900 900  8900 1200
Connection ~ 8900 900 
$Comp
L tinkerforge:R_PACK4 RP2
U 1 1 5E3496B0
P 2850 2950
F 0 "RP2" H 2700 3400 50  0000 C CNN
F 1 "22" H 2850 3400 50  0000 C CNN
F 2 "kicad-libraries:4X0402" H 2850 2950 50  0001 C CNN
F 3 "" H 2850 2950 50  0000 C CNN
	1    2850 2950
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:R_PACK4 RP3
U 1 1 5E349E0D
P 1200 3450
F 0 "RP3" H 1050 3900 50  0000 C CNN
F 1 "22" H 1200 3900 50  0000 C CNN
F 2 "kicad-libraries:0603X4" H 1200 3450 50  0001 C CNN
F 3 "" H 1200 3450 50  0000 C CNN
	1    1200 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 2600 2650 2600
Wire Wire Line
	2650 2700 2450 2700
Wire Wire Line
	2450 2800 2650 2800
Wire Wire Line
	2650 2900 2450 2900
Wire Wire Line
	3050 2600 3300 2600
Wire Wire Line
	3300 2700 3050 2700
Wire Wire Line
	3050 2800 3300 2800
Wire Wire Line
	3300 2900 3050 2900
$Comp
L tinkerforge:Cs C5
U 1 1 5E41575C
P 2500 2050
F 0 "C5" V 2350 2050 31  0000 C CNN
F 1 "100nF/100V" V 2400 2050 31  0000 C CNN
F 2 "kicad-libraries:C0603F" H 2500 2050 60  0001 C CNN
F 3 "" H 2500 2050 60  0000 C CNN
	1    2500 2050
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:GND #PWR016
U 1 1 5E416BF0
P 2500 2300
F 0 "#PWR016" H 2500 2050 50  0001 C CNN
F 1 "GND" H 2505 2127 50  0000 C CNN
F 2 "" H 2500 2300 50  0000 C CNN
F 3 "" H 2500 2300 50  0000 C CNN
	1    2500 2300
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:Cs C6
U 1 1 5E417B7E
P 2950 1950
F 0 "C6" V 3000 2050 31  0000 C CNN
F 1 "2,2uF/16V" V 2900 2150 31  0000 C CNN
F 2 "kicad-libraries:C0603F" H 2950 1950 60  0001 C CNN
F 3 "" H 2950 1950 60  0000 C CNN
	1    2950 1950
	0    -1   -1   0   
$EndComp
$Comp
L tinkerforge:Cs C7
U 1 1 5E419CC0
P 2950 2150
F 0 "C7" V 3000 2250 31  0000 C CNN
F 1 "2,2uF/16V" V 2900 2350 31  0000 C CNN
F 2 "kicad-libraries:C0603F" H 2950 2150 60  0001 C CNN
F 3 "" H 2950 2150 60  0000 C CNN
	1    2950 2150
	0    -1   -1   0   
$EndComp
$Comp
L tinkerforge:Cs C8
U 1 1 5E41A530
P 2950 2350
F 0 "C8" V 3000 2450 31  0000 C CNN
F 1 "470nF/25V" V 2900 2550 31  0000 C CNN
F 2 "kicad-libraries:C0603F" H 2950 2350 60  0001 C CNN
F 3 "" H 2950 2350 60  0000 C CNN
	1    2950 2350
	0    -1   -1   0   
$EndComp
$Comp
L tinkerforge:Rs R1
U 1 1 5E41B0B2
P 3100 2250
F 0 "R1" V 3050 2250 31  0000 C CNN
F 1 "2,2" V 3100 2250 31  0000 C CNN
F 2 "kicad-libraries:R0603F" H 3100 2250 60  0001 C CNN
F 3 "" H 3100 2250 60  0000 C CNN
	1    3100 2250
	-1   0    0    1   
$EndComp
Wire Wire Line
	3050 2150 3100 2150
Wire Wire Line
	3050 2350 3100 2350
Wire Wire Line
	3100 2350 3200 2350
Wire Wire Line
	3200 2350 3200 2400
Wire Wire Line
	3200 2400 3300 2400
Connection ~ 3100 2350
Wire Wire Line
	3300 2300 3200 2300
Wire Wire Line
	3200 2300 3200 2150
Wire Wire Line
	3200 2150 3100 2150
Connection ~ 3100 2150
Wire Wire Line
	3300 2200 3250 2200
Wire Wire Line
	3250 2200 3250 1950
Wire Wire Line
	3250 1950 3050 1950
Wire Wire Line
	3300 2100 3300 1800
Wire Wire Line
	3300 1800 2500 1800
Wire Wire Line
	2500 1800 2500 1950
Wire Wire Line
	2500 2150 2500 2300
Wire Wire Line
	2500 2300 2600 2300
Wire Wire Line
	2700 2300 2700 2350
Wire Wire Line
	2700 2350 2850 2350
Connection ~ 2500 2300
Wire Wire Line
	2850 1950 2600 1950
Wire Wire Line
	2600 1950 2600 2150
Connection ~ 2600 2300
Wire Wire Line
	2600 2300 2700 2300
Wire Wire Line
	2600 2150 2850 2150
Connection ~ 2600 2150
Wire Wire Line
	2600 2150 2600 2300
Wire Wire Line
	2500 1800 2500 1050
Connection ~ 2500 1800
Wire Wire Line
	4400 900  5050 900 
Wire Wire Line
	6200 650  6200 900 
Wire Wire Line
	2500 650  6200 650 
Connection ~ 6200 900 
Wire Wire Line
	6200 900  6650 900 
Text GLabel 2150 1050 0    50   Output ~ 0
V_GPIO
Wire Wire Line
	2150 1050 2500 1050
Connection ~ 2500 1050
Wire Wire Line
	2500 1050 2500 650 
$Comp
L tinkerforge:TMC5160A U?
U 1 1 5E294D45
P 4100 3250
AR Path="/5E294D45" Ref="U?"  Part="1" 
AR Path="/5E2859D2/5E294D45" Ref="U?"  Part="1" 
AR Path="/5E2C943B/5E294D45" Ref="U1"  Part="1" 
F 0 "U1" H 3450 4600 50  0000 C CNN
F 1 "TMC5160A" H 4200 3200 50  0000 C CNN
F 2 "kicad-libraries:eTQFP-48_7x7mm" H 4100 3250 50  0001 C CNN
F 3 "" H 4100 3250 50  0001 C CNN
	1    4100 3250
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:CONN_5 P4
U 1 1 5E4425A0
P 2000 3800
F 0 "P4" H 1842 3385 50  0000 C CNN
F 1 "PROTO" H 1842 3476 50  0000 C CNN
F 2 "kicad-libraries:pin_array_5x1" H 2000 3800 60  0001 C CNN
F 3 "" H 2000 3800 60  0000 C CNN
	1    2000 3800
	-1   0    0    1   
$EndComp
Wire Wire Line
	2400 3600 3300 3600
Wire Wire Line
	2400 3700 3300 3700
Wire Wire Line
	3300 4050 2700 4050
Wire Wire Line
	2700 4050 2700 4000
Wire Wire Line
	2700 4000 2400 4000
$Comp
L tinkerforge:Cs C29
U 1 1 5E38203E
P 10400 3950
F 0 "C29" H 10350 3850 31  0000 C CNN
F 1 "1nF/100V" H 10500 4050 20  0000 C CNN
F 2 "kicad-libraries:C0603F" H 10400 3950 60  0001 C CNN
F 3 "" H 10400 3950 60  0000 C CNN
	1    10400 3950
	-1   0    0    1   
$EndComp
$Comp
L tinkerforge:Cs C27
U 1 1 5E385C3F
P 10150 3950
F 0 "C27" H 10100 3850 31  0000 C CNN
F 1 "1nF/100V" H 10250 4050 20  0000 C CNN
F 2 "kicad-libraries:C0603F" H 10150 3950 60  0001 C CNN
F 3 "" H 10150 3950 60  0000 C CNN
	1    10150 3950
	-1   0    0    1   
$EndComp
$Comp
L tinkerforge:Cs C26
U 1 1 5E38611B
P 10150 3200
F 0 "C26" H 10100 3100 31  0000 C CNN
F 1 "1nF/100V" H 10250 3300 20  0000 C CNN
F 2 "kicad-libraries:C0603F" H 10150 3200 60  0001 C CNN
F 3 "" H 10150 3200 60  0000 C CNN
	1    10150 3200
	-1   0    0    1   
$EndComp
$Comp
L tinkerforge:Cs C28
U 1 1 5E386826
P 10400 3200
F 0 "C28" H 10350 3100 31  0000 C CNN
F 1 "1nF/100V" H 10500 3300 20  0000 C CNN
F 2 "kicad-libraries:C0603F" H 10400 3200 60  0001 C CNN
F 3 "" H 10400 3200 60  0000 C CNN
	1    10400 3200
	-1   0    0    1   
$EndComp
Wire Wire Line
	10150 3850 10150 3750
Connection ~ 10150 3750
Wire Wire Line
	10150 3750 9900 3750
Wire Wire Line
	10400 3850 10400 3650
Connection ~ 10400 3650
Wire Wire Line
	10400 3650 10450 3650
Wire Wire Line
	10400 3550 10400 3300
Connection ~ 10400 3550
Wire Wire Line
	10400 3550 10450 3550
Wire Wire Line
	10150 3450 10150 3300
Connection ~ 10150 3450
Wire Wire Line
	10150 3450 10450 3450
Wire Wire Line
	10150 3100 10150 2950
Wire Wire Line
	10150 2950 10400 2950
Wire Wire Line
	10400 2950 10400 3100
Wire Wire Line
	10400 2950 10600 2950
Wire Wire Line
	10600 2950 10600 3000
Connection ~ 10400 2950
$Comp
L tinkerforge:GND #PWR024
U 1 1 5E4063D2
P 10600 3000
F 0 "#PWR024" H 10600 2750 50  0001 C CNN
F 1 "GND" H 10605 2827 50  0000 C CNN
F 2 "" H 10600 3000 50  0000 C CNN
F 3 "" H 10600 3000 50  0000 C CNN
	1    10600 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 4050 10150 4200
Wire Wire Line
	10150 4200 10400 4200
Wire Wire Line
	10400 4200 10400 4050
Wire Wire Line
	10400 4200 10600 4200
Wire Wire Line
	10600 4200 10600 4250
Connection ~ 10400 4200
$Comp
L tinkerforge:GND #PWR025
U 1 1 5E4339A0
P 10600 4250
F 0 "#PWR025" H 10600 4000 50  0001 C CNN
F 1 "GND" H 10605 4077 50  0000 C CNN
F 2 "" H 10600 4250 50  0000 C CNN
F 3 "" H 10600 4250 50  0000 C CNN
	1    10600 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 3350 1800 3350
Connection ~ 1600 3200
Connection ~ 1800 3350
Wire Wire Line
	1500 3100 3300 3100
Wire Wire Line
	1600 3200 3300 3200
Wire Wire Line
	1800 3350 3300 3350
Wire Wire Line
	1500 3350 1500 3400
Wire Wire Line
	1500 3400 1400 3400
Wire Wire Line
	1400 3200 1600 3200
Wire Wire Line
	1500 3100 1400 3100
Connection ~ 1500 3100
Wire Wire Line
	900  3100 1000 3100
Wire Wire Line
	1000 3200 900  3200
Wire Wire Line
	900  3400 1000 3400
NoConn ~ 1000 3300
NoConn ~ 1400 3300
Wire Wire Line
	2900 3800 2900 3850
Wire Wire Line
	2900 3850 3300 3850
Wire Wire Line
	2400 3800 2900 3800
Wire Wire Line
	3300 3950 2800 3950
Wire Wire Line
	2800 3950 2800 3900
Wire Wire Line
	2800 3900 2400 3900
$Comp
L tinkerforge:TVS D8
U 1 1 5E3A0CEA
P 10050 1150
F 0 "D8" V 9950 1000 31  0000 L CNN
F 1 "SMBJ24CA" V 9700 1000 31  0000 L CNN
F 2 "kicad-libraries:SMB" H 10050 1150 60  0001 C CNN
F 3 "" H 10050 1150 60  0000 C CNN
	1    10050 1150
	0    1    1    0   
$EndComp
Wire Wire Line
	7950 900  8100 900 
Wire Wire Line
	8900 900  9250 900 
Wire Wire Line
	10600 1400 10600 1100
Wire Wire Line
	10600 900  10300 900 
Connection ~ 9250 900 
Wire Wire Line
	9250 900  10050 900 
Wire Wire Line
	9250 900  9250 1200
Connection ~ 10050 900 
Connection ~ 10050 1400
Wire Wire Line
	10050 1400 9950 1400
$Comp
L tinkerforge:PTC PTC1
U 1 1 5E47E9B1
P 10300 1150
F 0 "PTC1" H 10200 1050 31  0000 R CNN
F 1 "VC080526C580DP" H 10400 800 31  0000 R CNN
F 2 "kicad-libraries:R0805E" H 10300 1150 60  0001 C CNN
F 3 "" H 10300 1150 60  0000 C CNN
	1    10300 1150
	-1   0    0    1   
$EndComp
Connection ~ 10300 1400
Wire Wire Line
	10300 1400 10600 1400
Connection ~ 10300 900 
Wire Wire Line
	10050 1400 10300 1400
Wire Wire Line
	10050 900  10300 900 
$EndSCHEMATC