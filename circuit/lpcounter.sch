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
LIBS:lpcounter-cache
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
L ATTINY85-P IC1
U 1 1 591027B2
P 6500 3550
F 0 "IC1" H 5350 3950 50  0000 C CNN
F 1 "ATTINY85V-P" H 7450 3150 50  0000 C CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_LongPads" H 6100 3150 50  0000 C CIN
F 3 "" H 6500 3550 50  0000 C CNN
	1    6500 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 3700 4750 3700
Text Label 4800 3700 0    60   ~ 0
OUT
Wire Wire Line
	5150 3600 4750 3600
Text Label 4800 3600 0    60   ~ 0
IN3
Wire Wire Line
	4750 3500 5150 3500
Wire Wire Line
	4750 3400 5150 3400
Wire Wire Line
	5150 3300 4750 3300
Text Label 4800 3500 0    60   ~ 0
IN2
Text Label 4800 3400 0    60   ~ 0
IN1
Text Label 4800 3300 0    60   ~ 0
IN0
$Comp
L GND #PWR01
U 1 1 591028B9
P 7900 3850
F 0 "#PWR01" H 7900 3600 50  0001 C CNN
F 1 "GND" H 7900 3700 50  0000 C CNN
F 2 "" H 7900 3850 50  0000 C CNN
F 3 "" H 7900 3850 50  0000 C CNN
	1    7900 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 3800 7900 3800
Wire Wire Line
	7900 3800 7900 3850
Wire Wire Line
	7850 3300 7900 3300
Wire Wire Line
	7900 3300 7900 3250
Wire Wire Line
	4750 3800 5150 3800
Text Label 4800 3800 0    60   ~ 0
RESETn
$Comp
L +3V3 #PWR02
U 1 1 59102AB8
P 7900 3250
F 0 "#PWR02" H 7900 3100 50  0001 C CNN
F 1 "+3V3" H 7900 3390 50  0000 C CNN
F 2 "" H 7900 3250 50  0000 C CNN
F 3 "" H 7900 3250 50  0000 C CNN
	1    7900 3250
	1    0    0    -1  
$EndComp
$Comp
L D_ALT D1
U 1 1 59102ADE
P 6200 4850
F 0 "D1" H 6200 4950 50  0000 C CNN
F 1 "1N4148" H 6200 4750 50  0000 C CNN
F 2 "Diodes_SMD:MELF_Handsoldering" H 6200 4850 50  0001 C CNN
F 3 "" H 6200 4850 50  0000 C CNN
	1    6200 4850
	0    1    1    0   
$EndComp
$Comp
L +3V3 #PWR03
U 1 1 59102B87
P 6200 4700
F 0 "#PWR03" H 6200 4550 50  0001 C CNN
F 1 "+3V3" H 6200 4840 50  0000 C CNN
F 2 "" H 6200 4700 50  0000 C CNN
F 3 "" H 6200 4700 50  0000 C CNN
	1    6200 4700
	1    0    0    -1  
$EndComp
$Comp
L Battery_Cell BT1
U 1 1 59102E6E
P 6200 5200
F 0 "BT1" H 6300 5300 50  0000 L CNN
F 1 "3.7V" H 6300 5200 50  0000 L CNN
F 2 "Battery_Holders:Keystone_106_1x20mm-CoinCell" V 6200 5260 50  0001 C CNN
F 3 "" V 6200 5260 50  0000 C CNN
	1    6200 5200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 59102F4B
P 6200 5300
F 0 "#PWR04" H 6200 5050 50  0001 C CNN
F 1 "GND" H 6200 5150 50  0000 C CNN
F 2 "" H 6200 5300 50  0000 C CNN
F 3 "" H 6200 5300 50  0000 C CNN
	1    6200 5300
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 M1
U 1 1 591033D1
P 9800 3500
F 0 "M1" H 9800 3750 50  0000 C CNN
F 1 "SYN115" V 9900 3500 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.00mm" H 9800 3500 50  0001 C CNN
F 3 "" H 9800 3500 50  0000 C CNN
	1    9800 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 3350 9350 3350
Text Label 9400 3350 0    60   ~ 0
ANT
$Comp
L GND #PWR05
U 1 1 591034EC
P 9250 3450
F 0 "#PWR05" H 9250 3200 50  0001 C CNN
F 1 "GND" H 9250 3300 50  0000 C CNN
F 2 "" H 9250 3450 50  0000 C CNN
F 3 "" H 9250 3450 50  0000 C CNN
	1    9250 3450
	0    1    1    0   
$EndComp
$Comp
L +3V3 #PWR06
U 1 1 59103508
P 9250 3650
F 0 "#PWR06" H 9250 3500 50  0001 C CNN
F 1 "+3V3" H 9250 3790 50  0000 C CNN
F 2 "" H 9250 3650 50  0000 C CNN
F 3 "" H 9250 3650 50  0000 C CNN
	1    9250 3650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9250 3450 9600 3450
Wire Wire Line
	9250 3650 9600 3650
Wire Wire Line
	9600 3550 9350 3550
Text Label 9400 3550 0    60   ~ 0
OUT
$Comp
L CONN_01X08 P1
U 1 1 59103860
P 1350 3600
F 0 "P1" H 1350 4050 50  0000 C CNN
F 1 "CONN_01X08" V 1450 3600 50  0000 C CNN
F 2 "Connectors_Phoenix:PhoenixContact_MSTBVA-G_08x5.08mm_Vertical" H 1350 3600 50  0001 C CNN
F 3 "" H 1350 3600 50  0000 C CNN
	1    1350 3600
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1550 3250 2000 3250
Wire Wire Line
	1550 3450 2000 3450
Wire Wire Line
	1550 3650 2000 3650
Wire Wire Line
	1550 3850 2000 3850
$Comp
L GND #PWR07
U 1 1 59103D18
P 1550 3350
F 0 "#PWR07" H 1550 3100 50  0001 C CNN
F 1 "GND" H 1550 3200 50  0000 C CNN
F 2 "" H 1550 3350 50  0000 C CNN
F 3 "" H 1550 3350 50  0000 C CNN
	1    1550 3350
	0    -1   1    0   
$EndComp
$Comp
L GND #PWR08
U 1 1 59103D4F
P 1550 3550
F 0 "#PWR08" H 1550 3300 50  0001 C CNN
F 1 "GND" H 1550 3400 50  0000 C CNN
F 2 "" H 1550 3550 50  0000 C CNN
F 3 "" H 1550 3550 50  0000 C CNN
	1    1550 3550
	0    -1   1    0   
$EndComp
$Comp
L GND #PWR09
U 1 1 59103D6D
P 1550 3750
F 0 "#PWR09" H 1550 3500 50  0001 C CNN
F 1 "GND" H 1550 3600 50  0000 C CNN
F 2 "" H 1550 3750 50  0000 C CNN
F 3 "" H 1550 3750 50  0000 C CNN
	1    1550 3750
	0    -1   1    0   
$EndComp
$Comp
L GND #PWR010
U 1 1 59103DA4
P 1550 3950
F 0 "#PWR010" H 1550 3700 50  0001 C CNN
F 1 "GND" H 1550 3800 50  0000 C CNN
F 2 "" H 1550 3950 50  0000 C CNN
F 3 "" H 1550 3950 50  0000 C CNN
	1    1550 3950
	0    -1   1    0   
$EndComp
Text Label 1950 3250 2    60   ~ 0
IN0
Text Label 1950 3450 2    60   ~ 0
IN1
Text Label 1950 3650 2    60   ~ 0
IN2
Text Label 1950 3850 2    60   ~ 0
IN3
$Comp
L R R1
U 1 1 59104329
P 2550 3350
F 0 "R1" V 2630 3350 50  0000 C CNN
F 1 "1M" V 2550 3350 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 2480 3350 50  0001 C CNN
F 3 "" H 2550 3350 50  0000 C CNN
	1    2550 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 3500 2550 3600
$Comp
L C C1
U 1 1 59104456
P 2550 3750
F 0 "C1" H 2575 3850 50  0000 L CNN
F 1 "100nF" H 2575 3650 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 2588 3600 50  0001 C CNN
F 3 "" H 2550 3750 50  0000 C CNN
	1    2550 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 3550 2250 3550
Connection ~ 2550 3550
Text Label 2300 3550 0    60   ~ 0
IN0
$Comp
L GND #PWR011
U 1 1 5910458F
P 2550 3900
F 0 "#PWR011" H 2550 3650 50  0001 C CNN
F 1 "GND" H 2550 3750 50  0000 C CNN
F 2 "" H 2550 3900 50  0000 C CNN
F 3 "" H 2550 3900 50  0000 C CNN
	1    2550 3900
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR012
U 1 1 591045FE
P 2550 3200
F 0 "#PWR012" H 2550 3050 50  0001 C CNN
F 1 "+3V3" H 2550 3340 50  0000 C CNN
F 2 "" H 2550 3200 50  0000 C CNN
F 3 "" H 2550 3200 50  0000 C CNN
	1    2550 3200
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR013
U 1 1 5910481E
P 6700 5000
F 0 "#PWR013" H 6700 4850 50  0001 C CNN
F 1 "+3V3" H 6700 5140 50  0000 C CNN
F 2 "" H 6700 5000 50  0000 C CNN
F 3 "" H 6700 5000 50  0000 C CNN
	1    6700 5000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 59104863
P 6700 5300
F 0 "#PWR014" H 6700 5050 50  0001 C CNN
F 1 "GND" H 6700 5150 50  0000 C CNN
F 2 "" H 6700 5300 50  0000 C CNN
F 3 "" H 6700 5300 50  0000 C CNN
	1    6700 5300
	1    0    0    -1  
$EndComp
$Comp
L CP C5
U 1 1 5910488C
P 6700 5150
F 0 "C5" H 6725 5250 50  0000 L CNN
F 1 "4.7uF" H 6725 5050 50  0000 L CNN
F 2 "Capacitors_Tantalum_SMD:Tantalum_Case-R_EIA-2012-12_Hand" H 6738 5000 50  0001 C CNN
F 3 "" H 6700 5150 50  0000 C CNN
	1    6700 5150
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 59104EA6
P 3150 3350
F 0 "R2" V 3230 3350 50  0000 C CNN
F 1 "1M" V 3150 3350 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 3080 3350 50  0001 C CNN
F 3 "" H 3150 3350 50  0000 C CNN
	1    3150 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 3500 3150 3600
$Comp
L C C2
U 1 1 59104EAD
P 3150 3750
F 0 "C2" H 3175 3850 50  0000 L CNN
F 1 "100nF" H 3175 3650 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 3188 3600 50  0001 C CNN
F 3 "" H 3150 3750 50  0000 C CNN
	1    3150 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 3550 2850 3550
Connection ~ 3150 3550
Text Label 2900 3550 0    60   ~ 0
IN1
$Comp
L GND #PWR015
U 1 1 59104EB6
P 3150 3900
F 0 "#PWR015" H 3150 3650 50  0001 C CNN
F 1 "GND" H 3150 3750 50  0000 C CNN
F 2 "" H 3150 3900 50  0000 C CNN
F 3 "" H 3150 3900 50  0000 C CNN
	1    3150 3900
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR016
U 1 1 59104EBC
P 3150 3200
F 0 "#PWR016" H 3150 3050 50  0001 C CNN
F 1 "+3V3" H 3150 3340 50  0000 C CNN
F 2 "" H 3150 3200 50  0000 C CNN
F 3 "" H 3150 3200 50  0000 C CNN
	1    3150 3200
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5910501C
P 3750 3350
F 0 "R3" V 3830 3350 50  0000 C CNN
F 1 "1M" V 3750 3350 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 3680 3350 50  0001 C CNN
F 3 "" H 3750 3350 50  0000 C CNN
	1    3750 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 3500 3750 3600
$Comp
L C C3
U 1 1 59105023
P 3750 3750
F 0 "C3" H 3775 3850 50  0000 L CNN
F 1 "100nF" H 3775 3650 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 3788 3600 50  0001 C CNN
F 3 "" H 3750 3750 50  0000 C CNN
	1    3750 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 3550 3450 3550
Connection ~ 3750 3550
Text Label 3500 3550 0    60   ~ 0
IN2
$Comp
L GND #PWR017
U 1 1 5910502C
P 3750 3900
F 0 "#PWR017" H 3750 3650 50  0001 C CNN
F 1 "GND" H 3750 3750 50  0000 C CNN
F 2 "" H 3750 3900 50  0000 C CNN
F 3 "" H 3750 3900 50  0000 C CNN
	1    3750 3900
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR018
U 1 1 59105032
P 3750 3200
F 0 "#PWR018" H 3750 3050 50  0001 C CNN
F 1 "+3V3" H 3750 3340 50  0000 C CNN
F 2 "" H 3750 3200 50  0000 C CNN
F 3 "" H 3750 3200 50  0000 C CNN
	1    3750 3200
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 59105038
P 4350 3350
F 0 "R4" V 4430 3350 50  0000 C CNN
F 1 "1M" V 4350 3350 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 4280 3350 50  0001 C CNN
F 3 "" H 4350 3350 50  0000 C CNN
	1    4350 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 3500 4350 3600
$Comp
L C C4
U 1 1 5910503F
P 4350 3750
F 0 "C4" H 4375 3850 50  0000 L CNN
F 1 "100nF" H 4375 3650 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4388 3600 50  0001 C CNN
F 3 "" H 4350 3750 50  0000 C CNN
	1    4350 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 3550 4050 3550
Connection ~ 4350 3550
Text Label 4100 3550 0    60   ~ 0
IN3
$Comp
L GND #PWR019
U 1 1 59105048
P 4350 3900
F 0 "#PWR019" H 4350 3650 50  0001 C CNN
F 1 "GND" H 4350 3750 50  0000 C CNN
F 2 "" H 4350 3900 50  0000 C CNN
F 3 "" H 4350 3900 50  0000 C CNN
	1    4350 3900
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR020
U 1 1 5910504E
P 4350 3200
F 0 "#PWR020" H 4350 3050 50  0001 C CNN
F 1 "+3V3" H 4350 3340 50  0000 C CNN
F 2 "" H 4350 3200 50  0000 C CNN
F 3 "" H 4350 3200 50  0000 C CNN
	1    4350 3200
	1    0    0    -1  
$EndComp
$Comp
L LED_ALT D2
U 1 1 59105BC3
P 8800 3750
F 0 "D2" H 8800 3850 50  0000 C CNN
F 1 "ORANGE" H 8800 3650 50  0000 C CNN
F 2 "Diodes_SMD:MiniMELF_Handsoldering" H 8800 3750 50  0001 C CNN
F 3 "" H 8800 3750 50  0000 C CNN
	1    8800 3750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8800 3600 8800 3550
Wire Wire Line
	8800 3550 8550 3550
Text Label 8000 3550 0    60   ~ 0
OUT
$Comp
L R R5
U 1 1 59105DB2
P 8400 3550
F 0 "R5" V 8480 3550 50  0000 C CNN
F 1 "1k" V 8400 3550 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 8330 3550 50  0001 C CNN
F 3 "" H 8400 3550 50  0000 C CNN
	1    8400 3550
	0    1    1    0   
$EndComp
Text Label 8600 3550 0    60   ~ 0
LED
Wire Wire Line
	8250 3550 7950 3550
$Comp
L GND #PWR021
U 1 1 59106156
P 8800 3900
F 0 "#PWR021" H 8800 3650 50  0001 C CNN
F 1 "GND" H 8800 3750 50  0000 C CNN
F 2 "" H 8800 3900 50  0000 C CNN
F 3 "" H 8800 3900 50  0000 C CNN
	1    8800 3900
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR022
U 1 1 5910891F
P 7250 5000
F 0 "#PWR022" H 7250 4850 50  0001 C CNN
F 1 "+3V3" H 7250 5140 50  0000 C CNN
F 2 "" H 7250 5000 50  0000 C CNN
F 3 "" H 7250 5000 50  0000 C CNN
	1    7250 5000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR023
U 1 1 59108925
P 7250 5300
F 0 "#PWR023" H 7250 5050 50  0001 C CNN
F 1 "GND" H 7250 5150 50  0000 C CNN
F 2 "" H 7250 5300 50  0000 C CNN
F 3 "" H 7250 5300 50  0000 C CNN
	1    7250 5300
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 59108977
P 7250 5150
F 0 "C6" H 7275 5250 50  0000 L CNN
F 1 "100nF" H 7275 5050 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 7288 5000 50  0001 C CNN
F 3 "" H 7250 5150 50  0000 C CNN
	1    7250 5150
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X01 P2
U 1 1 5910624E
P 4350 5250
F 0 "P2" H 4350 5350 50  0000 C CNN
F 1 "CONN_02X01" H 4350 5150 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_SKQG" H 4350 4050 50  0001 C CNN
F 3 "" H 4350 4050 50  0000 C CNN
	1    4350 5250
	0    -1   -1   0   
$EndComp
$Comp
L R R6
U 1 1 591067B7
P 4350 4650
F 0 "R6" V 4430 4650 50  0000 C CNN
F 1 "10k" V 4350 4650 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 4280 4650 50  0001 C CNN
F 3 "" H 4350 4650 50  0000 C CNN
	1    4350 4650
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR024
U 1 1 591067BD
P 4350 4500
F 0 "#PWR024" H 4350 4350 50  0001 C CNN
F 1 "+3V3" H 4350 4640 50  0000 C CNN
F 2 "" H 4350 4500 50  0000 C CNN
F 3 "" H 4350 4500 50  0000 C CNN
	1    4350 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 4800 4350 5000
Wire Wire Line
	4350 4950 4850 4950
Connection ~ 4350 4950
$Comp
L GND #PWR025
U 1 1 59106977
P 4350 5500
F 0 "#PWR025" H 4350 5250 50  0001 C CNN
F 1 "GND" H 4350 5350 50  0000 C CNN
F 2 "" H 4350 5500 50  0000 C CNN
F 3 "" H 4350 5500 50  0000 C CNN
	1    4350 5500
	1    0    0    -1  
$EndComp
Text Label 4450 4950 0    60   ~ 0
RESETn
$EndSCHEMATC
