EESchema Schematic File Version 4
EELAYER 30 0
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
L STM32G0_SOIC8:STM32G0_SO8_T2 U2
U 1 1 5EBDEA54
P 3100 3400
F 0 "U2" H 5600 3787 60  0000 C CNN
F 1 "STM32G0_SO8_T2 Y AXIS" H 5600 3681 60  0000 C CNN
F 2 "STM32G0_SO8:STM32G0_SO8_T2" H 5600 3640 60  0001 C CNN
F 3 "" H 3100 3400 60  0000 C CNN
	1    3100 3400
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Male J3
U 1 1 5EBE07B2
P 10400 2700
F 0 "J3" H 10372 2188 50  0000 R CNN
F 1 "TO MAIN BOARD" H 10372 2279 50  0000 R CNN
F 2 "Lollo Custom:Wire_Pads_1x06_Pitch2.00mm_SMD_Pin1Right" H 10400 2700 50  0001 C CNN
F 3 "~" H 10400 2700 50  0001 C CNN
	1    10400 2700
	-1   0    0    1   
$EndComp
Text Notes 10420 2935 0    50   ~ 0
Y1
Text Notes 10420 2830 0    50   ~ 0
Y0
Text Notes 10420 2730 0    50   ~ 0
X1
Text Notes 10415 2635 0    50   ~ 0
GND
Text Notes 10420 2530 0    50   ~ 0
VCC
Text Notes 10420 2435 0    50   ~ 0
X0
Wire Wire Line
	8100 1450 9250 1450
Wire Wire Line
	9250 1450 9250 2400
Wire Wire Line
	9250 2400 10200 2400
Wire Wire Line
	8100 2500 8600 2500
Wire Wire Line
	8600 3900 8600 2500
Connection ~ 8600 2500
Wire Wire Line
	8600 2500 9000 2500
Wire Wire Line
	2550 1450 2550 2700
Wire Wire Line
	2550 2700 10200 2700
Wire Wire Line
	10200 2600 10000 2600
Wire Wire Line
	10000 2600 10000 4550
Wire Wire Line
	10000 4550 3100 4550
Wire Wire Line
	3100 4550 3100 4000
Wire Wire Line
	10000 2600 10000 800 
Wire Wire Line
	10000 800  2800 800 
Wire Wire Line
	2800 800  2800 1850
Wire Wire Line
	2800 1850 3100 1850
Connection ~ 10000 2600
$Comp
L Connector:Conn_01x03_Male J2
U 1 1 5EBEC867
P 6750 2500
F 0 "J2" H 6858 2781 50  0000 C CNN
F 1 "CAL_SW" H 6858 2690 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 6750 2500 50  0001 C CNN
F 3 "~" H 6750 2500 50  0001 C CNN
	1    6750 2500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5EBF0B1A
P 10000 800
F 0 "#PWR05" H 10000 550 50  0001 C CNN
F 1 "GND" V 10005 672 50  0000 R CNN
F 2 "" H 10000 800 50  0001 C CNN
F 3 "" H 10000 800 50  0001 C CNN
	1    10000 800 
	0    -1   -1   0   
$EndComp
Connection ~ 10000 800 
$Comp
L power:GND #PWR02
U 1 1 5EBF18C1
P 7350 2200
F 0 "#PWR02" H 7350 1950 50  0001 C CNN
F 1 "GND" V 7355 2072 50  0000 R CNN
F 2 "" H 7350 2200 50  0001 C CNN
F 3 "" H 7350 2200 50  0001 C CNN
	1    7350 2200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7750 2350 7750 2500
Wire Wire Line
	7750 2500 7250 2500
Wire Wire Line
	8100 3500 8200 3500
Wire Wire Line
	8200 3500 8200 3100
Wire Wire Line
	8200 3100 7750 3100
Wire Wire Line
	7750 3100 7750 2500
Connection ~ 7750 2500
$Comp
L Connector:Conn_01x06_Male J1
U 1 1 5EBF3CE7
P 1050 1500
F 0 "J1" H 1158 1881 50  0000 C CNN
F 1 "Serial Wire" H 1158 1790 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 1050 1500 50  0001 C CNN
F 3 "~" H 1050 1500 50  0001 C CNN
	1    1050 1500
	1    0    0    -1  
$EndComp
Text Notes 870  1430 0    50   ~ 0
X IO
Text Notes 870  1530 0    50   ~ 0
GND
Text Notes 880  1635 0    50   ~ 0
VCC
Text Notes 870  1735 0    50   ~ 0
Y IO
Text Notes 805  1835 0    50   ~ 0
Y CLK
Wire Wire Line
	3100 1550 2400 1550
Wire Wire Line
	3100 1650 2300 1650
$Comp
L power:GND #PWR01
U 1 1 5EBF7B0F
P 1250 1500
F 0 "#PWR01" H 1250 1250 50  0001 C CNN
F 1 "GND" V 1255 1372 50  0000 R CNN
F 2 "" H 1250 1500 50  0001 C CNN
F 3 "" H 1250 1500 50  0001 C CNN
	1    1250 1500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8600 2500 8600 950 
Wire Wire Line
	8600 950  2200 950 
Wire Wire Line
	2200 950  2200 1600
Wire Wire Line
	2200 1600 1250 1600
Wire Wire Line
	1250 1700 2400 1700
Wire Wire Line
	2400 1700 2400 3500
Wire Wire Line
	2400 3500 3100 3500
Wire Wire Line
	3100 3600 2300 3600
Wire Wire Line
	2300 3600 2300 1800
Wire Wire Line
	2300 1800 1250 1800
Wire Wire Line
	9100 3700 8100 3700
Wire Wire Line
	9250 3550 9250 3300
Wire Wire Line
	9250 3300 9000 3300
Wire Wire Line
	9000 3300 9000 2500
Connection ~ 9000 2500
Wire Wire Line
	9000 2500 10200 2500
Wire Wire Line
	9000 2500 9000 1300
Wire Wire Line
	9000 1300 9550 1300
Wire Wire Line
	9550 1300 9550 1600
$Comp
L power:GND #PWR03
U 1 1 5EC06F04
P 9250 3850
F 0 "#PWR03" H 9250 3600 50  0001 C CNN
F 1 "GND" H 9255 3677 50  0000 C CNN
F 2 "" H 9250 3850 50  0001 C CNN
F 3 "" H 9250 3850 50  0001 C CNN
	1    9250 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5EC07728
P 9550 1900
F 0 "#PWR04" H 9550 1650 50  0001 C CNN
F 1 "GND" H 9555 1727 50  0000 C CNN
F 2 "" H 9550 1900 50  0001 C CNN
F 3 "" H 9550 1900 50  0001 C CNN
	1    9550 1900
	1    0    0    -1  
$EndComp
Connection ~ 8100 2500
Wire Wire Line
	8100 1950 8100 2050
Wire Wire Line
	8100 3900 8600 3900
Connection ~ 3100 4000
Wire Wire Line
	3100 4000 3100 3800
Wire Wire Line
	8100 3900 8100 4000
Connection ~ 8100 3900
Connection ~ 8100 2050
Wire Wire Line
	8100 2050 8100 2500
Wire Wire Line
	3100 1850 3100 2050
$Comp
L Device:C C1
U 1 1 5EC13EEB
P 5500 1550
F 0 "C1" V 5248 1550 50  0000 C CNN
F 1 "0.1uF" V 5339 1550 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5538 1400 50  0001 C CNN
F 3 "~" H 5500 1550 50  0001 C CNN
	1    5500 1550
	0    1    1    0   
$EndComp
Connection ~ 3100 1850
Wire Wire Line
	3100 1450 2550 1450
$Comp
L STM32G0_SOIC8:STM32G0_SO8_T2 U1
U 1 1 5EBDD4C2
P 3100 1450
F 0 "U1" H 5600 1837 60  0000 C CNN
F 1 "STM32G0_SO8_T2 X AXIS" H 5600 1731 60  0000 C CNN
F 2 "STM32G0_SO8:STM32G0_SO8_T2" H 5600 1690 60  0001 C CNN
F 3 "" H 3100 1450 60  0000 C CNN
	1    3100 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C2
U 1 1 5EC198B1
P 5500 1950
F 0 "C2" V 5245 1950 50  0000 C CNN
F 1 "4.7uF" V 5336 1950 50  0000 C CNN
F 2 "Capacitors_SMD:CP_Elec_4x5.3" H 5538 1800 50  0001 C CNN
F 3 "~" H 5500 1950 50  0001 C CNN
	1    5500 1950
	0    1    1    0   
$EndComp
Wire Wire Line
	5350 2050 5350 1950
Wire Wire Line
	3100 2050 5350 2050
Wire Wire Line
	5350 1550 5350 1950
Connection ~ 5350 1950
Wire Wire Line
	5650 1550 5650 1950
Wire Wire Line
	5650 1950 5650 2050
Wire Wire Line
	5650 2050 8100 2050
Connection ~ 5650 1950
$Comp
L Device:C C3
U 1 1 5EC2919F
P 5500 3500
F 0 "C3" V 5248 3500 50  0000 C CNN
F 1 "0.1uF" V 5339 3500 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5538 3350 50  0001 C CNN
F 3 "~" H 5500 3500 50  0001 C CNN
	1    5500 3500
	0    1    1    0   
$EndComp
$Comp
L Device:CP C4
U 1 1 5EC2991A
P 5500 3900
F 0 "C4" V 5245 3900 50  0000 C CNN
F 1 "4.7uF" V 5336 3900 50  0000 C CNN
F 2 "Capacitors_SMD:CP_Elec_4x5.3" H 5538 3750 50  0001 C CNN
F 3 "~" H 5500 3900 50  0001 C CNN
	1    5500 3900
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 4000 5650 3900
Wire Wire Line
	5650 4000 8100 4000
Wire Wire Line
	5650 3900 5650 3500
Connection ~ 5650 3900
Wire Wire Line
	5350 4000 5350 3900
Wire Wire Line
	3100 4000 5350 4000
Wire Wire Line
	5350 3900 5350 3500
Connection ~ 5350 3900
$Comp
L RKJXK122000D:RKJXK122000D RV1
U 1 1 5EC50842
P 9600 1850
F 0 "RV1" H 9532 1804 50  0000 R CNN
F 1 "RKJXK122000D" H 9532 1895 50  0000 R CNN
F 2 "RKJXK122000D:RKJXK122000D" H 9600 1850 50  0001 C CNN
F 3 "~" H 9600 1850 50  0001 C CNN
	1    9600 1850
	-1   0    0    1   
$EndComp
$Comp
L RKJXK122000D:RKJXK122000D RV1
U 2 1 5EC58DF0
P 9300 3800
F 0 "RV1" H 9232 3754 50  0000 R CNN
F 1 "RKJXK122000D" H 9232 3845 50  0000 R CNN
F 2 "RKJXK122000D:RKJXK122000D" H 9300 3800 50  0001 C CNN
F 3 "~" H 9300 3800 50  0001 C CNN
	2    9300 3800
	-1   0    0    1   
$EndComp
Text Notes 7220 1580 0    50   ~ 0
AXIS_IN
Text Notes 7240 1790 0    50   ~ 0
CAL_SW
Text Notes 3700 1480 0    50   ~ 0
X1
Text Notes 7420 1480 0    50   ~ 0
X0
Text Notes 7230 3540 0    50   ~ 0
CAL_SW
Text Notes 7230 3740 0    50   ~ 0
AXIS_IN
Text Notes 7420 3430 0    50   ~ 0
Y0
Text Notes 3690 3440 0    50   ~ 0
Y1
Wire Wire Line
	8100 1750 8350 1750
Wire Wire Line
	8350 1750 8350 2350
Wire Wire Line
	8350 2350 7750 2350
Wire Wire Line
	8100 1550 8800 1550
Wire Wire Line
	8800 1550 8800 1750
Wire Wire Line
	8800 1750 9400 1750
Wire Wire Line
	9400 2950 9400 2900
Wire Wire Line
	9400 2900 10200 2900
Wire Wire Line
	8100 3400 8100 2800
Wire Wire Line
	8100 2800 10200 2800
Wire Wire Line
	9400 2950 3100 2950
Wire Wire Line
	3100 2950 3100 3400
Text Notes 800  1340 0    50   ~ 0
X CLK
Wire Wire Line
	2300 1300 1250 1300
Wire Wire Line
	2300 1300 2300 1650
Wire Wire Line
	2400 1550 2400 1400
Wire Wire Line
	2400 1400 1250 1400
$Comp
L Device:R R1
U 1 1 5EF481A2
P 7200 2200
F 0 "R1" H 7270 2246 50  0000 L CNN
F 1 "30k" H 7270 2155 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7130 2200 50  0001 C CNN
F 3 "~" H 7200 2200 50  0001 C CNN
	1    7200 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	6950 2400 7050 2400
Wire Wire Line
	7050 2400 7050 2200
$Comp
L Device:C C5
U 1 1 5EF4CFF5
P 7100 2800
F 0 "C5" V 6848 2800 50  0000 C CNN
F 1 "0.1uF" V 6939 2800 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7138 2650 50  0001 C CNN
F 3 "~" H 7100 2800 50  0001 C CNN
	1    7100 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	6950 2600 6950 2800
Wire Wire Line
	7250 2800 7250 2500
Connection ~ 7250 2500
Wire Wire Line
	8100 2650 7400 2650
Wire Wire Line
	7400 2650 7400 2900
Wire Wire Line
	7400 2900 6950 2900
Wire Wire Line
	6950 2900 6950 2800
Wire Wire Line
	8100 2500 8100 2650
Connection ~ 6950 2800
Wire Wire Line
	6950 2500 7250 2500
$EndSCHEMATC
