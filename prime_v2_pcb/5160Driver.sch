EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 6
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
L Driver_Motor:Pololu_Breakout_TMC5160 A1
U 1 1 618DA2E0
P 5400 3400
AR Path="/618D418F/618DA2E0" Ref="A1"  Part="1" 
AR Path="/6190A851/618DA2E0" Ref="A2"  Part="1" 
AR Path="/6190D054/618DA2E0" Ref="A3"  Part="1" 
F 0 "A2" H 5475 4281 50  0000 C CNN
F 1 "Pololu_Breakout_TMC5160" H 5475 4190 50  0000 C CNN
F 2 "Module:Pololu_Breakout-20_15.2x20.3mm" H 5625 2300 50  0001 L CNN
F 3 "https://www.pololu.com/product/2980/pictures" H 5500 3100 50  0001 C CNN
	1    5400 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 618DD82D
P 6800 2800
AR Path="/618DD82D" Ref="C?"  Part="1" 
AR Path="/618D418F/618DD82D" Ref="C1"  Part="1" 
AR Path="/6190A851/618DD82D" Ref="C2"  Part="1" 
AR Path="/6190D054/618DD82D" Ref="C3"  Part="1" 
F 0 "C2" H 6915 2846 50  0000 L CNN
F 1 "47uF" H 6915 2755 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 6838 2650 50  0001 C CNN
F 3 "~" H 6800 2800 50  0001 C CNN
	1    6800 2800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 618DD833
P 6800 3050
AR Path="/618DD833" Ref="#PWR?"  Part="1" 
AR Path="/618D418F/618DD833" Ref="#PWR0101"  Part="1" 
AR Path="/6190A851/618DD833" Ref="#PWR0108"  Part="1" 
AR Path="/6190D054/618DD833" Ref="#PWR0112"  Part="1" 
F 0 "#PWR0108" H 6800 2800 50  0001 C CNN
F 1 "GND" H 6805 2877 50  0000 C CNN
F 2 "" H 6800 3050 50  0001 C CNN
F 3 "" H 6800 3050 50  0001 C CNN
	1    6800 3050
	1    0    0    -1  
$EndComp
$Comp
L power:+VDC #PWR?
U 1 1 618DD839
P 6800 2550
AR Path="/618DD839" Ref="#PWR?"  Part="1" 
AR Path="/618D418F/618DD839" Ref="#PWR0105"  Part="1" 
AR Path="/6190A851/618DD839" Ref="#PWR0109"  Part="1" 
AR Path="/6190D054/618DD839" Ref="#PWR0113"  Part="1" 
F 0 "#PWR0109" H 6800 2450 50  0001 C CNN
F 1 "+VDC" H 6800 2825 50  0000 C CNN
F 2 "" H 6800 2550 50  0001 C CNN
F 3 "" H 6800 2550 50  0001 C CNN
	1    6800 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 2550 6800 2650
Wire Wire Line
	6800 2950 6800 3050
$Comp
L power:GND #PWR?
U 1 1 618DE9E3
P 5450 4750
AR Path="/618DE9E3" Ref="#PWR?"  Part="1" 
AR Path="/618D418F/618DE9E3" Ref="#PWR0106"  Part="1" 
AR Path="/6190A851/618DE9E3" Ref="#PWR0110"  Part="1" 
AR Path="/6190D054/618DE9E3" Ref="#PWR0114"  Part="1" 
F 0 "#PWR0110" H 5450 4500 50  0001 C CNN
F 1 "GND" H 5455 4577 50  0000 C CNN
F 2 "" H 5450 4750 50  0001 C CNN
F 3 "" H 5450 4750 50  0001 C CNN
	1    5450 4750
	1    0    0    -1  
$EndComp
$Comp
L power:+VDC #PWR?
U 1 1 618DF324
P 5600 2500
AR Path="/618DF324" Ref="#PWR?"  Part="1" 
AR Path="/618D418F/618DF324" Ref="#PWR0107"  Part="1" 
AR Path="/6190A851/618DF324" Ref="#PWR0111"  Part="1" 
AR Path="/6190D054/618DF324" Ref="#PWR0115"  Part="1" 
F 0 "#PWR0111" H 5600 2400 50  0001 C CNN
F 1 "+VDC" H 5600 2775 50  0000 C CNN
F 2 "" H 5600 2500 50  0001 C CNN
F 3 "" H 5600 2500 50  0001 C CNN
	1    5600 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 2500 5500 2500
Wire Wire Line
	5500 2500 5500 2700
Wire Wire Line
	5400 4500 5400 4600
Wire Wire Line
	5400 4600 5450 4600
Wire Wire Line
	5450 4600 5450 4750
Wire Wire Line
	5450 4600 5500 4600
Wire Wire Line
	5500 4600 5500 4500
Connection ~ 5450 4600
$Comp
L Connector_Generic:Conn_01x04 J?
U 1 1 618E4BC8
P 8200 3350
AR Path="/60B644A9/618E4BC8" Ref="J?"  Part="1" 
AR Path="/60C9AE58/618E4BC8" Ref="J?"  Part="1" 
AR Path="/618D418F/618E4BC8" Ref="J3"  Part="1" 
AR Path="/6190A851/618E4BC8" Ref="J6"  Part="1" 
AR Path="/6190D054/618E4BC8" Ref="J10"  Part="1" 
F 0 "J6" H 8280 3342 50  0000 L CNN
F 1 "Conn_01x04" H 8280 3251 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 8200 3350 50  0001 C CNN
F 3 "~" H 8200 3350 50  0001 C CNN
	1    8200 3350
	-1   0    0    1   
$EndComp
Text Label 8500 3150 0    50   ~ 0
S2_1A
Text Label 8500 3250 0    50   ~ 0
S2_2A
Text Label 8500 3350 0    50   ~ 0
S2_1B
Text Label 8500 3450 0    50   ~ 0
S2_2B
Wire Wire Line
	8400 3450 8500 3450
Wire Wire Line
	8500 3350 8400 3350
Wire Wire Line
	8400 3250 8500 3250
Wire Wire Line
	8500 3150 8400 3150
Text Label 6050 3250 0    50   ~ 0
S2_1A
Text Label 6050 3350 0    50   ~ 0
S2_2A
Text Label 6050 3150 0    50   ~ 0
S2_1B
Text Label 6050 3450 0    50   ~ 0
S2_2B
Wire Wire Line
	5950 3150 6050 3150
Wire Wire Line
	6050 3250 5950 3250
Wire Wire Line
	5950 3350 6050 3350
Wire Wire Line
	6050 3450 5950 3450
Text HLabel 4900 3300 0    50   Input ~ 0
EN
Text HLabel 4900 3400 0    50   Input ~ 0
STEP
Text HLabel 4900 3500 0    50   Input ~ 0
DIR
Text HLabel 4900 3900 0    50   Input ~ 0
CS
Text HLabel 4900 3800 0    50   Input ~ 0
SCK
Text HLabel 4900 3700 0    50   Input ~ 0
MOSI
Text HLabel 4900 4000 0    50   Output ~ 0
MISO
Wire Wire Line
	5000 4000 4900 4000
Wire Wire Line
	4900 3900 5000 3900
Wire Wire Line
	4900 3800 5000 3800
Wire Wire Line
	4900 3700 5000 3700
$Comp
L Connector_Generic:Conn_01x05 J11
U 1 1 618DA9E3
P 8200 4150
AR Path="/618D418F/618DA9E3" Ref="J11"  Part="1" 
AR Path="/6190A851/618DA9E3" Ref="J12"  Part="1" 
AR Path="/6190D054/618DA9E3" Ref="J14"  Part="1" 
F 0 "J12" H 8118 3725 50  0000 C CNN
F 1 "Conn_01x05" H 8118 3816 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 8200 4150 50  0001 C CNN
F 3 "~" H 8200 4150 50  0001 C CNN
	1    8200 4150
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 618DC4F6
P 5250 2400
AR Path="/618DC4F6" Ref="#PWR?"  Part="1" 
AR Path="/618D418F/618DC4F6" Ref="#PWR0117"  Part="1" 
AR Path="/6190A851/618DC4F6" Ref="#PWR0138"  Part="1" 
AR Path="/6190D054/618DC4F6" Ref="#PWR0142"  Part="1" 
F 0 "#PWR0138" H 5250 2250 50  0001 C CNN
F 1 "+3.3V" H 5265 2573 50  0000 C CNN
F 2 "" H 5250 2400 50  0001 C CNN
F 3 "" H 5250 2400 50  0001 C CNN
	1    5250 2400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 618DCDEA
P 8700 3800
AR Path="/618DCDEA" Ref="#PWR?"  Part="1" 
AR Path="/618D418F/618DCDEA" Ref="#PWR0118"  Part="1" 
AR Path="/6190A851/618DCDEA" Ref="#PWR0140"  Part="1" 
AR Path="/6190D054/618DCDEA" Ref="#PWR0148"  Part="1" 
F 0 "#PWR0140" H 8700 3650 50  0001 C CNN
F 1 "+3.3V" H 8715 3973 50  0000 C CNN
F 2 "" H 8700 3800 50  0001 C CNN
F 3 "" H 8700 3800 50  0001 C CNN
	1    8700 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 618DDBCE
P 9000 3950
AR Path="/618DDBCE" Ref="#PWR?"  Part="1" 
AR Path="/618D418F/618DDBCE" Ref="#PWR0127"  Part="1" 
AR Path="/6190A851/618DDBCE" Ref="#PWR0141"  Part="1" 
AR Path="/6190D054/618DDBCE" Ref="#PWR0149"  Part="1" 
F 0 "#PWR0141" H 9000 3700 50  0001 C CNN
F 1 "GND" H 9005 3777 50  0000 C CNN
F 2 "" H 9000 3950 50  0001 C CNN
F 3 "" H 9000 3950 50  0001 C CNN
	1    9000 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 4050 8900 4050
Wire Wire Line
	8900 4050 8900 3850
Wire Wire Line
	8900 3850 9000 3850
Wire Wire Line
	9000 3850 9000 3950
Wire Wire Line
	8700 3800 8700 3950
Wire Wire Line
	8700 3950 8400 3950
Text Label 8500 4150 0    50   ~ 0
ENCN
Text Label 8500 4250 0    50   ~ 0
ENCA
Text Label 8500 4350 0    50   ~ 0
ENCB
Wire Wire Line
	8500 4350 8400 4350
Wire Wire Line
	8400 4250 8500 4250
Wire Wire Line
	8500 4150 8400 4150
Text Label 6050 3750 0    50   ~ 0
ENCN
Text Label 6050 3850 0    50   ~ 0
ENCA
Text Label 6050 3950 0    50   ~ 0
ENCB
Wire Wire Line
	6050 3950 5950 3950
Wire Wire Line
	5950 3850 6050 3850
Wire Wire Line
	6050 3750 5950 3750
Wire Wire Line
	5250 2400 5250 2500
Wire Wire Line
	5250 2500 5400 2500
Wire Wire Line
	5400 2500 5400 2700
Wire Wire Line
	4900 3300 5000 3300
Wire Wire Line
	5000 3400 4900 3400
Wire Wire Line
	4900 3500 5000 3500
$EndSCHEMATC
