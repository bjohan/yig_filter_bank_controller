EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 7
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
L 74xx:74HC595 U?
U 1 1 61DAC30A
P 5050 2400
AR Path="/61DAC30A" Ref="U?"  Part="1" 
AR Path="/61DA1D9E/61DAC30A" Ref="U12"  Part="1" 
F 0 "U12" H 5050 3181 50  0000 C CNN
F 1 "74HC595" H 5050 3090 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 5050 2400 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 5050 2400 50  0001 C CNN
	1    5050 2400
	1    0    0    -1  
$EndComp
$Comp
L Transistor_Array:ULN2003A U14
U 1 1 61DAC86F
P 6450 2200
F 0 "U14" H 6450 2867 50  0000 C CNN
F 1 "ULN2003A" H 6450 2776 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 6500 1650 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/uln2003a.pdf" H 6550 2000 50  0001 C CNN
	1    6450 2200
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC595 U?
U 1 1 61DAE926
P 4950 4050
AR Path="/61DAE926" Ref="U?"  Part="1" 
AR Path="/61DA1D9E/61DAE926" Ref="U11"  Part="1" 
F 0 "U11" H 4950 4831 50  0000 C CNN
F 1 "74HC595" H 4950 4740 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 4950 4050 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 4950 4050 50  0001 C CNN
	1    4950 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 2700 5700 2700
Wire Wire Line
	5700 2700 5700 3200
Wire Wire Line
	5700 3200 4300 3200
Wire Wire Line
	4300 3200 4300 3650
Wire Wire Line
	4300 3650 4550 3650
Wire Wire Line
	4550 3850 4200 3850
Wire Wire Line
	4200 3850 4200 2200
Wire Wire Line
	4200 2200 4650 2200
$Comp
L Transistor_Array:ULN2003A U13
U 1 1 61DB0D1B
P 6350 3750
F 0 "U13" H 6350 4417 50  0000 C CNN
F 1 "ULN2003A" H 6350 4326 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 6400 3200 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/uln2003a.pdf" H 6450 3550 50  0001 C CNN
	1    6350 3750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0163
U 1 1 61DB50DE
P 6350 4450
F 0 "#PWR0163" H 6350 4200 50  0001 C CNN
F 1 "GND" H 6355 4277 50  0000 C CNN
F 2 "" H 6350 4450 50  0001 C CNN
F 3 "" H 6350 4450 50  0001 C CNN
	1    6350 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 3200 5700 3550
Wire Wire Line
	5700 3550 5950 3550
Connection ~ 5700 3200
Wire Wire Line
	4650 2000 3750 2000
Wire Wire Line
	4200 2200 3750 2200
Connection ~ 4200 2200
Wire Wire Line
	4650 2600 4000 2600
$Comp
L Device:R R42
U 1 1 61DBA256
P 4200 2300
F 0 "R42" V 3993 2300 50  0000 C CNN
F 1 "R" V 4084 2300 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 4130 2300 50  0001 C CNN
F 3 "~" H 4200 2300 50  0001 C CNN
	1    4200 2300
	0    1    1    0   
$EndComp
$Comp
L Device:R R43
U 1 1 61DBA67C
P 4200 3950
F 0 "R43" V 3993 3950 50  0000 C CNN
F 1 "R" V 4084 3950 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 4130 3950 50  0001 C CNN
F 3 "~" H 4200 3950 50  0001 C CNN
	1    4200 3950
	0    1    1    0   
$EndComp
Wire Wire Line
	4650 2300 4350 2300
Wire Wire Line
	4550 3950 4350 3950
Wire Wire Line
	4050 3950 4050 3450
Wire Wire Line
	4050 3450 4550 3450
Wire Wire Line
	4050 2300 4050 1800
Wire Wire Line
	4050 1800 4600 1800
Wire Wire Line
	4650 2500 3900 2500
Wire Wire Line
	3900 2500 3900 2700
Wire Wire Line
	3900 4150 4550 4150
Wire Wire Line
	3900 2700 3750 2700
Connection ~ 3900 2700
Wire Wire Line
	3900 2700 3900 4150
Text HLabel 3750 2000 0    50   Input ~ 0
DIN
Text HLabel 3750 2200 0    50   Input ~ 0
DCLK
Text HLabel 3750 2600 0    50   Input ~ 0
OE
Text HLabel 3750 2700 0    50   Input ~ 0
STOR_CLK
Wire Wire Line
	4000 2600 4000 4250
Wire Wire Line
	4000 4250 4550 4250
Connection ~ 4000 2600
Wire Wire Line
	4000 2600 3750 2600
$Comp
L Connector_Generic:Conn_02x07_Odd_Even J17
U 1 1 61DC12D5
P 7800 2300
F 0 "J17" H 7850 2817 50  0000 C CNN
F 1 "Conn_02x07_Odd_Even" H 7850 2726 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x07_P2.54mm_Vertical" H 7800 2300 50  0001 C CNN
F 3 "~" H 7800 2300 50  0001 C CNN
	1    7800 2300
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x07_Odd_Even J18
U 1 1 61DC1A60
P 7800 3850
F 0 "J18" H 7850 4367 50  0000 C CNN
F 1 "Conn_02x07_Odd_Even" H 7850 4276 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x07_P2.54mm_Vertical" H 7800 3850 50  0001 C CNN
F 3 "~" H 7800 3850 50  0001 C CNN
	1    7800 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 2000 6850 2000
Wire Wire Line
	7600 2100 6850 2100
Wire Wire Line
	7600 2200 6850 2200
Wire Wire Line
	7600 2300 6850 2300
Wire Wire Line
	7600 2400 6850 2400
Wire Wire Line
	7600 2500 6850 2500
Wire Wire Line
	7600 2600 6850 2600
Wire Wire Line
	7600 3550 6750 3550
Wire Wire Line
	7600 3650 6750 3650
Wire Wire Line
	7600 3750 6750 3750
Wire Wire Line
	7600 3850 6750 3850
Wire Wire Line
	7600 3950 6750 3950
Wire Wire Line
	7600 4050 6750 4050
Wire Wire Line
	7600 4150 6750 4150
$Comp
L Connector_Generic:Conn_02x01 J15
U 1 1 61DD2734
P 7100 3350
F 0 "J15" H 7150 3567 50  0000 C CNN
F 1 "Conn_02x01" H 7150 3476 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 7100 3350 50  0001 C CNN
F 3 "~" H 7100 3350 50  0001 C CNN
	1    7100 3350
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x01 J16
U 1 1 61DD2DC8
P 7150 1800
F 0 "J16" H 7200 2017 50  0000 C CNN
F 1 "Conn_02x01" H 7200 1926 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 7150 1800 50  0001 C CNN
F 3 "~" H 7150 1800 50  0001 C CNN
	1    7150 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0164
U 1 1 61DD349A
P 7550 1800
F 0 "#PWR0164" H 7550 1550 50  0001 C CNN
F 1 "GND" H 7555 1627 50  0000 C CNN
F 2 "" H 7550 1800 50  0001 C CNN
F 3 "" H 7550 1800 50  0001 C CNN
	1    7550 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0165
U 1 1 61DD389D
P 7550 3350
F 0 "#PWR0165" H 7550 3100 50  0001 C CNN
F 1 "GND" H 7555 3177 50  0000 C CNN
F 2 "" H 7550 3350 50  0001 C CNN
F 3 "" H 7550 3350 50  0001 C CNN
	1    7550 3350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0166
U 1 1 61DD3E1D
P 6450 2900
F 0 "#PWR0166" H 6450 2650 50  0001 C CNN
F 1 "GND" H 6455 2727 50  0000 C CNN
F 2 "" H 6450 2900 50  0001 C CNN
F 3 "" H 6450 2900 50  0001 C CNN
	1    6450 2900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0167
U 1 1 61DD6283
P 4950 4850
F 0 "#PWR0167" H 4950 4600 50  0001 C CNN
F 1 "GND" H 4955 4677 50  0000 C CNN
F 2 "" H 4950 4850 50  0001 C CNN
F 3 "" H 4950 4850 50  0001 C CNN
	1    4950 4850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0168
U 1 1 61DD66CC
P 5050 3150
F 0 "#PWR0168" H 5050 2900 50  0001 C CNN
F 1 "GND" H 5055 2977 50  0000 C CNN
F 2 "" H 5050 3150 50  0001 C CNN
F 3 "" H 5050 3150 50  0001 C CNN
	1    5050 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 3100 5050 3150
Wire Wire Line
	6450 2800 6450 2900
Wire Wire Line
	6350 4350 6350 4450
Wire Wire Line
	7400 3350 7550 3350
Wire Wire Line
	4950 4750 4950 4850
Wire Wire Line
	6750 3350 6900 3350
Wire Wire Line
	7550 1800 7450 1800
Wire Wire Line
	6950 1800 6850 1800
$Comp
L power:+5V #PWR0169
U 1 1 61DE0EEB
P 4600 1800
F 0 "#PWR0169" H 4600 1650 50  0001 C CNN
F 1 "+5V" H 4615 1973 50  0000 C CNN
F 2 "" H 4600 1800 50  0001 C CNN
F 3 "" H 4600 1800 50  0001 C CNN
	1    4600 1800
	1    0    0    -1  
$EndComp
Connection ~ 4600 1800
Wire Wire Line
	4600 1800 5050 1800
$Comp
L power:+5V #PWR0170
U 1 1 61DE1428
P 4550 3450
F 0 "#PWR0170" H 4550 3300 50  0001 C CNN
F 1 "+5V" H 4565 3623 50  0000 C CNN
F 2 "" H 4550 3450 50  0001 C CNN
F 3 "" H 4550 3450 50  0001 C CNN
	1    4550 3450
	1    0    0    -1  
$EndComp
Connection ~ 4550 3450
Wire Wire Line
	4550 3450 4950 3450
Wire Wire Line
	8100 3550 8100 3650
Connection ~ 8100 3650
Wire Wire Line
	8100 3650 8100 3750
Connection ~ 8100 3750
Wire Wire Line
	8100 3750 8100 3850
Connection ~ 8100 3850
Wire Wire Line
	8100 3850 8100 3950
Connection ~ 8100 3950
Wire Wire Line
	8100 3950 8100 4050
Connection ~ 8100 4050
Wire Wire Line
	8100 4050 8100 4150
Connection ~ 8100 4150
Wire Wire Line
	8100 4150 8100 4250
Wire Wire Line
	8100 2000 8100 2100
Connection ~ 8100 2100
Wire Wire Line
	8100 2100 8100 2200
Connection ~ 8100 2200
Wire Wire Line
	8100 2200 8100 2300
Connection ~ 8100 2300
Wire Wire Line
	8100 2300 8100 2400
Connection ~ 8100 2400
Wire Wire Line
	8100 2400 8100 2500
Connection ~ 8100 2500
Wire Wire Line
	8100 2500 8100 2600
Connection ~ 8100 2600
Wire Wire Line
	8100 2600 8100 2700
$Comp
L power:GND #PWR0171
U 1 1 61DE88CA
P 8100 2700
F 0 "#PWR0171" H 8100 2450 50  0001 C CNN
F 1 "GND" H 8105 2527 50  0000 C CNN
F 2 "" H 8100 2700 50  0001 C CNN
F 3 "" H 8100 2700 50  0001 C CNN
	1    8100 2700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0172
U 1 1 61DE8D1B
P 8100 4250
F 0 "#PWR0172" H 8100 4000 50  0001 C CNN
F 1 "GND" H 8105 4077 50  0000 C CNN
F 2 "" H 8100 4250 50  0001 C CNN
F 3 "" H 8100 4250 50  0001 C CNN
	1    8100 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 4150 5450 4150
Wire Wire Line
	5450 4150 5450 3650
Wire Wire Line
	5450 3650 5950 3650
Wire Wire Line
	5350 4050 5550 4050
Wire Wire Line
	5550 4050 5550 3750
Wire Wire Line
	5550 3750 5950 3750
Wire Wire Line
	5350 3950 5650 3950
Wire Wire Line
	5650 3950 5650 3850
Wire Wire Line
	5650 3850 5950 3850
Wire Wire Line
	5350 3850 5600 3850
Wire Wire Line
	5600 3850 5600 3900
Wire Wire Line
	5600 3900 5700 3900
Wire Wire Line
	5700 3900 5700 3950
Wire Wire Line
	5700 3950 5950 3950
Wire Wire Line
	5350 3750 5350 3800
Wire Wire Line
	5350 3800 5750 3800
Wire Wire Line
	5750 3800 5750 4050
Wire Wire Line
	5750 4050 5950 4050
Wire Wire Line
	5350 3650 5400 3650
Wire Wire Line
	5400 3650 5400 3700
Wire Wire Line
	5400 3700 5850 3700
Wire Wire Line
	5850 3700 5850 4150
Wire Wire Line
	5850 4150 5950 4150
Wire Wire Line
	5450 2300 6050 2300
Wire Wire Line
	5550 2200 5550 2400
Wire Wire Line
	5550 2400 6050 2400
Wire Wire Line
	5450 2200 5550 2200
Wire Wire Line
	5450 2100 5650 2100
Wire Wire Line
	5650 2100 5650 2500
Wire Wire Line
	5650 2500 6050 2500
Wire Wire Line
	5750 2000 5750 2600
Wire Wire Line
	5750 2600 6050 2600
Wire Wire Line
	5450 2400 5500 2400
Wire Wire Line
	5500 2400 5500 2350
Wire Wire Line
	5500 2350 5800 2350
Wire Wire Line
	5800 2350 5800 2200
Wire Wire Line
	5800 2200 6050 2200
Wire Wire Line
	5450 2500 5500 2500
Wire Wire Line
	5500 2500 5500 2450
Wire Wire Line
	5500 2450 5850 2450
Wire Wire Line
	5850 2450 5850 2100
Wire Wire Line
	5850 2100 6050 2100
Wire Wire Line
	5450 2600 5450 2550
Wire Wire Line
	5450 2550 5900 2550
Wire Wire Line
	5900 2550 5900 2000
Wire Wire Line
	5900 2000 6050 2000
Wire Wire Line
	5450 2000 5750 2000
$Comp
L Device:D D24
U 1 1 6237A488
P 7150 3050
F 0 "D24" H 7150 3267 50  0000 C CNN
F 1 "D" H 7150 3176 50  0000 C CNN
F 2 "Diode_SMD:D_2010_5025Metric" H 7150 3050 50  0001 C CNN
F 3 "~" H 7150 3050 50  0001 C CNN
	1    7150 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:D D25
U 1 1 6237AB18
P 7200 1500
F 0 "D25" H 7200 1717 50  0000 C CNN
F 1 "D" H 7200 1626 50  0000 C CNN
F 2 "Diode_SMD:D_2010_5025Metric" H 7200 1500 50  0001 C CNN
F 3 "~" H 7200 1500 50  0001 C CNN
	1    7200 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 3350 7400 3050
Wire Wire Line
	7400 3050 7300 3050
Connection ~ 7400 3350
Wire Wire Line
	6900 3350 6900 3050
Wire Wire Line
	6900 3050 7000 3050
Connection ~ 6900 3350
Wire Wire Line
	7450 1800 7450 1500
Wire Wire Line
	7450 1500 7350 1500
Connection ~ 7450 1800
Wire Wire Line
	6950 1800 6950 1500
Wire Wire Line
	6950 1500 7050 1500
Connection ~ 6950 1800
$EndSCHEMATC