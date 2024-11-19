EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 7 7
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
L Device:Q_NPN_BEC Q1
U 1 1 62DF07A9
P 1650 3050
F 0 "Q1" H 1841 3096 50  0000 L CNN
F 1 "Q_NPN_BEC" H 1841 3005 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1850 3150 50  0001 C CNN
F 3 "~" H 1650 3050 50  0001 C CNN
	1    1650 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R23
U 1 1 62DF0A84
P 1750 2550
F 0 "R23" H 1820 2596 50  0000 L CNN
F 1 "R" H 1820 2505 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 1680 2550 50  0001 C CNN
F 3 "~" H 1750 2550 50  0001 C CNN
	1    1750 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R22
U 1 1 62DF60B6
P 1200 3050
F 0 "R22" V 993 3050 50  0000 C CNN
F 1 "R" V 1084 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 1130 3050 50  0001 C CNN
F 3 "~" H 1200 3050 50  0001 C CNN
	1    1200 3050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 62DF6A64
P 1750 3350
F 0 "#PWR04" H 1750 3100 50  0001 C CNN
F 1 "GND" H 1755 3177 50  0000 C CNN
F 2 "" H 1750 3350 50  0001 C CNN
F 3 "" H 1750 3350 50  0001 C CNN
	1    1750 3350
	1    0    0    -1  
$EndComp
Text HLabel 950  3050 1    50   Input ~ 0
IN0
Text HLabel 2250 3050 1    50   Input ~ 0
IN1
Text HLabel 3550 3050 1    50   Input ~ 0
IN2
Text HLabel 4900 3050 1    50   Input ~ 0
IN3
Text HLabel 6200 3050 1    50   Input ~ 0
IN4
Text HLabel 7550 3050 1    50   Input ~ 0
IN5
Text HLabel 1900 2800 2    50   Output ~ 0
~OUT0
Text HLabel 3200 2800 2    50   Output ~ 0
~OUT1
Text HLabel 4500 2800 2    50   Output ~ 0
~OUT2
Text HLabel 5850 2800 2    50   Output ~ 0
~OUT3
Text HLabel 7150 2800 2    50   Output ~ 0
~OUT4
Text HLabel 8500 2800 2    50   Output ~ 0
~OUT5
Text HLabel 1750 1450 0    50   Input ~ 0
VBUF
Wire Wire Line
	1750 3350 1750 3250
Wire Wire Line
	1450 3050 1350 3050
Wire Wire Line
	1050 3050 950  3050
Wire Wire Line
	1750 2850 1750 2800
Wire Wire Line
	1750 2800 1900 2800
Connection ~ 1750 2800
Wire Wire Line
	1750 2800 1750 2700
Wire Wire Line
	1750 2400 1750 1450
$Comp
L Device:Q_NPN_BEC Q2
U 1 1 62DFA9B8
P 2950 3050
F 0 "Q2" H 3141 3096 50  0000 L CNN
F 1 "Q_NPN_BEC" H 3141 3005 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3150 3150 50  0001 C CNN
F 3 "~" H 2950 3050 50  0001 C CNN
	1    2950 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R26
U 1 1 62DFAA0E
P 3050 2550
F 0 "R26" H 3120 2596 50  0000 L CNN
F 1 "R" H 3120 2505 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 2980 2550 50  0001 C CNN
F 3 "~" H 3050 2550 50  0001 C CNN
	1    3050 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R24
U 1 1 62DFAA18
P 2500 3050
F 0 "R24" V 2293 3050 50  0000 C CNN
F 1 "R" V 2384 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 2430 3050 50  0001 C CNN
F 3 "~" H 2500 3050 50  0001 C CNN
	1    2500 3050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR05
U 1 1 62DFAA22
P 3050 3350
F 0 "#PWR05" H 3050 3100 50  0001 C CNN
F 1 "GND" H 3055 3177 50  0000 C CNN
F 2 "" H 3050 3350 50  0001 C CNN
F 3 "" H 3050 3350 50  0001 C CNN
	1    3050 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 3350 3050 3250
Wire Wire Line
	2750 3050 2650 3050
Wire Wire Line
	3050 2850 3050 2800
Wire Wire Line
	3050 2800 3200 2800
Connection ~ 3050 2800
Wire Wire Line
	3050 2800 3050 2700
Wire Wire Line
	3050 2400 3050 1450
$Comp
L Device:Q_NPN_BEC Q3
U 1 1 62DFC5E7
P 4250 3050
F 0 "Q3" H 4441 3096 50  0000 L CNN
F 1 "Q_NPN_BEC" H 4441 3005 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4450 3150 50  0001 C CNN
F 3 "~" H 4250 3050 50  0001 C CNN
	1    4250 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R30
U 1 1 62DFC673
P 4350 2550
F 0 "R30" H 4420 2596 50  0000 L CNN
F 1 "R" H 4420 2505 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 4280 2550 50  0001 C CNN
F 3 "~" H 4350 2550 50  0001 C CNN
	1    4350 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R28
U 1 1 62DFC67D
P 3800 3050
F 0 "R28" V 3593 3050 50  0000 C CNN
F 1 "R" V 3684 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 3730 3050 50  0001 C CNN
F 3 "~" H 3800 3050 50  0001 C CNN
	1    3800 3050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR06
U 1 1 62DFC687
P 4350 3350
F 0 "#PWR06" H 4350 3100 50  0001 C CNN
F 1 "GND" H 4355 3177 50  0000 C CNN
F 2 "" H 4350 3350 50  0001 C CNN
F 3 "" H 4350 3350 50  0001 C CNN
	1    4350 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 3350 4350 3250
Wire Wire Line
	4050 3050 3950 3050
Wire Wire Line
	4350 2850 4350 2800
Wire Wire Line
	4350 2800 4500 2800
Connection ~ 4350 2800
Wire Wire Line
	4350 2800 4350 2700
Wire Wire Line
	4350 2400 4350 1450
$Comp
L Device:Q_NPN_BEC Q4
U 1 1 62E0008F
P 5600 3050
F 0 "Q4" H 5791 3096 50  0000 L CNN
F 1 "Q_NPN_BEC" H 5791 3005 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5800 3150 50  0001 C CNN
F 3 "~" H 5600 3050 50  0001 C CNN
	1    5600 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R34
U 1 1 62E00151
P 5700 2550
F 0 "R34" H 5770 2596 50  0000 L CNN
F 1 "R" H 5770 2505 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 5630 2550 50  0001 C CNN
F 3 "~" H 5700 2550 50  0001 C CNN
	1    5700 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R32
U 1 1 62E0015B
P 5150 3050
F 0 "R32" V 4943 3050 50  0000 C CNN
F 1 "R" V 5034 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 5080 3050 50  0001 C CNN
F 3 "~" H 5150 3050 50  0001 C CNN
	1    5150 3050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR07
U 1 1 62E00165
P 5700 3350
F 0 "#PWR07" H 5700 3100 50  0001 C CNN
F 1 "GND" H 5705 3177 50  0000 C CNN
F 2 "" H 5700 3350 50  0001 C CNN
F 3 "" H 5700 3350 50  0001 C CNN
	1    5700 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 3350 5700 3250
Wire Wire Line
	5400 3050 5300 3050
Wire Wire Line
	5700 2850 5700 2800
Wire Wire Line
	5700 2800 5850 2800
Connection ~ 5700 2800
Wire Wire Line
	5700 2800 5700 2700
Wire Wire Line
	5700 2400 5700 1450
$Comp
L Device:Q_NPN_BEC Q5
U 1 1 62E00176
P 6900 3050
F 0 "Q5" H 7091 3096 50  0000 L CNN
F 1 "Q_NPN_BEC" H 7091 3005 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 7100 3150 50  0001 C CNN
F 3 "~" H 6900 3050 50  0001 C CNN
	1    6900 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R38
U 1 1 62E00180
P 7000 2550
F 0 "R38" H 7070 2596 50  0000 L CNN
F 1 "R" H 7070 2505 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 6930 2550 50  0001 C CNN
F 3 "~" H 7000 2550 50  0001 C CNN
	1    7000 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R36
U 1 1 62E0018A
P 6450 3050
F 0 "R36" V 6243 3050 50  0000 C CNN
F 1 "R" V 6334 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 6380 3050 50  0001 C CNN
F 3 "~" H 6450 3050 50  0001 C CNN
	1    6450 3050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR08
U 1 1 62E00194
P 7000 3350
F 0 "#PWR08" H 7000 3100 50  0001 C CNN
F 1 "GND" H 7005 3177 50  0000 C CNN
F 2 "" H 7000 3350 50  0001 C CNN
F 3 "" H 7000 3350 50  0001 C CNN
	1    7000 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 3350 7000 3250
Wire Wire Line
	6700 3050 6600 3050
Wire Wire Line
	7000 2850 7000 2800
Wire Wire Line
	7000 2800 7150 2800
Connection ~ 7000 2800
Wire Wire Line
	7000 2800 7000 2700
Wire Wire Line
	7000 2400 7000 1450
$Comp
L Device:Q_NPN_BEC Q14
U 1 1 62E039E9
P 8250 3050
F 0 "Q14" H 8441 3096 50  0000 L CNN
F 1 "Q_NPN_BEC" H 8441 3005 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8450 3150 50  0001 C CNN
F 3 "~" H 8250 3050 50  0001 C CNN
	1    8250 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R60
U 1 1 62E03B17
P 8350 2550
F 0 "R60" H 8420 2596 50  0000 L CNN
F 1 "R" H 8420 2505 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 8280 2550 50  0001 C CNN
F 3 "~" H 8350 2550 50  0001 C CNN
	1    8350 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R59
U 1 1 62E03B21
P 7800 3050
F 0 "R59" V 7593 3050 50  0000 C CNN
F 1 "R" V 7684 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 7730 3050 50  0001 C CNN
F 3 "~" H 7800 3050 50  0001 C CNN
	1    7800 3050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 62E03B2B
P 8350 3350
F 0 "#PWR09" H 8350 3100 50  0001 C CNN
F 1 "GND" H 8355 3177 50  0000 C CNN
F 2 "" H 8350 3350 50  0001 C CNN
F 3 "" H 8350 3350 50  0001 C CNN
	1    8350 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 3350 8350 3250
Wire Wire Line
	8050 3050 7950 3050
Wire Wire Line
	8350 2850 8350 2800
Wire Wire Line
	8350 2800 8500 2800
Connection ~ 8350 2800
Wire Wire Line
	8350 2800 8350 2700
Wire Wire Line
	8350 2400 8350 1450
Wire Wire Line
	2350 3050 2250 3050
Wire Wire Line
	3650 3050 3550 3050
Wire Wire Line
	5000 3050 4900 3050
Wire Wire Line
	6300 3050 6200 3050
Wire Wire Line
	7650 3050 7550 3050
Wire Wire Line
	8350 1450 7000 1450
Connection ~ 3050 1450
Wire Wire Line
	3050 1450 1750 1450
Connection ~ 4350 1450
Wire Wire Line
	4350 1450 3050 1450
Connection ~ 5700 1450
Wire Wire Line
	5700 1450 4350 1450
Connection ~ 7000 1450
Wire Wire Line
	7000 1450 5700 1450
Text Notes 5900 3350 0    50   ~ 0
TODO connect LABEL!!!11\n
$EndSCHEMATC