EESchema Schematic File Version 4
LIBS:C245_conn-cache
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
L Connector:Conn_01x06_Male J2
U 1 1 5E5C0C87
P 8200 3300
F 0 "J2" H 8172 3182 50  0000 R CNN
F 1 "C245" H 8172 3273 50  0000 R CNN
F 2 "otter:RPC1-12RB-6P(71)" H 8200 3300 50  0001 C CNN
F 3 "~" H 8200 3300 50  0001 C CNN
	1    8200 3300
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5E5CAC01
P 6600 3050
F 0 "#PWR0101" H 6600 2800 50  0001 C CNN
F 1 "GND" V 6605 2922 50  0000 R CNN
F 2 "" H 6600 3050 50  0001 C CNN
F 3 "" H 6600 3050 50  0001 C CNN
	1    6600 3050
	0    -1   -1   0   
$EndComp
Text GLabel 6600 3150 2    50   Input ~ 0
JUNC
Text GLabel 6600 3350 2    50   Input ~ 0
HEAT
$Comp
L power:GND #PWR0102
U 1 1 5E5CCFBB
P 8000 3500
F 0 "#PWR0102" H 8000 3250 50  0001 C CNN
F 1 "GND" V 8005 3372 50  0000 R CNN
F 2 "" H 8000 3500 50  0001 C CNN
F 3 "" H 8000 3500 50  0001 C CNN
	1    8000 3500
	0    1    1    0   
$EndComp
Text GLabel 8000 3400 0    50   Input ~ 0
JUNC
Text GLabel 8000 3100 0    50   Input ~ 0
HEAT
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 5E992574
P 6400 3150
F 0 "J1" H 6508 3431 50  0000 C CNN
F 1 "Otter-Iron" H 6508 3340 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 6400 3150 50  0001 C CNN
F 3 "~" H 6400 3150 50  0001 C CNN
	1    6400 3150
	1    0    0    -1  
$EndComp
$EndSCHEMATC
