v 20130925 2
C 45800 53800 1 0 1 rs485-1.sym
{
T 44150 55600 5 10 0 0 0 6 1
device=MAX489
T 44450 53950 5 10 1 1 0 6 1
refdes=U7
T 44150 55400 5 10 0 0 0 6 1
footprint=SO8
}
T 41800 60600 9 10 1 0 0 6 1
12V
T 41800 60200 9 10 1 0 0 6 1
GND
T 42200 54900 9 10 1 0 0 6 1
RS485-A
T 42200 54500 9 10 1 0 0 6 1
RS485-B
N 42700 60700 44200 60700 4
N 42700 60300 43800 60300 4
N 43800 58800 43800 60300 4
N 45900 59600 45900 58800 4
N 45900 60500 45900 60700 4
C 53900 58500 1 0 0 gnd-1.sym
{
T 53900 58500 5 10 0 0 0 0 1
netname=GND
}
C 61200 49700 1 90 1 capacitor-1.sym
{
T 60500 49500 5 10 0 1 270 2 1
device=CAPACITOR
T 61300 48700 5 10 1 1 0 6 1
refdes=C9
T 60300 49500 5 10 0 0 270 2 1
symversion=0.1
T 61500 48900 5 10 1 1 0 6 1
value=22pF
T 61200 49700 5 10 0 0 0 6 1
footprint=0805
}
C 60900 48500 1 0 0 gnd-1.sym
C 61700 49700 1 270 0 capacitor-1.sym
{
T 62400 49500 5 10 0 1 270 0 1
device=CAPACITOR
T 62000 48700 5 10 1 1 0 0 1
refdes=C10
T 62600 49500 5 10 0 0 270 0 1
symversion=0.1
T 62000 48900 5 10 1 1 0 0 1
value=22pF
T 61700 49700 5 10 0 0 0 0 1
footprint=0805
}
C 61800 48500 1 0 0 gnd-1.sym
N 43900 54400 44200 54400 4
N 45800 55100 47000 55100 4
{
T 47100 55100 5 10 1 1 0 1 1
netname=TX
}
C 46200 52800 1 0 0 gnd-1.sym
{
T 46200 52800 5 10 0 0 0 0 1
netname=GND
}
N 45800 54800 47000 54800 4
{
T 47100 54800 5 10 1 1 0 1 1
netname=TXEN
}
T 66800 40900 9 10 1 0 0 0 1
MRBus Fast Clock Master
T 66800 40600 9 10 1 0 0 0 1
mrb-fcm.sch
T 67000 40300 9 10 1 0 0 0 1
1
T 68500 40300 9 10 1 0 0 0 1
1
T 70800 40300 9 10 1 0 0 0 1
Nathan D. Holmes
T 70800 40600 9 10 1 0 0 0 1
$Revision: 82 $
C 40000 40000 0 0 0 title-bordered-D.sym
T 70100 43100 9 10 1 0 0 2 3
Notes:
1) All caps ceramic unless otherwise noted.

C 43000 54400 1 0 1 termblk2-1.sym
{
T 42000 55050 5 10 0 0 0 6 1
device=TERMBLK2
T 42600 54200 5 10 1 1 0 6 1
refdes=J2
T 43000 54400 5 10 0 1 0 0 1
footprint=TERMBLK2_200MIL
}
C 42700 60100 1 0 1 termblk2-1.sym
{
T 41700 60750 5 10 0 0 0 6 1
device=TERMBLK2
T 42300 61000 5 10 1 1 0 6 1
refdes=J1
T 42700 60100 5 10 0 1 0 0 1
footprint=TERMBLK2_200MIL
}
C 71100 41800 1 0 0 hole-1.sym
{
T 71100 41800 5 10 0 1 0 0 1
device=HOLE
T 71300 42400 5 10 1 1 0 4 1
refdes=H1
T 71100 41800 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 71600 41800 1 0 0 hole-1.sym
{
T 71600 41800 5 10 0 1 0 0 1
device=HOLE
T 71800 42400 5 10 1 1 0 4 1
refdes=H2
T 71600 41800 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 72100 41800 1 0 0 hole-1.sym
{
T 72100 41800 5 10 0 1 0 0 1
device=HOLE
T 72300 42400 5 10 1 1 0 4 1
refdes=H3
T 72100 41800 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 72600 41800 1 0 0 hole-1.sym
{
T 72600 41800 5 10 0 1 0 0 1
device=HOLE
T 72800 42400 5 10 1 1 0 4 1
refdes=H4
T 72600 41800 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
N 45100 60700 47100 60700 4
N 47900 59300 47900 58800 4
C 53800 60500 1 270 0 capacitor-1.sym
{
T 54500 60300 5 10 0 1 270 0 1
device=CAPACITOR
T 54100 60200 5 10 1 1 0 0 1
refdes=C2A
T 54700 60300 5 10 0 0 270 0 1
symversion=0.1
T 54100 59700 5 10 1 1 0 0 1
value=22uF
T 53800 60500 5 10 0 0 0 0 1
footprint=0805
T 54100 59500 5 10 1 1 0 0 1
comment=6.3V
}
N 54000 60500 54000 60700 4
N 54000 59600 54000 58800 4
N 46300 54200 46300 54800 4
N 45800 53100 45800 54200 4
N 42700 58800 54000 58800 4
N 58600 55000 58000 55000 4
{
T 57900 55000 5 10 1 1 0 7 1
netname=RX
}
N 58600 54600 58000 54600 4
{
T 57900 54600 5 10 1 1 0 7 1
netname=TX
}
N 58600 52600 58000 52600 4
{
T 57900 52600 5 10 1 1 0 7 1
netname=TXEN
}
C 54000 55700 1 0 0 avrprog-1.sym
{
T 54000 57300 5 10 0 1 0 0 1
device=AVRPROG
T 54300 57100 5 10 1 1 0 0 1
refdes=J3
T 54000 55700 5 10 0 0 0 0 1
footprint=JUMPER3x2-SMT
}
N 55400 56700 55900 56700 4
C 55300 55400 1 0 0 gnd-1.sym
N 55400 55900 55400 55700 4
C 45700 60500 1 270 0 Cap_H-2.sym
{
T 45400 60200 5 10 1 1 0 0 1
refdes=C1
T 47200 60500 5 10 0 0 270 0 1
device=Capacitor
T 45200 60100 5 10 1 1 0 2 1
value=68uF
T 45700 60500 5 10 0 1 0 0 1
footprint=cap-elec-Panasonic-FK--D6.30-H5.80-mm
}
C 72600 58500 1 90 1 battery-2.sym
{
T 73000 58100 5 10 1 1 90 6 1
device=CR1220 SOCKET
T 73000 57800 5 10 0 1 180 2 1
footprint=S8411
T 71300 58200 5 10 0 0 90 6 1
symversion=0.1
T 72800 58500 5 10 1 1 0 2 1
refdes=XB1
}
C 72300 57200 1 0 0 gnd-1.sym
C 67400 49900 1 180 0 LCD-1.sym
{
T 66400 44325 5 10 0 0 180 0 1
device=LCD
T 66600 50100 5 10 1 1 0 0 1
refdes=LCD1
T 67400 49900 5 10 0 0 0 0 1
footprint=HEADER16
}
C 69400 43900 1 0 0 gnd-1.sym
N 67600 45100 67600 44200 4
N 67400 49000 68100 49000 4
{
T 67500 49100 5 10 1 1 0 0 1
netname=DATA3
}
N 67400 48700 68100 48700 4
{
T 67500 48800 5 10 1 1 0 0 1
netname=DATA2
}
N 67400 48400 68100 48400 4
{
T 67500 48500 5 10 1 1 0 0 1
netname=DATA1
}
N 67400 48100 68100 48100 4
{
T 67500 48200 5 10 1 1 0 0 1
netname=DATA0
}
N 67400 46600 68100 46600 4
{
T 67800 46700 5 10 1 1 0 0 1
netname=EN
}
N 67400 46300 68100 46300 4
{
T 67700 46400 5 10 1 1 0 0 1
netname=R/W
}
N 67400 46000 68100 46000 4
{
T 67800 46100 5 10 1 1 0 0 1
netname=RS
}
N 67400 45100 67600 45100 4
N 67400 45700 68100 45700 4
N 67400 45400 69500 45400 4
N 67400 49600 67600 49600 4
N 67400 49300 68600 49300 4
N 52700 52600 54700 52600 4
{
T 54800 52600 5 10 1 1 0 1 1
netname=BTN0
}
N 52700 52300 54700 52300 4
{
T 54800 52300 5 10 1 1 0 1 1
netname=BTN1
}
N 52700 52000 54700 52000 4
{
T 54800 52000 5 10 1 1 0 1 1
netname=BTN2
}
N 52700 51700 54700 51700 4
{
T 54800 51700 5 10 1 1 0 1 1
netname=BTN3
}
N 53300 49900 53300 52600 4
N 53700 49900 53700 52300 4
N 54100 49900 54100 52000 4
N 54500 49900 54500 51700 4
N 43000 55000 43900 55000 4
N 43900 55000 43900 55200 4
N 43900 55200 44200 55200 4
N 43000 54600 43900 54600 4
N 43900 54600 43900 54400 4
N 67600 49600 67600 49800 4
C 41700 58700 1 0 0 pwrjack3-1.sym
{
T 41800 59200 5 10 0 0 0 0 1
device=PWRJACK
T 41700 59400 5 10 1 1 0 0 1
refdes=J1A
T 41700 58700 5 10 0 0 0 0 1
footprint=CUI_PJ-202AH
}
N 42700 59200 43200 59200 4
N 43200 59200 43200 60700 4
N 42700 59000 43000 59000 4
N 43000 59000 43000 58800 4
N 72400 57500 72400 57600 4
C 68700 44200 1 90 0 pot-bourns.sym
{
T 67800 45000 5 10 0 0 90 0 1
device=VARIABLE_RESISTOR
T 68900 44500 5 10 1 1 90 0 1
refdes=R8
T 68400 44800 5 10 1 1 90 0 1
value=10k
T 68700 44200 5 10 0 0 90 0 1
footprint=TC33_trimmer
}
N 68100 45700 68100 44700 4
C 43400 53500 1 90 0 resistor-1.sym
{
T 43000 53800 5 10 0 0 90 0 1
device=RESISTOR
T 43100 53500 5 10 1 1 90 0 1
refdes=R7
T 43100 54000 5 10 1 1 90 0 1
value=2K
T 43400 53500 5 10 0 0 90 0 1
footprint=0805
}
C 43400 55200 1 90 0 resistor-1.sym
{
T 43000 55500 5 10 0 0 90 0 1
device=RESISTOR
T 43100 55300 5 10 1 1 90 0 1
refdes=R6
T 43100 55800 5 10 1 1 90 0 1
value=2K
T 43400 55200 5 10 0 0 90 0 1
footprint=0805
}
N 43300 54400 43300 54600 4
N 43300 55200 43300 55000 4
N 68600 45100 68600 45400 4
C 69300 45100 1 270 0 capacitor-1.sym
{
T 70000 44900 5 10 0 1 270 0 1
device=CAPACITOR
T 69600 44800 5 10 1 1 0 0 1
refdes=C8
T 70200 44900 5 10 0 0 270 0 1
symversion=0.1
T 69600 44300 5 10 1 1 0 0 1
value=0.1uF
T 69300 45100 5 10 0 0 0 0 1
footprint=0805
}
N 69500 45400 69500 45100 4
N 67600 44200 69500 44200 4
N 44800 47000 44800 50600 4
C 49500 42300 1 270 1 led-3.sym
{
T 49500 42300 5 10 0 0 180 2 1
footprint=0805
T 50150 42050 5 10 1 1 270 6 1
device=RED (ASSOC) LED
T 49250 42650 5 10 1 1 270 6 1
refdes=D5
}
N 49700 43200 49700 45500 4
N 49700 42100 49700 42300 4
C 49800 40800 1 0 1 gnd-1.sym
N 49200 44900 50300 44900 4
{
T 50400 44850 5 10 1 1 0 0 1
netname=\_CTS\_
}
C 45300 43200 1 0 0 xb3_mmt.sym
{
T 47700 47600 5 10 0 0 0 0 1
device=XBEE
T 47800 47400 5 10 0 0 0 0 1
footprint=XB3_MMT
T 47250 47700 5 10 1 1 0 3 1
refdes=U5
}
C 49500 43500 1 0 1 gnd-1.sym
N 49400 43800 49400 47900 4
N 49400 44000 49200 44000 4
N 49400 47300 49200 47300 4
N 49400 47900 49200 47900 4
C 45000 43500 1 0 0 gnd-1.sym
N 45100 44000 45300 44000 4
N 45100 43800 45100 47300 4
N 45100 47300 45300 47300 4
N 45100 44600 45300 44600 4
N 49700 45500 49200 45500 4
N 44800 47000 45300 47000 4
N 43800 46700 45300 46700 4
{
T 43700 46700 5 10 1 1 0 7 1
netname=RX
}
N 44800 45500 45300 45500 4
N 44800 43200 44800 45500 4
N 41100 44900 45300 44900 4
C 46600 50000 1 270 0 capacitor-1.sym
{
T 47300 49800 5 10 0 1 270 0 1
device=CAPACITOR
T 47500 49800 5 10 0 0 270 0 1
symversion=0.1
T 46600 50000 5 10 0 0 0 0 1
footprint=0805
T 46900 49700 5 10 1 1 0 0 1
refdes=C4
T 46900 49200 5 10 1 1 0 0 1
value=1uF
T 46900 49000 5 10 1 1 0 0 1
description=16V
}
C 45400 50000 1 270 0 capacitor-1.sym
{
T 46100 49800 5 10 0 1 270 0 1
device=CAPACITOR
T 46300 49800 5 10 0 0 270 0 1
symversion=0.1
T 45400 50000 5 10 0 0 0 0 1
footprint=0805
T 45700 49700 5 10 1 1 0 0 1
refdes=C11
T 45700 49200 5 10 1 1 0 0 1
value=8.2pF
T 45700 49000 5 10 1 1 0 0 1
description=16V, NP0
}
N 46800 50000 46800 50200 4
N 45600 50000 45600 50200 4
C 45500 48800 1 0 0 gnd-1.sym
C 46700 48800 1 0 0 gnd-1.sym
N 44800 50200 46800 50200 4
C 42900 48200 1 0 1 xbprog-1.sym
{
T 42900 49800 5 10 0 1 0 6 1
device=AVRPROG
T 42900 48200 5 10 0 0 0 6 1
footprint=TC2030
T 42300 49500 5 10 1 1 0 6 1
refdes=J4
}
C 41400 47900 1 0 0 gnd-1.sym
N 41500 48200 41500 48400 4
N 42900 48400 43200 48400 4
{
T 43300 48400 5 10 1 1 0 1 1
netname=\_RESET\_
}
C 44900 40800 1 0 1 gnd-1.sym
N 44800 41100 44800 41200 4
T 47500 50300 9 10 1 0 0 6 1
Place C4 & C11 near XBee pin 1
N 49200 45800 50300 45800 4
{
T 50400 45750 5 10 1 1 0 0 1
netname=\_RTS\_
}
N 41100 44900 41100 48800 4
N 41100 48800 41500 48800 4
N 42900 49200 44500 49200 4
N 44500 49200 44500 46700 4
N 41500 49200 41500 49800 4
N 41500 49800 44200 49800 4
N 44200 49800 44200 46400 4
N 43800 46400 45300 46400 4
{
T 43700 46400 5 10 1 1 0 7 1
netname=TX
}
N 42900 48800 43200 48800 4
{
T 43300 48750 5 10 1 1 0 0 1
netname=\_RTS\_
}
C 44600 50600 1 0 0 3V3-plus-1.sym
C 62300 61000 1 270 0 capacitor-1.sym
{
T 63000 60800 5 10 0 1 270 0 1
device=CAPACITOR
T 62600 60700 5 10 1 1 0 0 1
refdes=C6
T 63200 60800 5 10 0 0 270 0 1
symversion=0.1
T 62600 60200 5 10 1 1 0 0 1
value=0.1uF
T 62300 61000 5 10 0 0 0 0 1
footprint=0805
}
C 61500 61000 1 270 0 capacitor-1.sym
{
T 62200 60800 5 10 0 1 270 0 1
device=CAPACITOR
T 61800 60700 5 10 1 1 0 0 1
refdes=C7
T 62400 60800 5 10 0 0 270 0 1
symversion=0.1
T 61800 60200 5 10 1 1 0 0 1
value=0.1uF
T 61500 61000 5 10 0 0 0 0 1
footprint=0805
}
C 62400 59600 1 0 0 gnd-1.sym
N 62500 59900 62500 60100 4
N 61700 60100 61700 59900 4
N 61700 59900 62500 59900 4
C 55700 56700 1 0 0 3V3-plus-1.sym
C 44200 60400 1 0 0 schottky-diode-1.sym
{
T 44522 61072 5 10 0 0 0 0 1
device=DIODE
T 44600 61000 5 10 1 1 0 3 1
refdes=D1
T 45041 60832 5 10 0 1 0 0 1
footprint=SOD123T
T 44700 60400 5 10 1 1 0 5 1
device=CDBM140
}
T 54200 57500 9 10 1 0 0 0 1
ICSP Header
T 41800 47300 9 10 1 0 0 0 3
XBee Programming
Header

C 53300 48900 1 90 0 switch-pushbutton-no-1.sym
{
T 53200 50000 5 10 1 1 90 0 1
refdes=SW1
T 53200 50500 5 10 1 1 90 0 1
device=3-1825910-1
T 53300 48900 5 10 0 0 90 0 1
footprint=1825910
}
C 53700 48900 1 90 0 switch-pushbutton-no-1.sym
{
T 53600 50000 5 10 1 1 90 0 1
refdes=SW2
T 53600 50500 5 10 1 1 90 0 1
device=3-1825910-1
T 53700 48900 5 10 0 0 90 0 1
footprint=1825910
}
C 54100 48900 1 90 0 switch-pushbutton-no-1.sym
{
T 54000 50000 5 10 1 1 90 0 1
refdes=SW3
T 54000 50500 5 10 1 1 90 0 1
device=3-1825910-1
T 54100 48900 5 10 0 0 90 0 1
footprint=1825910
}
C 54500 48900 1 90 0 switch-pushbutton-no-1.sym
{
T 54400 50000 5 10 1 1 90 0 1
refdes=SW4
T 54400 50500 5 10 1 1 90 0 1
device=3-1825910-1
T 54500 48900 5 10 0 0 90 0 1
footprint=1825910
}
N 53300 48900 54500 48900 4
C 53800 48600 1 0 0 gnd-1.sym
C 56000 49200 1 0 0 cpdt6-5v4-2.sym
{
T 57600 50850 5 10 1 1 0 0 1
refdes=U4
T 56200 51600 5 10 0 0 0 0 1
footprint=SOT26
T 57300 50850 5 10 1 1 0 6 1
device=CPDT6-5V4-HF
}
N 58000 50500 58200 50500 4
{
T 58300 50500 5 10 1 1 0 1 1
netname=BTN0
}
N 58000 49500 58200 49500 4
{
T 58300 49500 5 10 1 1 0 1 1
netname=BTN1
}
C 58200 49700 1 0 0 gnd-1.sym
N 58000 50000 58300 50000 4
C 55600 49700 1 0 0 gnd-1.sym
N 55700 50000 56000 50000 4
C 51400 52500 1 0 0 res-pack4-1.sym
{
T 51400 52500 5 10 0 0 0 0 1
slot=1
T 51400 52500 5 10 0 0 0 0 1
footprint=RPACK4-1206
T 52200 52800 5 10 1 1 0 0 1
value=10k
T 52200 53000 5 10 1 1 0 0 1
refdes=R2
}
C 51400 52200 1 0 0 res-pack4-1.sym
{
T 51400 52200 5 10 0 0 0 0 1
slot=2
T 51400 52200 5 10 0 0 0 0 1
footprint=RPACK4-1206
T 51400 52200 5 10 0 0 0 0 1
value=10k
T 51400 52200 5 10 0 1 0 0 1
refdes=R2
}
C 51400 51900 1 0 0 res-pack4-1.sym
{
T 51400 51900 5 10 0 0 0 0 1
slot=3
T 51400 51900 5 10 0 0 0 0 1
footprint=RPACK4-1206
T 51400 51900 5 10 0 0 0 0 1
value=10k
T 51400 51900 5 10 0 1 0 0 1
refdes=R2
}
C 51400 51600 1 0 0 res-pack4-1.sym
{
T 51400 51600 5 10 0 0 0 0 1
slot=4
T 51400 51600 5 10 0 0 0 0 1
footprint=RPACK4-1206
T 51400 51600 5 10 0 0 0 0 1
value=10k
T 51400 51600 5 10 0 1 0 0 1
refdes=R2
}
N 51800 51700 51800 53100 4
C 51600 53100 1 0 0 3V3-plus-1.sym
C 54900 60700 1 0 0 3V3-plus-1.sym
N 52000 60700 55100 60700 4
C 48400 56800 1 0 1 78l05-1.sym
{
T 46800 58100 5 10 0 0 0 6 1
device=7805
T 47000 57800 5 10 1 1 0 0 1
refdes=U8
T 48400 56800 5 10 0 0 0 6 1
footprint=SOT89
}
C 46700 57300 1 90 1 capacitor-1.sym
{
T 46000 57100 5 10 0 1 270 2 1
device=CAPACITOR
T 46400 57000 5 10 1 1 0 6 1
refdes=C3
T 45800 57100 5 10 0 0 270 2 1
symversion=0.1
T 46400 56500 5 10 1 1 0 6 1
value=1uF
T 46700 57300 5 10 0 0 0 6 1
footprint=0805
}
N 45000 57400 46800 57400 4
N 46500 57300 46500 57400 4
C 46600 56100 1 0 1 gnd-1.sym
{
T 46600 56100 5 10 0 0 0 6 1
netname=GND
}
N 47600 56800 47600 56400 4
N 47600 56400 46500 56400 4
C 45200 60700 1 0 0 12V-plus-1.sym
C 48900 57600 1 0 1 12V-plus-1.sym
N 48700 57600 48700 57400 4
N 48700 57400 48400 57400 4
C 55300 59700 1 90 0 led-3.sym
{
T 55300 59700 5 10 0 0 0 0 1
footprint=0805
T 55550 59950 5 10 1 1 90 0 1
device=GREEN LED
T 54750 60350 5 10 1 1 0 0 1
refdes=D2
}
N 55100 60700 55100 60600 4
C 55000 58300 1 0 0 gnd-1.sym
C 44900 40800 1 90 0 res-pack4-1.sym
{
T 44900 40800 5 10 0 0 90 0 1
slot=3
T 44900 40800 5 10 0 0 90 0 1
footprint=RPACK4-1206
T 45100 41500 5 10 1 1 0 0 1
value=330
T 45100 41700 5 10 1 1 0 0 1
refdes=R1
}
C 44600 42300 1 270 1 led-3.sym
{
T 44600 42300 5 10 0 0 180 2 1
footprint=0805
T 44450 42950 5 10 1 1 270 5 1
device=YELLOW (RSSI) LED
T 45150 42750 5 10 1 1 270 6 1
refdes=D4
}
N 44800 42100 44800 42300 4
C 49600 40800 1 270 1 res-pack4-1.sym
{
T 49600 40800 5 10 0 0 90 2 1
slot=4
T 49600 40800 5 10 0 0 90 2 1
footprint=RPACK4-1206
T 49400 41500 5 10 1 1 0 6 1
value=330
T 49400 41700 5 10 1 1 0 6 1
refdes=R1
}
N 49700 41100 49700 41200 4
C 55200 58200 1 90 0 res-pack4-1.sym
{
T 55200 58200 5 10 0 0 90 0 1
slot=2
T 55200 58200 5 10 0 0 90 0 1
footprint=RPACK4-1206
T 55400 59000 5 10 1 1 0 0 1
value=330
T 55400 59200 5 10 1 1 0 0 1
refdes=R1
}
N 55100 59500 55100 59700 4
C 43300 56000 1 0 0 SolderJumperOpen-3.sym
{
T 43650 56250 5 10 1 1 0 3 1
refdes=JP1A
T 43300 57600 5 10 0 0 0 0 1
footprint=SolderJumperSmall
T 43300 57800 5 10 0 0 0 0 1
device=SolderJumper
}
C 44000 53200 1 180 0 SolderJumperOpen-3.sym
{
T 43650 52950 5 10 1 1 180 3 1
refdes=JP1B
T 44000 51600 5 10 0 0 180 0 1
footprint=SolderJumperSmall
T 44000 51400 5 10 0 0 180 0 1
device=SolderJumper
}
N 46300 53100 46300 53300 4
N 44000 53100 46300 53100 4
N 45000 53800 45000 53100 4
N 43300 53100 43300 53500 4
N 44000 56100 45000 56100 4
N 45000 55700 45000 57400 4
C 48900 54900 1 90 0 led-3.sym
{
T 48900 54900 5 10 0 0 0 0 1
footprint=0805
T 49050 55150 5 10 1 1 0 0 1
device=AMBER LED
T 49050 55450 5 10 1 1 0 0 1
refdes=D3
}
N 45800 54500 49000 54500 4
{
T 49200 54500 5 10 1 1 0 1 1
netname=RX_5V
}
N 48700 54900 48700 54500 4
B 40900 51500 10100 6700 3 0 0 0 -1 -1 0 -1 -1 -1 -1 -1
B 40900 40600 10100 10600 3 0 0 0 -1 -1 0 -1 -1 -1 -1 -1
T 41000 57800 9 10 1 0 0 0 1
Wired MRBus Interface Only
T 41000 50900 9 10 1 0 0 0 1
Wireless MRBus Interface Only
C 69200 48900 1 0 0 DMN5L06DMK.sym
{
T 70800 51000 5 10 1 1 0 0 1
refdes=U6
T 70595 51000 5 10 1 1 0 6 1
device=DMN5L06DMK
}
N 67600 49800 69200 49800 4
N 71200 49400 71900 49400 4
{
T 72000 49400 5 10 1 1 0 1 1
netname=BKLT_EN
}
C 72600 51800 1 0 0 3V3-plus-1.sym
N 72800 51800 72800 50200 4
N 72800 50200 71200 50200 4
N 71200 50600 72200 50600 4
{
T 72300 50600 5 10 1 1 0 1 1
netname=RX
}
N 68700 50600 69200 50600 4
{
T 68500 50400 5 10 1 1 0 7 1
netname=RX_5V
}
N 45000 55800 47300 55800 4
N 48700 55800 48200 55800 4
N 71800 51700 71800 51800 4
N 71800 51800 72800 51800 4
N 71800 50800 71800 50600 4
C 46200 52900 1 270 1 res-pack4-1.sym
{
T 46200 52900 5 10 0 0 90 2 1
slot=1
T 46200 52900 5 10 0 0 90 2 1
footprint=RPACK4-1206
T 46600 53600 5 10 1 1 0 0 1
value=10k
T 46600 53800 5 10 1 1 0 0 1
refdes=R3
}
C 46900 55700 1 0 0 res-pack4-1.sym
{
T 46900 55700 5 10 0 0 180 8 1
slot=1
T 46900 55700 5 10 0 0 180 8 1
footprint=RPACK4-1206
T 47800 56000 5 10 1 1 0 0 1
value=330
T 47800 56200 5 10 1 1 0 0 1
refdes=R1
}
C 72800 53100 1 0 1 qwiic-1.sym
{
T 71800 53750 5 10 0 0 0 6 1
device=QWIIC
T 72500 54950 5 10 1 1 0 3 1
refdes=J5
T 72000 55300 5 10 0 1 0 6 1
footprint=SM04B-SRSS
}
N 69700 53900 71900 53900 4
{
T 69600 53900 5 10 1 1 0 7 1
netname=SDA
}
N 69700 53500 71900 53500 4
{
T 69600 53500 5 10 1 1 0 7 1
netname=SCL
}
N 70700 53500 70700 54400 4
C 72200 55300 1 0 0 gnd-1.sym
N 72300 55600 71900 55600 4
N 71900 55600 71900 54700 4
N 71300 55700 71300 54300 4
N 71300 54300 71900 54300 4
C 70100 54000 1 90 0 res-pack2-1.sym
{
T 69805 54900 5 10 1 1 0 6 1
refdes=R5
T 70100 54000 5 10 0 0 0 0 1
footprint=RPACK2-0606
T 69800 54600 5 10 1 1 0 6 1
value=2k
T 70100 54000 5 10 0 0 0 0 1
slot=1
}
C 70800 54000 1 90 0 res-pack2-1.sym
{
T 70505 54900 5 10 1 1 0 6 1
refdes=R5
T 70800 54000 5 10 0 0 0 0 1
footprint=RPACK2-0606
T 70500 54600 5 10 1 1 0 6 1
value=2k
T 70800 54000 5 10 0 0 0 0 1
slot=2
}
N 70000 55300 70000 55600 4
N 70000 55600 71300 55600 4
N 70700 55300 70700 55600 4
N 70000 54400 70000 53900 4
C 71700 52100 1 270 0 res-pack4-1.sym
{
T 71700 52100 5 10 0 0 270 0 1
slot=3
T 71700 52100 5 10 0 0 270 0 1
footprint=RPACK4-1206
T 71600 51200 5 10 1 1 180 0 1
value=10k
T 71600 51400 5 10 1 1 180 0 1
refdes=R3
}
C 73000 48700 1 0 0 gnd-1.sym
N 71200 49800 73100 49800 4
N 73100 49800 73100 49000 4
C 42800 43500 1 0 1 gnd-1.sym
C 68400 49300 1 0 0 3V3-plus-1.sym
C 69300 45400 1 0 0 3V3-plus-1.sym
N 56000 50500 55800 50500 4
{
T 55700 50500 5 10 1 1 0 7 1
netname=BTN3
}
N 56000 49500 55800 49500 4
{
T 55700 49500 5 10 1 1 0 7 1
netname=BTN2
}
C 58500 51400 1 0 0 ATmega164-tqfp44.sym
{
T 62700 58900 5 10 1 1 0 6 1
refdes=U2
T 60800 55450 5 10 1 1 0 4 1
device=ATmega1284
T 60800 55150 5 10 1 1 0 4 1
footprint=TQFP44
}
N 63000 55000 63600 55000 4
{
T 63700 55000 5 10 1 1 0 1 1
netname=SCL
}
N 63000 54600 63600 54600 4
{
T 63700 54600 5 10 1 1 0 1 1
netname=SDA
}
N 54000 56700 53400 56700 4
{
T 53300 56700 5 10 1 1 0 7 1
netname=MISO
}
N 58600 56000 58000 56000 4
{
T 57900 56000 5 10 1 1 0 7 1
netname=MISO
}
N 54000 56300 53400 56300 4
{
T 53300 56300 5 10 1 1 0 7 1
netname=SCK
}
N 58600 55600 58000 55600 4
{
T 57900 55600 5 10 1 1 0 7 1
netname=SCK
}
N 55400 56300 56000 56300 4
{
T 56100 56300 5 10 1 1 0 1 1
netname=MOSI
}
N 58600 56400 58000 56400 4
{
T 57900 56400 5 10 1 1 0 7 1
netname=MOSI
}
N 59800 59100 61200 59100 4
N 60900 59100 60900 61000 4
N 60900 61000 62500 61000 4
C 60700 61000 1 0 0 3V3-plus-1.sym
N 63000 54200 63600 54200 4
{
T 63700 54200 5 10 1 1 0 1 1
netname=BTN0
}
N 63000 53800 63600 53800 4
{
T 63700 53800 5 10 1 1 0 1 1
netname=BTN1
}
N 63000 53400 63600 53400 4
{
T 63700 53400 5 10 1 1 0 1 1
netname=BTN2
}
N 63000 53000 63600 53000 4
{
T 63700 53000 5 10 1 1 0 1 1
netname=BTN3
}
N 59300 51500 60500 51500 4
C 59800 50900 1 0 0 gnd-1.sym
N 59900 51200 59900 51500 4
C 71100 55700 1 0 0 3V3-plus-1.sym
C 51800 55700 1 270 0 capacitor-1.sym
{
T 52500 55500 5 10 0 1 270 0 1
device=CAPACITOR
T 52700 55500 5 10 0 0 270 0 1
symversion=0.1
T 51800 55700 5 10 0 0 0 0 1
footprint=0805
T 52100 55400 5 10 1 1 0 0 1
refdes=C5
T 52100 54900 5 10 1 1 0 0 1
value=1uF
}
C 51900 54300 1 0 0 gnd-1.sym
N 52000 54800 52000 54600 4
N 52000 55700 52000 56000 4
C 51800 57100 1 0 0 3V3-plus-1.sym
N 52000 57100 52000 56900 4
C 52100 57300 1 90 1 res-pack4-1.sym
{
T 52100 57300 5 10 0 0 270 2 1
slot=4
T 52100 57300 5 10 0 0 270 2 1
footprint=RPACK4-1206
T 51800 56400 5 10 1 1 180 0 1
value=10k
T 51800 56600 5 10 1 1 180 0 1
refdes=R3
}
N 53600 55300 53000 55300 4
{
T 53700 55300 5 10 1 1 0 1 1
netname=\_RESET\_
}
N 52000 55900 54000 55900 4
N 53000 55900 53000 55300 4
N 62400 51300 63400 51300 4
{
T 63500 51300 5 10 1 1 0 1 1
netname=\_RESET\_
}
N 62400 51300 62400 51500 4
C 69700 57100 1 0 0 rv3129-1.sym
{
T 71400 59600 5 10 1 1 0 6 1
device=RV-3129-C3
T 70000 59750 5 10 0 0 0 0 1
footprint=RV3129
T 71400 59800 5 10 1 1 0 6 1
refdes=U3
}
N 71700 58700 72400 58700 4
N 72400 58700 72400 58500 4
N 71700 57500 72400 57500 4
C 68700 58700 1 0 0 3V3-plus-1.sym
N 68900 58700 69700 58700 4
N 69700 57900 69100 57900 4
{
T 69000 57900 5 10 1 1 0 7 1
netname=SCL
}
N 69700 57500 69100 57500 4
{
T 69000 57500 5 10 1 1 0 7 1
netname=SDA
}
C 69200 59200 1 0 0 gnd-1.sym
N 69300 59500 69700 59500 4
N 69700 59500 69700 59100 4
N 47100 60700 47100 60300 4
C 49400 61200 1 270 0 capacitor-1.sym
{
T 50100 61000 5 10 0 1 270 0 1
device=CAPACITOR
T 49700 61100 5 10 1 1 0 0 1
refdes=C13
T 50300 61000 5 10 0 0 270 0 1
symversion=0.1
T 49700 60900 5 10 1 1 0 0 1
value=0.1uF
T 49400 61200 5 10 0 0 0 0 1
footprint=0805
}
N 49600 61200 48700 61200 4
N 48700 61200 48700 60700 4
N 48700 60300 50600 60300 4
N 48700 59900 52400 59900 4
N 52400 59900 52400 60700 4
C 51100 60600 1 0 0 inductor-1.sym
{
T 51300 61100 5 10 0 0 0 0 1
device=INDUCTOR
T 51300 61300 5 10 0 0 0 0 1
symversion=0.1
T 51100 60600 5 10 0 0 0 0 1
footprint=MSS6132
T 51550 60850 5 10 1 1 0 3 1
refdes=L1
T 51550 60400 5 10 1 1 0 5 1
model=MSS6132-472
T 51550 60600 5 10 1 1 0 5 1
value=4.7uH
}
N 50600 60300 50600 60700 4
N 50600 60700 51100 60700 4
C 53100 60500 1 270 0 capacitor-1.sym
{
T 53800 60300 5 10 0 1 270 0 1
device=CAPACITOR
T 53400 60200 5 10 1 1 0 0 1
refdes=C2
T 54000 60300 5 10 0 0 270 0 1
symversion=0.1
T 53400 59700 5 10 1 1 0 0 1
value=22uF
T 53100 60500 5 10 0 0 0 0 1
footprint=0805
T 53400 59500 5 10 1 1 0 0 1
comment=6.3V
}
N 53300 60500 53300 60700 4
N 53300 59600 53300 58800 4
C 46300 60500 1 270 0 capacitor-1.sym
{
T 47000 60300 5 10 0 1 270 0 1
device=CAPACITOR
T 46600 60200 5 10 1 1 0 0 1
refdes=C12
T 47200 60300 5 10 0 0 270 0 1
symversion=0.1
T 46600 59700 5 10 1 1 0 0 1
value=10uF
T 46300 60500 5 10 0 0 0 0 1
footprint=0805
}
N 46500 60500 46500 60700 4
N 46500 59600 46500 58800 4
N 58600 53800 58000 53800 4
{
T 57900 53800 5 10 1 1 0 7 1
netname=AUX_TX
}
N 58600 54200 58000 54200 4
{
T 57900 54200 5 10 1 1 0 7 1
netname=AUX_RX
}
C 53100 42900 1 0 1 termblk3-1.sym
{
T 52100 43550 5 10 0 0 0 6 1
device=HEADER3
T 52700 44200 5 10 1 1 0 6 1
refdes=J6
T 53100 42900 5 10 0 0 0 0 1
footprint=TERMBLK3_3p5MM
}
C 53500 43200 1 0 1 gnd-1.sym
N 53400 43500 53100 43500 4
N 53100 43900 53900 43900 4
{
T 54000 43900 5 10 1 1 0 1 1
netname=IN_1
}
N 53100 43100 55000 43100 4
{
T 55200 43100 5 10 1 1 0 1 1
netname=IN_2
}
N 53600 44300 53600 43900 4
N 54800 44300 54800 43100 4
C 54400 45200 1 0 1 3.3V-plus-1.sym
N 53600 45200 54800 45200 4
C 57000 40200 1 0 0 dm3at.sym
{
T 57300 44250 5 10 0 1 0 0 1
device=SD Card Socket
T 58300 41800 5 10 0 1 0 0 1
footprint=hirose-dm3at
T 58300 41800 5 10 1 1 0 0 1
refdes=J8
}
C 72900 44100 1 0 1 auxser-1.sym
{
T 71900 44750 5 10 0 0 0 6 1
device=QWIIC
T 72600 46850 5 10 1 1 0 3 1
refdes=J7
T 72100 46300 5 10 0 1 0 6 1
footprint=BM06B-SRSS
}
C 71600 46200 1 0 0 gnd-1.sym
N 71700 46500 72000 46500 4
C 70100 44500 1 0 0 3V3-plus-1.sym
N 70300 44500 72000 44500 4
N 72000 45300 71400 45300 4
{
T 71300 45300 5 10 1 1 0 7 1
netname=AUX_RX
}
N 72000 45700 71400 45700 4
{
T 71300 45700 5 10 1 1 0 7 1
netname=AUX_TX
}
N 58700 44600 59300 44600 4
{
T 59400 44600 5 10 1 1 0 1 1
netname=MOSI
}
N 58700 44000 59300 44000 4
{
T 59400 44000 5 10 1 1 0 1 1
netname=SCK
}
N 58700 43400 59300 43400 4
{
T 59400 43400 5 10 1 1 0 1 1
netname=MISO
}
C 59000 42000 1 0 0 gnd-1.sym
N 58700 42300 59100 42300 4
C 60500 43400 1 0 0 gnd-1.sym
N 58700 43700 60600 43700 4
C 60400 44600 1 270 0 capacitor-1.sym
{
T 61100 44400 5 10 0 1 270 0 1
device=CAPACITOR
T 60700 44300 5 10 1 1 0 0 1
refdes=C14
T 61300 44400 5 10 0 0 270 0 1
symversion=0.1
T 60700 43800 5 10 1 1 0 0 1
value=0.1uF
T 60400 44600 5 10 0 0 0 0 1
footprint=0805
}
C 60400 44600 1 0 0 3V3-plus-1.sym
N 58700 44300 60100 44300 4
N 60100 44300 60100 44600 4
N 60100 44600 60600 44600 4
N 58700 44900 59300 44900 4
{
T 59400 44900 5 10 1 1 0 1 1
netname=\_SDCS\_
}
N 58700 42600 62100 42600 4
{
T 62300 42600 5 10 1 1 0 1 1
netname=\_SDDET\_
}
N 72000 44900 71400 44900 4
{
T 71300 44900 5 10 1 1 0 7 1
netname=AUX_IO
}
N 58600 53400 58000 53400 4
{
T 57900 53400 5 10 1 1 0 7 1
netname=AUX_IO
}
C 61000 50200 1 0 0 crystal-4pin.sym
{
T 61200 50700 5 10 0 0 0 0 1
device=CRYSTAL
T 61200 50900 5 10 0 0 0 0 1
symversion=0.1
T 61000 50200 5 10 0 0 0 0 2
device=ABM3C-11.0592MHZ-D4Y-T
7B-11.0592MAAJ-T
T 61000 50200 5 10 0 0 0 0 1
footprint=crystal-4-smd
T 61450 50850 5 10 1 1 0 3 1
refdes=Y1
T 62100 50600 5 10 1 1 0 1 1
value=11.0592MHz
}
C 61500 49900 1 0 0 gnd-1.sym
C 61200 49900 1 0 0 gnd-1.sym
N 61000 49700 61000 51500 4
N 61000 51500 61100 51500 4
N 61900 49700 61900 51500 4
N 61900 51500 61800 51500 4
N 63000 57200 63600 57200 4
{
T 63700 57200 5 10 1 1 0 1 1
netname=IN_1
}
N 63000 56800 63600 56800 4
{
T 63700 56800 5 10 1 1 0 1 1
netname=IN_2
}
N 58600 53000 58000 53000 4
{
T 57900 53000 5 10 1 1 0 7 1
netname=\_SDDET\_
}
N 58600 57200 58000 57200 4
{
T 57900 57200 5 10 1 1 0 7 1
netname=DATA3
}
N 58600 57600 58000 57600 4
{
T 57900 57600 5 10 1 1 0 7 1
netname=DATA2
}
N 58600 58000 58000 58000 4
{
T 57900 58000 5 10 1 1 0 7 1
netname=DATA1
}
N 58600 58400 58000 58400 4
{
T 57900 58400 5 10 1 1 0 7 1
netname=DATA0
}
N 63000 57600 63600 57600 4
{
T 63700 57600 5 10 1 1 0 1 1
netname=RS
}
N 63000 58000 63600 58000 4
{
T 63700 58000 5 10 1 1 0 1 1
netname=R/W
}
N 63000 58400 63600 58400 4
{
T 63700 58400 5 10 1 1 0 1 1
netname=EN
}
N 58600 56800 58000 56800 4
{
T 57900 56800 5 10 1 1 0 7 1
netname=BKLT_EN
}
N 58600 52200 58000 52200 4
{
T 57900 52200 5 10 1 1 0 7 1
netname=\_SDCS\_
}
C 56000 46900 1 0 0 cpdt6-5v4-2.sym
{
T 56200 49300 5 10 0 0 0 0 1
footprint=SOT26
T 57600 48550 5 10 1 1 0 0 1
refdes=U9
T 57300 48550 5 10 1 1 0 6 1
device=CPDT6-5V4-HF
}
N 58000 48200 58200 48200 4
{
T 58300 48200 5 10 1 1 0 1 1
netname=IN_1
}
C 58200 47400 1 0 0 gnd-1.sym
N 58000 47700 58300 47700 4
C 55600 47400 1 0 0 gnd-1.sym
N 55700 47700 56000 47700 4
N 56000 48200 55800 48200 4
{
T 55700 48200 5 10 1 1 0 7 1
netname=IN_2
}
C 41800 45200 1 270 0 res-pack4-1.sym
{
T 41800 45200 5 10 0 0 270 0 1
slot=3
T 41800 45200 5 10 0 0 270 0 1
footprint=RPACK4-1206
T 41600 44500 5 10 1 1 180 0 1
value=10k
T 41600 44300 5 10 1 1 180 0 1
refdes=R4
}
C 42600 45200 1 270 0 res-pack4-1.sym
{
T 42600 45200 5 10 0 0 270 0 1
slot=4
T 42600 45200 5 10 0 0 270 0 1
footprint=RPACK4-1206
T 42400 44500 5 10 1 1 180 0 1
value=10k
T 42400 44300 5 10 1 1 180 0 1
refdes=R4
}
N 41900 44800 41900 44900 4
N 41900 43800 42700 43800 4
N 42700 44800 42700 44900 4
N 41900 43900 41900 43800 4
N 42700 43900 42700 43800 4
C 54700 45600 1 270 0 res-pack4-1.sym
{
T 54700 45600 5 10 0 0 270 0 1
slot=1
T 54700 45600 5 10 0 0 270 0 1
footprint=RPACK4-1206
T 54500 44900 5 10 1 1 180 0 1
value=10k
T 54500 44700 5 10 1 1 180 0 1
refdes=R4
}
C 53500 45600 1 270 0 res-pack4-1.sym
{
T 53500 45600 5 10 0 0 270 0 1
slot=2
T 53500 45600 5 10 0 0 270 0 1
footprint=RPACK4-1206
T 53300 44900 5 10 1 1 180 0 1
value=10k
T 53300 44700 5 10 1 1 180 0 1
refdes=R4
}
C 61600 43900 1 90 1 res-pack4-1.sym
{
T 61600 43900 5 10 0 0 270 2 1
slot=2
T 61600 43900 5 10 0 0 270 2 1
footprint=RPACK4-1206
T 61200 43200 5 10 1 1 180 0 1
value=10k
T 61200 43000 5 10 1 1 180 0 1
refdes=R3
}
C 61300 43500 1 0 0 3V3-plus-1.sym
N 72000 46100 71400 46100 4
{
T 71300 46100 5 10 1 1 0 7 1
netname=AUX_IO2
}
N 63000 55600 63600 55600 4
{
T 63700 55600 5 10 1 1 0 1 1
netname=AUX_IO2
}
C 47100 59300 1 0 0 ap6320x-1.sym
{
T 48400 61200 5 10 1 1 0 6 1
device=AP63203
T 47400 61950 5 10 0 0 0 0 1
footprint=SOT26
T 48400 61400 5 10 1 1 0 6 1
refdes=U1
T 50500 60400 5 10 0 1 0 6 1
footprint=SOT26
}
