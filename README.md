Summary
=======

Arduino sketch permitting read/write access to the 93C46 EEPROM attached to KSZ8851 SPI MAC+PHY IC.

  - Permits programming the MAC address into EEPROM without need for a separate programmer.
  - Allows the microcontroller to self-program the persistent MAC address.
  - Tested using ESP32-S3-DevKitC-1 and KSZ8851SNL Evaluation Board revision 1.9 (by Ewan Parker).

Usage
=====

EEPROM disabled
---------------
If the EEPROM access is disabled (removing jumper **EE** on evaluation board) then the response shows the randomly generated MAC.  It is not possible to access the EEPROM in this configuration.
```
93C46 EEPROM read and write via KSZ8851SNL over SPI.
Copyright (C) 2024 Ewan Parker.
https://www.ewan.cc/

Current State
EEPROM present: No
Current MAC:    d6-f5-52-aa-ed-94
```

New EEPROM
----------
A fresh EEPROM will be initialized to all 1's (FF in hex).  You will be presented with the following display (just press **RETURN** if your display is blank.
```
Current State
EEPROM present: Yes
Current MAC:    ff-ff-ff-ff-ff-ff
EEPROM MAC:     ff-ff-ff-ff-ff-ff

Menu
Choose 'w' to write EEPROM: 
```
The **Current MAC** is the effective MAC found in the registers of the MAC IC.  The **EEPROM MAC** is the address read from the EEPROM.  It may or may not be in effect.  The MAC IC reads the EEPROM once at power-up.

Programming
-----------
Press **W** to enter a new MAC, then type the address.  You will gradually overtype the old MAC.  Any key other than **0** to **F** aborts.
```
Choose 'w' to write EEPROM: W
Input MAC:      7c-df-a1-ff-ff-ff
```

You will see a confirmation and the EEPROM will be re-read.
```
Input MAC:      7c-df-a1-ab-cd-ef

EEPROM written

Current State
EEPROM present: Yes
Current MAC:    ff-ff-ff-ff-ff-ff
EEPROM MAC:     7c-df-a1-ab-cd-ef

Menu
Choose 'w' to write EEPROM: 
```

The effective MAC is unchanged.  A power cycle of the KSZ8851 is required for a re-read of the EEPROM into the MAC's registers.
```
Current State  
EEPROM present: Yes  
Current MAC:    7c-df-a1-ab-cd-ef  
EEPROM MAC:     7c-df-a1-ab-cd-ef
```
