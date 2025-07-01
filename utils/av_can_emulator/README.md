This is the code for emulating AV on an STM32 F446RE to use normal mode with the pcb (being payload) in main

You should connect the stm32 with the usual canbus setup
	- PA11 to Rx
	- PA12 to Tx
	- VCC to 3v3 power source
	- GND to ground pin
	- ~120 ohm resistor in between can high and can low

connect to the pcb canh (red) and canl (blue) and run the code in "main" or "payload_CAN" in rnd folder

open putty to check messages are recieved/sent on both boards