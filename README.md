# FreeRTOS-TrafficLight
FreeRTOS simulation of a Traffic Light

This Raspberry Pi Pico code exercise simulates a North-South and East-West intersection Traffic Light.
The light switches through the GREEN YELLOW RED sequence with a pause where both
directions show RED for a short pause as a safety measure.

As well, a switch is included to forcefully provide a blinking RED in both directions. This
blinking RED is active after the YELLOW to allow the traffic to stop co-operatively (Not 
commanding one direction to stop directly from a GREEN state).

The code includes a Switch Debounce Task which provides for many combinations of switches to
be used.  Any switch combinations you do not need can be removed to reduce the size of the code.

The I/O is implemented through a PCF8575 i2s port extender driving a seven segment LED on my 
experimental breadboard, although any six LEDs would work.
