Consider the following scenario:

The garage is located a few meters from the house. The electric garage door is currently operated by a push button inside the house, connected to the garage door motor via a two-wire cable. Pressing the button closes the motor control circuit.

Because the control signal is transmitted directly over this cable, anyone with physical access to it could open the garage door by simply short-circuiting the wires, creating a security risk.

This project addresses the vulnerability by relocating the closing of the motor control circuit to the garage itself, rather than relying on the unsecured cable from the house. Two Raspberry Pi Pico microcontrollers are installed at each end of the cable and communicate with each other using encrypted communication over the existing wiring.

The Raspberry Pi Pico located in the garage is responsible for closing the motor control circuit, while the Raspberry Pi Pico in the house detects when the button is pressed and sends the corresponding encrypted command.

