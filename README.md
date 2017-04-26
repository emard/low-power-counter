# Low power counter with ATTINY85

Intended for counting slow pulses which
occur once in few seconds or slower.

Counter keeps its state in EEPROM
which retains data when unpowered 
(without battery).

Counts up to 4 input channels and sends
them over 1 multiplexed output line.

Runs on 3V lithium battery like CR2032.
In standby state it enters low power mode (below 1uA),
what makes a long battery life.

Everything runs on a small AVR ATTINY85 microcontroller,
a single chip in 8-pin package.

# Input and Output

It counts every transition of the input state. It will trigger
on both rising ad falling edges. So it will update counter
when input chanes from logical 0 to 1 and also back from 1 to 0.

Too fast events >0.5Hz will not be counted, for example mechanical
contact ringing of the input switch will be filtered out.

After each input pulse it encodes new counter state over
a serial digital line. Output is a bit sequence 
containing header, channel number, counter value and CRC.
Output is intended to be connected to a simple
433 MHz transmitter and received remotely with 
some compatible receiver or RTL-SDR.

# Power saving issues

When designing the circuit take into account that everything
draws some power, so for example when internal pull up resistors
in the chip are enabled, they will draw about 80uA per input line
connected to GND.

During 3V battery life, voltage will drop down to about 2V until
battery is completely exhausted, so we recommend using low-voltage 
version ATTINY85V which is designed to work down to 1.8V.
Normal ATTINY85 seems to work at 2V though.

The output transmitter should also be designed to work with 
battery voltage below 2V and to enter low power mode when idle.
