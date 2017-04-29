# Low power counter with ATTINY85V

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

Everything runs on a small AVR ATTINY85V microcontroller,
a single chip in 8-pin package.

# Input and Output

It counts every transition of the input state. It will trigger
on both rising and falling edges. So it will update counter
when input changes from logical 0 to 1 and also back from 1 to 0.

Chip ATTINY85V layout

    nRESET |1  \/  8| +3V
       IN3 |2      7| IN2
       OUT |3      6| IN1
       GND |4      5| IN0

Too fast events >0.5Hz will not be counted, for example mechanical
contact ringing of the input switch will be filtered out.

After each input pulse, new counter state is sent over
the serial digital line. Output is a timed bit sequence 
containing header, channel number, counter value and CRC.
Output is intended to be connected to a simple
on-off keying (OOK) 433 MHz transmitter and received remotely with
some general-purpose receiver like
[rtl_433](https://github.com/merbanan/rtl_433.git)
(mostly compatible with honeywell.c protocol decoder)
see also
[ArduinoWeatherOS](https://github.com/robwlakes/ArduinoWeatherOS)

# Power saving issues

When designing the circuit take into account that every current flow
draws some power, so for example when internal pull-up resistors
in the chip are enabled, each input connected to GND will draw about 
80uA. Option is to disable internal pull-up and connect external
1Mohm pull-up resistor.

During 3V battery life, voltage will drop down to about 2V.
Low-voltage version of the chip "ATTINY85V" is recommended
because it is designed to work down to 1.8V.
Normal "ATTINY85" is designed down to 2.7V but it 
seems to work at 2V though.

The output transmitter should also be designed to work with
battery voltage below 2V and have low power mode when idle.
SYN115 433MHZ ASK Transmitter Module works well.

# Todo

    [ ] Reset all counters with nRESET
