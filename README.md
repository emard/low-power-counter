# Low power remote counter suitable for 433 MHz

Intended to monitor water counter meters that
make reed relay contact every 100 liters.

It can independently track 4 relay contacts.
On each relay pulse it increments the counter in EEPROM
and transmits new counter state with preamble and CRC 
over the 1-bit pin. After several transmissions, it
enters low power state (sleep).

It can be used to drive small CW transmitter at 433 MHz.
