#include <avr/io.h> 
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>

#define LED (1<<PB1)

enum 
{ 
  EEPROM_BYTES = 512, // eeprom size in bytes, see datasheet
  N_CHANNELS = 4, // number of channels to track
};

// counter data struct
struct record
{
  int32_t c[N_CHANNELS];
};

// calculate how many records can we store
enum
{
  N_RECORDS = EEPROM_BYTES/sizeof(struct record),
};

struct record EEMEM memory[N_RECORDS]; // storage in EEPROM

struct record counter[1]; // one record as counter in RAM

void record_read(struct record *r, uint8_t i)
{
  eeprom_read_block(r, &(memory[i % N_RECORDS]), sizeof(struct record));
}

void record_write(struct record *r, uint8_t i)
{
  eeprom_update_block(r, &(memory[i % N_RECORDS]), sizeof(struct record));
}

// find free data in eeprom
// that's first data which has 0
// also read the last non-free record
uint8_t find_free_record(struct record *r)
{
  uint8_t i;
  uint8_t nonzero_found = 0;
  struct record data[1];
  for(i = 0; i < N_RECORDS; i++)
  {
    uint8_t j, z = N_CHANNELS;
    // eeprom_read_block(data, &(memory[i % N_RECORDS]), sizeof(struct record));
    record_read(data, i);
    for(j = 0; j < N_CHANNELS; j++)
    {
      if(data->c[j] == 0)
        z--;
    }
    // all values zero -> found new free entry
    if(z == 0)
    {
      if(nonzero_found == 1)
      {
        return i;
      }
    }
    else
    {
      nonzero_found = 1;
      if(r)
        memcpy(r, data, sizeof(struct record));
    }
  }
  return 0;
}

// interrupt service routine: PIN CHANGE 0 interrupt
// on attiny84 it covers 8 input pins PA 0-7
// there also exists PIN CHANGE 1 interrupt which
// covers another 4 input pins PB 0-3
// on attiny85 only PIN CHANGE 0 exists
#if 1
ISR(PCINT0_vect)
{
}
#endif

void main()
{
 int i;
 unsigned char t = LED;
 
 uint8_t j, k;
 j = find_free_record(counter);
 // record_read(counter, j);
 counter->c[0]++; // ncrement it
 record_write(counter, j);
 // next position
 if(++j >= N_RECORDS)
   j = 0;
 // create 0-delimiter
 for(k = 0; k < N_CHANNELS; k++)
   counter->c[k] = 0;
 // write 0-delimiter to next position
 record_write(counter, j);

 DDRB = t;

 for(;;)
 {
   // blink 10 times
   for(j = 0; j < 11; j++)
   {
     PORTB = t;
     t ^= LED;
     for(i = 0; i < 10000; i++)
       asm("nop");
   }

   #if 1
   set_sleep_mode(
     1*SLEEP_MODE_IDLE
   | 1*SLEEP_MODE_ADC
   | 1*SLEEP_MODE_PWR_DOWN
   // | 1*SLEEP_MODE_STANDBY // not available on attiny85
   );
   #endif
   cli();
   if(1 == 1)
   {
     sleep_enable();
     sei();
     sleep_cpu();
     sleep_disable();
   }
   sei();
   // for(;;);
 }
}
