#include <avr/io.h> 
#include <stdlib.h>
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

// find free data in eeprom
// that's first data which has 0
uint8_t find_free_record()
{
  uint8_t i;
  struct record data[1];
  for(i = 0; i < N_RECORDS; i++)
  {
    int j, z = N_CHANNELS;
    eeprom_read_block(data, memory+i, sizeof(struct record));
    for(j = 0; j < N_CHANNELS; j++)
    {
      if(data->c == 0)
        z--;
    }
    // all values zero -> found new free entry
    if(z == 0)
      return i;
  }
  return 0;
}

void record_read(struct record *r, uint8_t i)
{
  eeprom_read_block(r, memory+i, sizeof(struct record));
}

void record_write(struct record *r, uint8_t i)
{
  eeprom_update_block(r, memory+i, sizeof(struct record));
}

void main()
{
 int i;
 unsigned char t = LED;
 
 uint8_t j, k;
 j = find_free_record();
 record_read(counter, j);
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
 
 set_sleep_mode(
   1*SLEEP_MODE_IDLE
 | 1*SLEEP_MODE_ADC
 | 1*SLEEP_MODE_PWR_DOWN
 | 1*SLEEP_MODE_STANDBY
 );

 for(;;)
 {
   // blink 10 times
   for(j = 0; j < 10; j++)
   {
     PORTB = t;
     t ^= LED;
     for(i = 0; i < 10000; i++);
   }
   sleep_enable();
 }
}
