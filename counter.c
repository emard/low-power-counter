#include <avr/io.h> 
#include <stdlib.h>
#include <avr/eeprom.h>

#define LED (1<<PB1)

enum 
{ 
  EEPROM_BYTES = 512, // eeprom size in bytes, see datasheet
  N_CHANNELS = 4, // number of channels to track
};

// counter data struct
struct counter
{
  int32_t c[N_CHANNELS];
};

// find free data in eeprom
// that's first data which has 0
int find_free()
{
  return 0;
}

void update_data()
{
  eeprom_update_block (NULL, NULL, 4);
}

void main()
{
 int i;
 unsigned char t = LED;

 DDRB = t;

 for(;;)
 {
   PORTB = t;
   t ^= LED;
   for(i = 0; i < 10000; i++);
 }
}
