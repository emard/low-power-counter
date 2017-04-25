#include <avr/io.h> 
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>

#define LED (1<<PB4)
#define INPUT ((1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3))
#define INTERRUPT_PINS ((1<<PCINT0)|(1<<PCINT1)|(1<<PCINT2)|(1<<PCINT3))

#define ADC_DISABLE() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
#define PIN_CHANGE_INTERRUPT_ENABLE() (GIMSK |= (1<<PCIE))
#define PIN_CHANGE_MONITOR(n) (PCMSK = n) // example: use (PCINT2 | PCINT3) to monitor PB2 and PB3 
#define PIN_CHANGE_FLAG_CLEAR() (GIFR = PCIF)

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
#if 0
ISR(PCINT0_vect)
{
  // executing ISR will automatically clear
  // interrupt flag - no need to manually clear it like this:
  // PIN_CHANGE_FLAG_CLEAR();
}
#endif

// increment ith counter using eeprom record storage
void increment(uint8_t i)
{
 uint8_t j, k;
 // struct record counter[1];

 j = find_free_record(counter);
 counter->c[i]++; // ncrement it
 record_write(counter, j);
 // next position, wraparound
 if(++j >= N_RECORDS)
   j = 0;
 // create 0-delimiter
 struct record delimiter[1];
 for(k = 0; k < N_CHANNELS; k++)
   delimiter->c[k] = 0;
 // write 0-delimiter to next position
 record_write(delimiter, j);
}

void transmit(uint8_t i)
{
 uint8_t j;
 uint32_t d, value;

 value = counter->c[i];
 // blink LED send binary counter, LSB first
 for(j = 0; j < 8; j++)
 {
   if((value & 1) != 0)
     PORTB |= LED; // LED ON
   else
     PORTB &= ~LED; // LED off
   value >>= 1; // downshift
   for(d = 0; d < 10000; d++)
     asm("nop");
 }
 PORTB &= ~LED; // led OFF
}

void delay()
{
 uint32_t d;
 for(d = 0; d < 10000; d++)
   asm("nop");
}

void main()
{
 uint8_t old_pin_state, new_pin_state;

 ADC_DISABLE();
 set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
 DDRB = LED; // only LED output, other pins input
 PORTB = INPUT; // pullup enable on input pins, led OFF

 PIN_CHANGE_FLAG_CLEAR();
 PIN_CHANGE_INTERRUPT_ENABLE();
 PIN_CHANGE_MONITOR(INTERRUPT_PINS); // this is PB0-PB3 inputs
 // should already set monitored pin as input and enable its pull up
 PIN_CHANGE_FLAG_CLEAR();

 old_pin_state = PINB & INPUT;
 new_pin_state = old_pin_state;

 for(;;)
 {
   #if 0
   delay();
   new_pin_state = PINB & INPUT;
   while(new_pin_state != old_pin_state)
   {
     uint8_t j, change;
     change = old_pin_state ^ new_pin_state;
     for(j = 0; j < N_CHANNELS; j++)
     {
       if( (change & 1) != 0)
       {
         increment(j);
         transmit(j);
       }
       change >>= 1; // downshift
     }
     old_pin_state = new_pin_state;
     delay();
     new_pin_state = PINB & INPUT;
   }
   #else
   // this works well
   increment(0);
   transmit(0);
   #endif

   cli();
   if(1 == 1)
   {
     sleep_enable();
     sei();
     sleep_cpu();
     sleep_disable();
   }
   // if we don't have ISR registered,
   // we need to manually clear interrupt flag:
   PIN_CHANGE_FLAG_CLEAR();
   sei();
 }
}
