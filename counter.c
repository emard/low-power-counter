#include <avr/io.h> 
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
// #include <util/crc16.h>

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

// encoded packet to be sent, bit-slice
union packet
{
  struct
  {
    // uint64_t preamble:16, unknown:4, serial:20, data:8, crc:16;
    uint64_t crc:16, data:8, serial:20, unknown:4, preamble:16;
  };
  uint64_t binary;
};

uint8_t pin_state, pin_change; // ISR tracks pin state and sets bits of pin_change to 1

void record_read(struct record *r, uint8_t i)
{
  eeprom_read_block(r, &(memory[i % N_RECORDS]), sizeof(struct record));
}

void record_write(struct record *r, uint8_t i)
{
  eeprom_update_block(r, &(memory[i % N_RECORDS]), sizeof(struct record));
}

void counter_reset()
{
  uint8_t i;
  memset(counter, 0, sizeof(struct record));
  for(i = 0; i < N_RECORDS; i++)
    record_write(counter, i);
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
ISR(PCINT0_vect)
{
  uint8_t pin_current = PINB & INPUT;
  pin_change |= pin_current ^ pin_state;
  pin_state = pin_current;
  // executing ISR will automatically clear its interrupt flag.
  // it is not neccessary to manually clear interrupt flag:
  // PIN_CHANGE_FLAG_CLEAR();
}

// store the counter using eeprom record storage
void store_counter()
{
 uint8_t j, k;

 j = find_free_record(NULL);
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

uint16_t crc16_ccitt(uint8_t const message[], unsigned nBytes, uint16_t polynomial, uint16_t init) {
    uint16_t remainder = init;
    unsigned byte, bit;

    for (byte = 0; byte < nBytes; ++byte) {
        remainder ^= message[byte] << 8;
        for (bit = 0; bit < 8; ++bit) {
            if (remainder & 0x8000) {
                remainder = (remainder << 1) ^ polynomial;
            }
            else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder;
}

void update_crc(union packet *p)
{
  uint8_t i;
  uint8_t bb[6], *b;
  b = (uint8_t *) &(p->binary);
  for(i = 0; i < 6; i++)
    bb[i] = ~b[7-i];
  p->crc = ~crc16_ccitt(bb, 6, 0x8005, 0xfffe);
}

void transmit(uint8_t i)
{
  uint8_t j;
  union packet tx;

  // Construct the 64-bit TX packet
  tx.preamble = ~0xFFFE;
  tx.unknown = 0x7;
  tx.serial = 0x55664L;
  tx.data = 0x3F;
  tx.crc = 0x277D;
  update_crc(&tx);

  // use timer0 
  TCNT0 = 0;
  OCR0A = 170;
  TCCR0A = 2; // ctc mode - TCNT0 runs from 0 to OCR0A
  TCCR0B = 1; // prescaler 1 (clk), 2 (clk/8), 3 (clk/64), 4 (clk/256), 5 (clk/1024)
 
  TIFR = 1<<OCF0A; // reset timer overflow interrupt flag
 
  // blink LED send binary counter, send LSB first
  for(j = 0; j < 64; j++) // send 64-bit packet manchester encoded, MSB first
  {
    uint8_t ledstate = (tx.binary & (1ULL<<63)) != 0 ? 1 : 0; // MSB
    while((TIFR & (1<<OCF0A)) == 0); // wait for interrupt flag
    TIFR = 1<<OCF0A; // reset timer overflow interrupt flag
    if(ledstate != 0)
      PORTB |= LED; // LED ON
    else
      PORTB &= ~LED; // LED off
    while((TIFR & (1<<OCF0A)) == 0); // wait for interrupt flag
    TIFR = 1<<OCF0A; // reset timer overflow interrupt flag
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    PINB = LED; // invert the led and wait again -> manchester encoding
    tx.binary <<= 1; // shift next bit
  }
  while((TIFR & (1<<OCF0A)) == 0); // wait for interrupt flag
  TIFR = 1<<OCF0A; // reset timer overflow interrupt flag
  PORTB &= ~LED; // led OFF
}

void update_pin_state()
{
  uint8_t i;
  pin_state = 0;
  for(i = N_CHANNELS-1; i != 0xff; i--)
  {
    pin_state <<= 1;
    pin_state |= counter->c[i] & 1; // LSB of the counter is the pin state
  }
}

void delay(uint8_t n)
{
  uint8_t j;
  TCNT0 = 0;
  OCR0A = 120;
  TCCR0A = 2;
  TCCR0B = 1; // prescaler 4 (clk/256), 5 (clk/1024)
 
  TIFR = 1<<OCF0A; // reset timer overflow interrupt flag
  for(j = 0; j < n; j++)
  {
    while((TIFR & (1<<OCF0A)) == 0); // wait for interrupt flag
    TIFR = 1<<OCF0A; // reset timer overflow interrupt flag
  }
}

void main()
{
 // as early as possible, check the MCUSR to detect external reset
 // upon external reset, reset the EEPROM counter state
 #if 0
 uint8_t mcusr = MCUSR;
 MCUSR = 0; // clear it
 if((mcusr | EXTRF) != 0) // external reset -- clear eeprom
   counter_reset();
 #endif

 delay(16); // wait a second for power to become stable
 
 ADC_DISABLE();
 set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
 DDRB = LED; // only LED output, other pins input
 PORTB = INPUT; // pullup enable on input pins, led OFF

 #if 1
 find_free_record(counter);
 update_pin_state(); // read pin state from last record in eeprom
 #endif

 PIN_CHANGE_FLAG_CLEAR();
 PIN_CHANGE_INTERRUPT_ENABLE();
 PIN_CHANGE_MONITOR(INTERRUPT_PINS); // this is PB0-PB3 inputs
 // should already set monitored pin as input and enable its pull up
 PIN_CHANGE_FLAG_CLEAR();

 // initialize pin change, compare stored state to current state
 pin_change = (PINB & INPUT) ^ pin_state;
 
 for(;;)
 {
   #if 1
   uint8_t j; // loop counter
   uint8_t tx = 0; // bitmap of which counters changed and should be transmitted
   delay(100); // time for de-bounce
   while(pin_change != 0)
   {
     uint8_t change = pin_change, m = 1; // m is shifting 1-bit bitmask
     uint8_t new_pin_state = PINB & INPUT;

     for(j = 0; j < N_CHANNELS; j++)
     {
       if( (change & m) != 0 || ((new_pin_state ^ pin_state) & m) != 0 )
       {
         counter->c[j]++;
         update_pin_state();
         #if 1
         if( ((new_pin_state ^ pin_state) & m) != 0)
         {
           counter->c[j]++;
           update_pin_state();
         }
         #endif
         cli();
         pin_change &= ~m; // clear pin change bit (change serviced)
         sei();
         tx |= m; // transmit this bit
       }
       m <<= 1; // upshift mask
     }
     delay(100); // allow for pins to change again
   }

   if(tx != 0)
   {
     store_counter();
     for(j = 0; j < N_CHANNELS; j++)
     {
       if( (tx & 1) != 0 )
         transmit(j);
       tx >>= 1;
     }
   }
   #else
   // this works for single input
   counter->c[0]++;
   store_counter();
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
   // PIN_CHANGE_FLAG_CLEAR();
   sei();
 }
}
