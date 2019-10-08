#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define TRUE 1
#define FALSE 0

#define HIGH 1
#define LOW 0

void init_millis();

unsigned long millis();
unsigned long micros();

uint8_t digitalRead(uint8_t pin);
int8_t my_random(int8_t min, int8_t max);
