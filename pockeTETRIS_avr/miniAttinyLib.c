#include "miniAttinyLib.h"

volatile unsigned long milliseconds;

ISR(TIMER1_COMPA_vect)
{
	milliseconds++;
}

unsigned long millis()
{
	return milliseconds;
}

void init_millis()
{
	// CTC Interrupt every 1ms
	TCCR1 = 1<<CTC1 | 7<<CS10;
	OCR1C = 0.001 * F_CPU/64 - 1;

	// enable interrupt
	TIMSK = 1<<OCIE1A;
}

/*
unsigned long micros() {
	unsigned long m;
	uint8_t oldSREG = SREG, t;

	cli();
	m = timer0_overflow_count;
	t = TCNT0;


	if ((TIFR0 & _BV(TOV0)) && (t < 255))
		m++;

	SREG = oldSREG;

	return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}
*/

uint8_t digitalRead(uint8_t pin) {
    return (PORTB >> pin) && 1;
}

int8_t my_random(int8_t min, int8_t max) {
    return min + (int)((double)rand() / ((double)RAND_MAX + 1) * max);
}
