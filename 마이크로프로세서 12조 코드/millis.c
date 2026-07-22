/*
 * millis.c
 * Timer0, 프리스케일러 64 -> 오버플로마다 (64*256)/16MHz = 1024us = 1.024ms
 * 아두이노 wiring.c 와 동일한 분수 누적 방식으로 정확한 ms 유지.
 */
#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define MICROSECONDS_PER_OVF ((64UL * 256UL) / (F_CPU / 1000000UL)) /* = 1024 */
#define MILLIS_INC  (MICROSECONDS_PER_OVF / 1000)        /* = 1 */
#define FRACT_INC   ((MICROSECONDS_PER_OVF % 1000) >> 3) /* = 3 */
#define FRACT_MAX   (1000 >> 3)                           /* = 125 */

static volatile uint32_t timer0_millis = 0;
static volatile uint8_t  timer0_fract  = 0;

ISR(TIMER0_OVF_vect)
{
    uint32_t m = timer0_millis;
    uint8_t  f = timer0_fract;

    m += MILLIS_INC;
    f += FRACT_INC;
    if (f >= FRACT_MAX) {
        f -= FRACT_MAX;
        m += 1;
    }
    timer0_fract  = f;
    timer0_millis = m;
}

void millis_init(void)
{
    /* Timer0 normal mode, 프리스케일러 64 (CS01|CS00), 오버플로 인터럽트 ON */
    TCCR0A = 0x00;
    TCCR0B = (1 << CS01) | (1 << CS00);
    TIMSK0 = (1 << TOIE0);
    TCNT0  = 0;
}

uint32_t millis(void)
{
    uint32_t m;
    uint8_t  sreg = SREG;
    cli();
    m = timer0_millis;
    SREG = sreg;
    return m;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while ((millis() - start) < ms) { /* busy wait */ }
}
