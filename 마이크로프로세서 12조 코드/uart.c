/*
 * uart.c
 * 115200 bps @ 16MHz. util/setbaud.h 로 UBRR/U2X 자동 계산.
 */
#include "config.h"
#include <avr/io.h>
#include <stdlib.h>
#include "uart.h"

/* setbaud 설정 */
#define BAUD     UART_BAUD
#define BAUD_TOL 3            /* 115200@16MHz 는 약 2.1% 오차 -> 허용오차 3% */
#include <util/setbaud.h>

/* ---------------- USART0 (디버그) ---------------- */
void uart0_init(void)
{
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= (1 << U2X0);
#else
    UCSR0A &= ~(1 << U2X0);
#endif
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* 8N1 */
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);   /* TX/RX enable */
}

void uart0_putc(char c)
{
    while (!(UCSR0A & (1 << UDRE0))) { }
    UDR0 = c;
}

void uart0_puts(const char *s)
{
    while (*s) uart0_putc(*s++);
}

void uart0_put_long(long v)
{
    char buf[12];
    ltoa(v, buf, 10);
    uart0_puts(buf);
}

void uart0_nl(void)
{
    uart0_putc('\r');
    uart0_putc('\n');
}

/* ---------------- USART1 (ESP32) ---------------- */
void uart1_init(void)
{
    UBRR1H = UBRRH_VALUE;
    UBRR1L = UBRRL_VALUE;
#if USE_2X
    UCSR1A |= (1 << U2X1);
#else
    UCSR1A &= ~(1 << U2X1);
#endif
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); /* 8N1 */
    UCSR1B = (1 << TXEN1) | (1 << RXEN1);
}

void uart1_putc(char c)
{
    while (!(UCSR1A & (1 << UDRE1))) { }
    UDR1 = c;
}

void uart1_puts(const char *s)
{
    while (*s) uart1_putc(*s++);
}

void uart1_println(const char *s)
{
    uart1_puts(s);
    uart1_putc('\r');
    uart1_putc('\n');
}
