/*
 * uart.h - USART0(디버그) + USART1(ESP32) 드라이버
 */
#ifndef UART_H
#define UART_H

#include <stdint.h>

/* USART0 : 디버그 콘솔 (Serial 대체) */
void uart0_init(void);
void uart0_putc(char c);
void uart0_puts(const char *s);
void uart0_put_long(long v);    /* 부호 있는 정수 출력 */
void uart0_nl(void);            /* 줄바꿈 */

/* USART1 : ESP32 명령 (Serial1 대체) */
void uart1_init(void);
void uart1_putc(char c);
void uart1_puts(const char *s);
void uart1_println(const char *s); /* s + CRLF */

#endif
