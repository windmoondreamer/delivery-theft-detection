/*
 * millis.h - Arduino millis()/delay() 대체 (Timer0 오버플로 인터럽트)
 */
#ifndef MILLIS_H
#define MILLIS_H

#include <stdint.h>

void     millis_init(void);     /* Timer0 설정. 호출 후 sei() 필요 */
uint32_t millis(void);          /* 부팅 후 경과 ms */
void     delay_ms(uint32_t ms); /* millis 기반 블로킹 지연 */

#endif
