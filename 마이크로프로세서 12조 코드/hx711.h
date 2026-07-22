/*
 * hx711.h - HX711 로드셀 ADC (비트뱅잉, 채널A gain 128)
 */
#ifndef HX711_H
#define HX711_H

#include <stdint.h>

void hx711_init(void);
uint8_t hx711_is_ready(void);
long hx711_read(void);                      /* 24bit 부호확장 1회 읽기 */
long hx711_read_average(uint8_t times);     /* 평균 (원본 호환) */
long hx711_read_median(uint8_t times);      /* 중앙값 (글리치 제거용) */
long hx711_get_value(uint8_t times);        /* read_average - offset */
long hx711_get_value_median(uint8_t times); /* read_median  - offset */
void hx711_tare(uint8_t times);

#endif
