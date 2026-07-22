/*
 * vl53l0x.h - VL53L0X ToF 거리센서 드라이버 (순수 C / TWI)
 * Pololu vl53l0x-arduino (MIT) 의 레지스터 시퀀스를 C로 포팅.
 * I2C 7bit 주소 0x29 고정.
 */
#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdint.h>

/* 초기화. 성공 1 / 실패 0 (모델ID 불일치 등) */
uint8_t vl53l0x_init(void);

/* 단발(single-shot) 측정 mm. 타임아웃/범위초과 시 큰 값 반환 */
uint16_t vl53l0x_read_range_single(void);

/* I/O 타임아웃(ms). 0이면 무한대기 */
void vl53l0x_set_timeout(uint16_t ms);
/* 마지막 측정에서 타임아웃 발생했는지 (읽으면 플래그 클리어) */
uint8_t vl53l0x_timeout_occurred(void);

#endif
