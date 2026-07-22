/*
 * i2c.h - ATmega2560 TWI 마스터 (VL53L0X 용)
 */
#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void    i2c_init(void);                 /* 100kHz */
uint8_t i2c_start(uint8_t addr_rw);     /* START + SLA. addr_rw = (7bit<<1)|R/W. 반환=TWSR 상태 */
void    i2c_write(uint8_t data);
uint8_t i2c_read_ack(void);             /* 읽고 ACK (다음 바이트 더 있음) */
uint8_t i2c_read_nack(void);            /* 읽고 NACK (마지막 바이트) */
void    i2c_stop(void);

#endif
