/*
 * i2c.c - 폴링 방식 TWI 마스터
 * SCL = 100kHz : TWBR = (F_CPU/SCL - 16)/2 = (16M/100k - 16)/2 = 72, 프리스케일러 1
 */
#include "config.h"
#include <avr/io.h>
#include "i2c.h"

void i2c_init(void)
{
    TWSR = 0x00;   /* 프리스케일러 1 */
    TWBR = 72;     /* 100 kHz */
    TWCR = (1 << TWEN);
}

uint8_t i2c_start(uint8_t addr_rw)
{
    /* START 전송 (반복 START 도 동일) */
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) { }

    /* SLA+R/W 전송 */
    TWDR = addr_rw;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) { }

    return (TWSR & 0xF8);
}

void i2c_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) { }
}

uint8_t i2c_read_ack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT))) { }
    return TWDR;
}

uint8_t i2c_read_nack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) { }
    return TWDR;
}

void i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    while (TWCR & (1 << TWSTO)) { }
}
