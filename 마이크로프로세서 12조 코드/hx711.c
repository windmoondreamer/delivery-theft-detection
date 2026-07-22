/*
 * hx711.c
 * DT(PE4) 입력, SCK(PE5) 출력. 24비트 + gain 128 펄스 1회.
 *
 * v2: 클럭을 늦춰(HX711_CLK_US) 아두이노 digitalWrite 수준으로 맞춤.
 *     너무 빠른 클럭은 상위 비트 오독(값이 수만으로 튐)을 유발하므로 느리게.
 *     read_median 으로 튀는 샘플 1개를 버릴 수 있게 함.
 */
#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "hx711.h"

/* SCK 반주기 지연(us). 클수록 느리고 안정적. 글리치 계속되면 키울 것 (예: 8) */
#define HX711_CLK_US 5

static long offset = 0;

#define SCK_HIGH()  (HX711_SCK_PORT |=  (1 << HX711_SCK_BIT))
#define SCK_LOW()   (HX711_SCK_PORT &= ~(1 << HX711_SCK_BIT))
#define DT_READ()   (HX711_DT_PIN  &   (1 << HX711_DT_BIT))

void hx711_init(void)
{
    HX711_SCK_DDR |=  (1 << HX711_SCK_BIT);  /* SCK 출력 */
    HX711_DT_DDR  &= ~(1 << HX711_DT_BIT);   /* DT  입력 */
    SCK_LOW();
}

uint8_t hx711_is_ready(void)
{
    return (DT_READ() == 0);
}

long hx711_read(void)
{
    while (!hx711_is_ready()) { }

    uint8_t data[3] = {0, 0, 0};
    uint8_t sreg = SREG;
    cli();

    for (int8_t j = 2; j >= 0; j--) {
        uint8_t b = 0;
        for (int8_t i = 7; i >= 0; i--) {
            SCK_HIGH();
            _delay_us(HX711_CLK_US);   /* DOUT 안정화 시간 확보 */
            if (DT_READ()) b |= (1 << i);
            SCK_LOW();
            _delay_us(HX711_CLK_US);
        }
        data[j] = b;
    }

    /* gain 128 (채널 A) = 펄스 1회 */
    SCK_HIGH(); _delay_us(HX711_CLK_US);
    SCK_LOW();  _delay_us(HX711_CLK_US);

    SREG = sreg;

    uint8_t filler = (data[2] & 0x80) ? 0xFF : 0x00;
    long value = ((long)filler << 24)
               | ((long)data[2] << 16)
               | ((long)data[1] << 8)
               |  (long)data[0];
    return value;
}

long hx711_read_average(uint8_t times)
{
    long sum = 0;
    for (uint8_t i = 0; i < times; i++) sum += hx711_read();
    return sum / (long)times;
}

/* 작은 N(보통 3~5)용 단순 선택정렬 후 가운데 값 */
long hx711_read_median(uint8_t times)
{
    if (times == 0) return 0;
    if (times == 1) return hx711_read();

    long buf[9];                 /* 최대 9개 */
    if (times > 9) times = 9;
    for (uint8_t i = 0; i < times; i++) buf[i] = hx711_read();

    for (uint8_t i = 0; i < times - 1; i++) {
        uint8_t m = i;
        for (uint8_t j = i + 1; j < times; j++)
            if (buf[j] < buf[m]) m = j;
        long t = buf[i]; buf[i] = buf[m]; buf[m] = t;
    }
    return buf[times / 2];
}

long hx711_get_value(uint8_t times)
{
    return hx711_read_average(times) - offset;
}

long hx711_get_value_median(uint8_t times)
{
    return hx711_read_median(times) - offset;
}

void hx711_tare(uint8_t times)
{
    offset = hx711_read_median(times);   /* tare 도 중앙값으로 안정화 */
}
