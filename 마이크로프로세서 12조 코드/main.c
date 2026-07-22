/*
 * main.c - 택배 도난 감지 상태머신 (순수 AVR / ATmega2560)
 */
#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "millis.h"
#include "uart.h"
#include "i2c.h"
#include "hx711.h"
#include "vl53l0x.h"

#define CONFIRM_N 3

typedef enum { ST_IDLE, ST_ARRIVED, ST_MONITORING, ST_ALERT, ST_COOLDOWN } State;

static State    state = ST_IDLE;
static uint32_t stateEnterMs = 0;
static uint32_t lastLogMs    = 0;
static long     baselineWeight = 0;
static int      baselineDist   = 0;

static uint8_t presentCnt = 0;
static uint8_t absentCnt  = 0;
static uint8_t theftCnt   = 0;

static long readWeight(void)
{
    if (!hx711_is_ready()) return 0;
    return -hx711_get_value_median(3);
}

static int readDistanceMM(void)
{
    uint16_t r = vl53l0x_read_range_single();
    if (vl53l0x_timeout_occurred()) return 9999;
    if (r > 2000) return 9999;
    if (r < 30)   return 9999;
    return (int)r;
}

static uint8_t distInvalid(int d)
{
    return (d == 9999 || d == 0);
}

static void sendCommand(const char *cmd)
{
    uart1_println(cmd);
    uart0_puts(">>> CMD: ");
    uart0_puts(cmd);
    uart0_nl();
}

static void changeState(State next, const char *label)
{
    uart0_puts(">>> STATE: ");
    uart0_puts(label);
    uart0_nl();
    state = next;
    stateEnterMs = millis();
    presentCnt = 0; absentCnt = 0; theftCnt = 0;
}

static void setup(void)
{
    millis_init();
    uart0_init();
    uart1_init();
    i2c_init();
    sei();
    delay_ms(1000);

    uart0_nl();
    uart0_puts("=================================="); uart0_nl();
    uart0_puts(" Delivery Detector - Mega (AVR)");    uart0_nl();
    uart0_puts("=================================="); uart0_nl();

    uart0_puts("VL53L0X... ");
    if (!vl53l0x_init()) {
        uart0_puts("FAIL"); uart0_nl();
        while (1) { delay_ms(1000); }
    }
    vl53l0x_set_timeout(500);
    uart0_puts("OK"); uart0_nl();

    uart0_puts("HX711... ");
    hx711_init();
    while (!hx711_is_ready()) { delay_ms(50); }
    hx711_tare(20);
    uart0_puts("OK (tared)"); uart0_nl();

    uart0_puts("System ready."); uart0_nl();
    changeState(ST_IDLE, "IDLE");
}

static void loop(void)
{
    long     weight = readWeight();
    int      dist   = readDistanceMM();
    uint32_t now    = millis();

    if (now - lastLogMs > LOG_MS) {
        lastLogMs = now;
        uart0_putc('[');
        switch (state) {
            case ST_IDLE:       uart0_puts("IDLE      "); break;
            case ST_ARRIVED:    uart0_puts("ARRIVED   "); break;
            case ST_MONITORING: uart0_puts("MONITORING"); break;
            case ST_ALERT:      uart0_puts("ALERT     "); break;
            case ST_COOLDOWN:   uart0_puts("COOLDOWN  "); break;
        }
        uart0_puts("] w="); uart0_put_long(weight);
        uart0_puts(" d=");  uart0_put_long(dist); uart0_puts("mm");
        if (state == ST_MONITORING) {
            uart0_puts("  (base w="); uart0_put_long(baselineWeight);
            uart0_puts(" d=");        uart0_put_long(baselineDist); uart0_putc(')');
        }
        uart0_nl();
    }

    switch (state) {
        case ST_IDLE:
            if (weight > WEIGHT_PRESENT_RAW && !distInvalid(dist) && dist < DISTANCE_PRESENT_MAX) {
                if (presentCnt < 255) presentCnt++;
            } else if (distInvalid(dist)) {
                /* 측정 실패는 카운터 유지 */
            } else {
                if (presentCnt > 0) presentCnt--;   /* 글리치는 즉시 리셋 안 함 */
            }
            if (presentCnt >= CONFIRM_N) {
                changeState(ST_ARRIVED, "ARRIVED - waiting baseline");
            }
            break;

        case ST_ARRIVED:
            if (weight < WEIGHT_PRESENT_RAW / 2 ||
                (!distInvalid(dist) && dist > DISTANCE_PRESENT_MAX)) {
                if (absentCnt < 255) absentCnt++;
            } else {
                if (absentCnt > 0) absentCnt--;
            }
            if (absentCnt >= CONFIRM_N) {
                changeState(ST_IDLE, "IDLE (false trigger)");
                break;
            }
            if (now - stateEnterMs >= BASELINE_MS) {
                baselineWeight = weight;
                baselineDist   = dist;
                uart0_puts("Baseline: w="); uart0_put_long(baselineWeight);
                uart0_puts(" d=");           uart0_put_long(baselineDist); uart0_nl();
                sendCommand("ALERT_ARRIVE");
                changeState(ST_MONITORING, "MONITORING");
            }
            break;

        case ST_MONITORING: {
            long weightDrop = baselineWeight - weight;
            if (weightDrop > WEIGHT_THEFT_DELTA &&
                !distInvalid(dist) && dist > DISTANCE_THEFT_MIN) {
                if (theftCnt < 255) theftCnt++;
            } else if (distInvalid(dist)) {
                /* 측정 실패 무시 */
            } else {
                if (theftCnt > 0) theftCnt--;
            }
            if (theftCnt >= CONFIRM_N) {
                changeState(ST_ALERT, "ALERT - theft detected");
            }
            break;
        }

        case ST_ALERT:
            sendCommand("ALERT_THEFT");
            changeState(ST_COOLDOWN, "COOLDOWN");
            break;

        case ST_COOLDOWN:
            if (now - stateEnterMs >= COOLDOWN_MS) {
                changeState(ST_IDLE, "IDLE (after cooldown)");
            }
            break;
    }

    delay_ms(50);
}

int main(void)
{
    setup();
    for (;;) loop();
    return 0;
}