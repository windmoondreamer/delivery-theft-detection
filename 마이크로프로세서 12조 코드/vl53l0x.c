/*
 * vl53l0x.c - Pololu vl53l0x-arduino 라이브러리의 C 포팅
 * 원본 로직(레지스터 시퀀스/타이밍 계산)은 그대로 유지하고
 * Wire(I2C) 호출만 i2c.c 의 TWI 함수로 교체.
 * setSignalRateLimit(0.25)는 float 회피를 위해 레지스터 값(32) 직접 기록.
 */
#include "config.h"
#include <avr/io.h>
#include "i2c.h"
#include "millis.h"
#include "vl53l0x.h"

/* 7bit 주소 0x29 -> SLA+W / SLA+R */
#define VL_ADDR    0x29
#define VL_SLA_W   ((VL_ADDR << 1) | 0)
#define VL_SLA_R   ((VL_ADDR << 1) | 1)

/* ---- 레지스터 주소 (원본 regAddr enum) ---- */
#define SYSRANGE_START                              0x00
#define SYSTEM_SEQUENCE_CONFIG                      0x01
#define SYSTEM_INTERMEASUREMENT_PERIOD              0x04
#define SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A
#define SYSTEM_INTERRUPT_CLEAR                      0x0B
#define RESULT_INTERRUPT_STATUS                     0x13
#define RESULT_RANGE_STATUS                         0x14
#define I2C_SLAVE_DEVICE_ADDRESS                    0x8A
#define MSRC_CONFIG_CONTROL                         0x60
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW            0x56
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH           0x57
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define PRE_RANGE_CONFIG_VCSEL_PERIOD               0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x51
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD             0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        0x71
#define MSRC_CONFIG_TIMEOUT_MACROP                  0x46
#define IDENTIFICATION_MODEL_ID                     0xC0
#define OSC_CALIBRATE_VAL                           0xF8
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0
#define GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4E
#define DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4F
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           0x89
#define GPIO_HV_MUX_ACTIVE_HIGH                     0x84

#define VcselPeriodPreRange   0
#define VcselPeriodFinalRange 1

/* calcMacroPeriod: PLL=1655ps, macro_vclks=2304 */
#define calcMacroPeriod(vcsel) ((((uint32_t)2304 * (vcsel) * 1655) + 500) / 1000)
#define decodeVcselPeriod(rv)  (((rv) + 1) << 1)

/* ---- 모듈 상태 ---- */
static uint8_t  stop_variable;
static uint32_t measurement_timing_budget_us;
static uint16_t io_timeout = 0;
static uint8_t  did_timeout = 0;
static uint16_t timeout_start_ms;

typedef struct { uint8_t tcc, msrc, dss, pre_range, final_range; } SeqEnables;
typedef struct {
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
} SeqTimeouts;

#define startTimeout()        (timeout_start_ms = (uint16_t)millis())
#define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)((uint16_t)millis() - timeout_start_ms) > io_timeout))

/* ---- I2C 레지스터 접근 ---- */
static void writeReg(uint8_t reg, uint8_t value)
{
    i2c_start(VL_SLA_W);
    i2c_write(reg);
    i2c_write(value);
    i2c_stop();
}
static void writeReg16(uint8_t reg, uint16_t value)
{
    i2c_start(VL_SLA_W);
    i2c_write(reg);
    i2c_write((uint8_t)(value >> 8));
    i2c_write((uint8_t)(value));
    i2c_stop();
}
static uint8_t readReg(uint8_t reg)
{
    uint8_t v;
    i2c_start(VL_SLA_W);
    i2c_write(reg);
    i2c_start(VL_SLA_R);   /* 반복 START */
    v = i2c_read_nack();
    i2c_stop();
    return v;
}
static uint16_t readReg16(uint8_t reg)
{
    uint16_t v;
    i2c_start(VL_SLA_W);
    i2c_write(reg);
    i2c_start(VL_SLA_R);
    v  = (uint16_t)i2c_read_ack() << 8;
    v |=           i2c_read_nack();
    i2c_stop();
    return v;
}
static void writeMulti(uint8_t reg, const uint8_t *src, uint8_t count)
{
    i2c_start(VL_SLA_W);
    i2c_write(reg);
    while (count--) i2c_write(*src++);
    i2c_stop();
}
static void readMulti(uint8_t reg, uint8_t *dst, uint8_t count)
{
    i2c_start(VL_SLA_W);
    i2c_write(reg);
    i2c_start(VL_SLA_R);
    while (count > 1) { *dst++ = i2c_read_ack(); count--; }
    *dst = i2c_read_nack();
    i2c_stop();
}

/* ---- 타이밍 계산 (원본 그대로) ---- */
static uint16_t decodeTimeout(uint16_t reg_val)
{
    return (uint16_t)((reg_val & 0x00FF) <<
           (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}
static uint16_t encodeTimeout(uint32_t timeout_mclks)
{
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;
    if (timeout_mclks > 0) {
        ls_byte = timeout_mclks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0) { ls_byte >>= 1; ms_byte++; }
        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    return 0;
}
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

static uint8_t getVcselPulsePeriod(uint8_t type)
{
    if (type == VcselPeriodPreRange)
        return decodeVcselPeriod(readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
    else if (type == VcselPeriodFinalRange)
        return decodeVcselPeriod(readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    return 255;
}

static void getSequenceStepEnables(SeqEnables *e)
{
    uint8_t c = readReg(SYSTEM_SEQUENCE_CONFIG);
    e->tcc         = (c >> 4) & 0x1;
    e->dss         = (c >> 3) & 0x1;
    e->msrc        = (c >> 2) & 0x1;
    e->pre_range   = (c >> 6) & 0x1;
    e->final_range = (c >> 7) & 0x1;
}

static void getSequenceStepTimeouts(const SeqEnables *e, SeqTimeouts *t)
{
    t->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

    t->msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    t->msrc_dss_tcc_us = timeoutMclksToMicroseconds(t->msrc_dss_tcc_mclks,
                                                    t->pre_range_vcsel_period_pclks);

    t->pre_range_mclks = decodeTimeout(readReg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    t->pre_range_us = timeoutMclksToMicroseconds(t->pre_range_mclks,
                                                 t->pre_range_vcsel_period_pclks);

    t->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);
    t->final_range_mclks = decodeTimeout(readReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    if (e->pre_range) t->final_range_mclks -= t->pre_range_mclks;
    t->final_range_us = timeoutMclksToMicroseconds(t->final_range_mclks,
                                                   t->final_range_vcsel_period_pclks);
}

static uint32_t getMeasurementTimingBudget(void)
{
    SeqEnables  enables;
    SeqTimeouts timeouts;
    const uint16_t StartOverhead = 1910, EndOverhead = 960, MsrcOverhead = 660;
    const uint16_t TccOverhead = 590, DssOverhead = 690, PreRangeOverhead = 660;
    const uint16_t FinalRangeOverhead = 550;

    uint32_t budget_us = StartOverhead + EndOverhead;
    getSequenceStepEnables(&enables);
    getSequenceStepTimeouts(&enables, &timeouts);

    if (enables.tcc)  budget_us += timeouts.msrc_dss_tcc_us + TccOverhead;
    if (enables.dss)  budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    else if (enables.msrc) budget_us += timeouts.msrc_dss_tcc_us + MsrcOverhead;
    if (enables.pre_range)   budget_us += timeouts.pre_range_us + PreRangeOverhead;
    if (enables.final_range) budget_us += timeouts.final_range_us + FinalRangeOverhead;

    measurement_timing_budget_us = budget_us;
    return budget_us;
}

static uint8_t setMeasurementTimingBudget(uint32_t budget_us)
{
    SeqEnables  enables;
    SeqTimeouts timeouts;
    const uint16_t StartOverhead = 1910, EndOverhead = 960, MsrcOverhead = 660;
    const uint16_t TccOverhead = 590, DssOverhead = 690, PreRangeOverhead = 660;
    const uint16_t FinalRangeOverhead = 550;

    uint32_t used_budget_us = StartOverhead + EndOverhead;
    getSequenceStepEnables(&enables);
    getSequenceStepTimeouts(&enables, &timeouts);

    if (enables.tcc)  used_budget_us += timeouts.msrc_dss_tcc_us + TccOverhead;
    if (enables.dss)  used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    else if (enables.msrc) used_budget_us += timeouts.msrc_dss_tcc_us + MsrcOverhead;
    if (enables.pre_range) used_budget_us += timeouts.pre_range_us + PreRangeOverhead;

    if (enables.final_range) {
        used_budget_us += FinalRangeOverhead;
        if (used_budget_us > budget_us) return 0; /* 요청 타임아웃 과다 */

        uint32_t final_range_timeout_us = budget_us - used_budget_us;
        uint32_t final_range_timeout_mclks =
            timeoutMicrosecondsToMclks(final_range_timeout_us,
                                       timeouts.final_range_vcsel_period_pclks);
        if (enables.pre_range) final_range_timeout_mclks += timeouts.pre_range_mclks;

        writeReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                   encodeTimeout(final_range_timeout_mclks));
        measurement_timing_budget_us = budget_us;
    }
    return 1;
}

static uint8_t getSpadInfo(uint8_t *count, uint8_t *type_is_aperture)
{
    uint8_t tmp;
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    writeReg(0xFF, 0x06);
    writeReg(0x83, readReg(0x83) | 0x04);
    writeReg(0xFF, 0x07);
    writeReg(0x81, 0x01);
    writeReg(0x80, 0x01);
    writeReg(0x94, 0x6b);
    writeReg(0x83, 0x00);

    startTimeout();
    while (readReg(0x83) == 0x00) { if (checkTimeoutExpired()) return 0; }

    writeReg(0x83, 0x01);
    tmp = readReg(0x92);
    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    writeReg(0x81, 0x00);
    writeReg(0xFF, 0x06);
    writeReg(0x83, readReg(0x83) & ~0x04);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);
    return 1;
}

static uint8_t performSingleRefCalibration(uint8_t vhv_init_byte)
{
    writeReg(SYSRANGE_START, 0x01 | vhv_init_byte);
    startTimeout();
    while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (checkTimeoutExpired()) return 0;
    }
    writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
    writeReg(SYSRANGE_START, 0x00);
    return 1;
}

/* ---- 공개 API ---- */
void vl53l0x_set_timeout(uint16_t ms) { io_timeout = ms; }

uint8_t vl53l0x_timeout_occurred(void)
{
    uint8_t t = did_timeout;
    did_timeout = 0;
    return t;
}

uint8_t vl53l0x_init(void)
{
    /* 모델 ID 확인 */
    if (readReg(IDENTIFICATION_MODEL_ID) != 0xEE) return 0;

    /* --- DataInit --- 2V8 모드 */
    writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
             readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
    writeReg(0x88, 0x00);
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    stop_variable = readReg(0x91);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    writeReg(MSRC_CONFIG_CONTROL, readReg(MSRC_CONFIG_CONTROL) | 0x12);
    /* setSignalRateLimit(0.25 MCPS): Q9.7 -> 0.25*128 = 32 */
    writeReg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32);
    writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

    /* --- StaticInit --- SPAD */
    uint8_t spad_count, spad_type_is_aperture;
    if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) return 0;

    uint8_t ref_spad_map[6];
    readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    writeReg(0xFF, 0x01);
    writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    writeReg(0xFF, 0x00);
    writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
            spads_enabled++;
        }
    }
    writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    /* --- 기본 튜닝 세팅 (DefaultTuningSettings) --- */
    writeReg(0xFF, 0x01); writeReg(0x00, 0x00);
    writeReg(0xFF, 0x00); writeReg(0x09, 0x00); writeReg(0x10, 0x00); writeReg(0x11, 0x00);
    writeReg(0x24, 0x01); writeReg(0x25, 0xFF); writeReg(0x75, 0x00);
    writeReg(0xFF, 0x01); writeReg(0x4E, 0x2C); writeReg(0x48, 0x00); writeReg(0x30, 0x20);
    writeReg(0xFF, 0x00); writeReg(0x30, 0x09); writeReg(0x54, 0x00); writeReg(0x31, 0x04);
    writeReg(0x32, 0x03); writeReg(0x40, 0x83); writeReg(0x46, 0x25); writeReg(0x60, 0x00);
    writeReg(0x27, 0x00); writeReg(0x50, 0x06); writeReg(0x51, 0x00); writeReg(0x52, 0x96);
    writeReg(0x56, 0x08); writeReg(0x57, 0x30); writeReg(0x61, 0x00); writeReg(0x62, 0x00);
    writeReg(0x64, 0x00); writeReg(0x65, 0x00); writeReg(0x66, 0xA0);
    writeReg(0xFF, 0x01); writeReg(0x22, 0x32); writeReg(0x47, 0x14); writeReg(0x49, 0xFF);
    writeReg(0x4A, 0x00);
    writeReg(0xFF, 0x00); writeReg(0x7A, 0x0A); writeReg(0x7B, 0x00); writeReg(0x78, 0x21);
    writeReg(0xFF, 0x01); writeReg(0x23, 0x34); writeReg(0x42, 0x00); writeReg(0x44, 0xFF);
    writeReg(0x45, 0x26); writeReg(0x46, 0x05); writeReg(0x40, 0x40); writeReg(0x0E, 0x06);
    writeReg(0x20, 0x1A); writeReg(0x43, 0x40);
    writeReg(0xFF, 0x00); writeReg(0x34, 0x03); writeReg(0x35, 0x44);
    writeReg(0xFF, 0x01); writeReg(0x31, 0x04); writeReg(0x4B, 0x09); writeReg(0x4C, 0x05);
    writeReg(0x4D, 0x04);
    writeReg(0xFF, 0x00); writeReg(0x44, 0x00); writeReg(0x45, 0x20); writeReg(0x47, 0x08);
    writeReg(0x48, 0x28); writeReg(0x67, 0x00); writeReg(0x70, 0x04); writeReg(0x71, 0x01);
    writeReg(0x72, 0xFE); writeReg(0x76, 0x00); writeReg(0x77, 0x00);
    writeReg(0xFF, 0x01); writeReg(0x0D, 0x01);
    writeReg(0xFF, 0x00); writeReg(0x80, 0x01); writeReg(0x01, 0xF8);
    writeReg(0xFF, 0x01); writeReg(0x8E, 0x01); writeReg(0x00, 0x01); writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    /* 인터럽트: new sample ready, active low */
    writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    writeReg(GPIO_HV_MUX_ACTIVE_HIGH, readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
    writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    measurement_timing_budget_us = getMeasurementTimingBudget();

    /* MSRC, TCC 비활성 */
    writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);
    setMeasurementTimingBudget(measurement_timing_budget_us);

    /* --- RefCalibration --- */
    writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!performSingleRefCalibration(0x40)) return 0;
    writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!performSingleRefCalibration(0x00)) return 0;
    writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    return 1;
}

static uint16_t readRangeContinuous(void)
{
    startTimeout();
    while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (checkTimeoutExpired()) { did_timeout = 1; return 65535; }
    }
    uint16_t range = readReg16(RESULT_RANGE_STATUS + 10);
    writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
    return range;
}

uint16_t vl53l0x_read_range_single(void)
{
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    writeReg(0x91, stop_variable);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    writeReg(SYSRANGE_START, 0x01);

    startTimeout();
    while (readReg(SYSRANGE_START) & 0x01) {
        if (checkTimeoutExpired()) { did_timeout = 1; return 65535; }
    }
    return readRangeContinuous();
}
