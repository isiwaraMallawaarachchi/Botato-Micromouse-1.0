#include "tof.h"

/* ---- Register map ------------------------------------------------------
 * Verified against the complete published VL53L0X register set. Note:
 * I2C_SLAVE_DEVICE_ADDRESS is correctly 0x8A here — some published
 * community ports mistakenly define this as 0x52 (the DEFAULT device
 * address) and then reuse that same wrong value as the register for
 * changing the address, which breaks multi-device address assignment
 * entirely. Fixed here.                                                */

#define REG_SYSRANGE_START                              0x00
#define REG_SYSTEM_SEQUENCE_CONFIG                       0x01
#define REG_SYSTEM_INTERMEASUREMENT_PERIOD               0x04
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO                 0x0A
#define REG_SYSTEM_INTERRUPT_CLEAR                       0x0B
#define REG_RESULT_INTERRUPT_STATUS                      0x13
#define REG_RESULT_RANGE_STATUS                          0x14
#define REG_I2C_SLAVE_DEVICE_ADDRESS                     0x8A
#define REG_MSRC_CONFIG_CONTROL                          0x60
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW             0x56
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH            0x57
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW           0x47
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH          0x48
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT  0x44
#define REG_PRE_RANGE_CONFIG_VCSEL_PERIOD                0x50
#define REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI           0x51
#define REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD              0x70
#define REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI         0x71
#define REG_MSRC_CONFIG_TIMEOUT_MACROP                   0x46
#define REG_IDENTIFICATION_MODEL_ID                      0xC0
#define REG_GPIO_HV_MUX_ACTIVE_HIGH                      0x84
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0             0xB0
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT            0xB6
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD          0x4E
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET             0x4F
#define REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV            0x89

#define IO_TIMEOUT_MS 100

/* ---- Low-level register access ----------------------------------------- */

static HAL_StatusTypeDef WriteReg8(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT,
                              &val, 1, IO_TIMEOUT_MS);
}

static HAL_StatusTypeDef ReadReg8(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t *out)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT,
                             out, 1, IO_TIMEOUT_MS);
}

static HAL_StatusTypeDef WriteReg16(VL53L0X_Dev_t *dev, uint8_t reg, uint16_t val)
{
    uint8_t buf[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT,
                              buf, 2, IO_TIMEOUT_MS);
}

static HAL_StatusTypeDef ReadReg16(VL53L0X_Dev_t *dev, uint8_t reg, uint16_t *out)
{
    uint8_t buf[2];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg,
                                             I2C_MEMADD_SIZE_8BIT, buf, 2, IO_TIMEOUT_MS);
    if (st == HAL_OK) *out = (uint16_t)((buf[0] << 8) | buf[1]);
    return st;
}

static HAL_StatusTypeDef WriteMulti(VL53L0X_Dev_t *dev, uint8_t reg, const uint8_t *src, uint8_t count)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT,
                              (uint8_t *)src, count, IO_TIMEOUT_MS);
}

static HAL_StatusTypeDef ReadMulti(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t *dst, uint8_t count)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT,
                             dst, count, IO_TIMEOUT_MS);
}

/* ---- Timeout math (verbatim from the verified reference) --------------- */

static uint16_t DecodeVcselPeriod(uint8_t reg_val) { return (uint16_t)((reg_val + 1) << 1); }

static uint32_t CalcMacroPeriodNs(uint8_t vcsel_period_pclks)
{
    return (((uint32_t)2304 * vcsel_period_pclks * 1655) + 500) / 1000;
}

static uint16_t DecodeTimeout(uint16_t reg_val)
{
    return (uint16_t)(((reg_val & 0x00FF) << ((reg_val & 0xFF00) >> 8)) + 1);
}

static uint16_t EncodeTimeout(uint16_t timeout_mclks)
{
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;
    if (timeout_mclks > 0)
    {
        ls_byte = timeout_mclks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }
        return (uint16_t)((ms_byte << 8) | (ls_byte & 0xFF));
    }
    return 0;
}

static uint32_t TimeoutMclksToUs(uint16_t timeout_mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = CalcMacroPeriodNs(vcsel_period_pclks);
    return ((timeout_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

static uint32_t TimeoutUsToMclks(uint32_t timeout_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = CalcMacroPeriodNs(vcsel_period_pclks);
    return (((timeout_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

/* ---- Sequence step enable / timeout readback ---------------------------- */

typedef struct {
    bool tcc, msrc, dss, pre_range, final_range;
} SeqEnables;

typedef struct {
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} SeqTimeouts;

static void GetSequenceStepEnables(VL53L0X_Dev_t *dev, SeqEnables *e)
{
    uint8_t cfg = 0;
    ReadReg8(dev, REG_SYSTEM_SEQUENCE_CONFIG, &cfg);
    e->tcc         = (cfg >> 4) & 0x1;
    e->dss         = (cfg >> 3) & 0x1;
    e->msrc        = (cfg >> 2) & 0x1;
    e->pre_range   = (cfg >> 6) & 0x1;
    e->final_range = (cfg >> 7) & 0x1;
}

static uint8_t GetVcselPulsePeriod(VL53L0X_Dev_t *dev, bool final_range)
{
    uint8_t raw = 0;
    ReadReg8(dev, final_range ? REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD
                               : REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, &raw);
    return (uint8_t)DecodeVcselPeriod(raw);
}

static void GetSequenceStepTimeouts(VL53L0X_Dev_t *dev, const SeqEnables *e, SeqTimeouts *t)
{
    uint8_t  msrc_raw = 0;
    uint16_t pre_raw = 0, final_raw = 0;

    t->pre_range_vcsel_period_pclks = GetVcselPulsePeriod(dev, false);

    ReadReg8(dev, REG_MSRC_CONFIG_TIMEOUT_MACROP, &msrc_raw);
    t->msrc_dss_tcc_mclks = (uint16_t)(msrc_raw + 1);
    t->msrc_dss_tcc_us = TimeoutMclksToUs(t->msrc_dss_tcc_mclks, t->pre_range_vcsel_period_pclks);

    ReadReg16(dev, REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &pre_raw);
    t->pre_range_mclks = DecodeTimeout(pre_raw);
    t->pre_range_us = TimeoutMclksToUs(t->pre_range_mclks, t->pre_range_vcsel_period_pclks);

    t->final_range_vcsel_period_pclks = GetVcselPulsePeriod(dev, true);

    ReadReg16(dev, REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &final_raw);
    t->final_range_mclks = DecodeTimeout(final_raw);
    if (e->pre_range)
    {
        t->final_range_mclks -= t->pre_range_mclks;
    }
    t->final_range_us = TimeoutMclksToUs(t->final_range_mclks, t->final_range_vcsel_period_pclks);
}

static uint32_t GetMeasurementTimingBudget(VL53L0X_Dev_t *dev)
{
    SeqEnables  e;
    SeqTimeouts t;
    const uint16_t StartOverhead = 1910, EndOverhead = 960, MsrcOverhead = 660,
                   TccOverhead = 590, DssOverhead = 690, PreRangeOverhead = 660,
                   FinalRangeOverhead = 550;

    GetSequenceStepEnables(dev, &e);
    GetSequenceStepTimeouts(dev, &e, &t);

    uint32_t budget_us = StartOverhead + EndOverhead;

    if (e.tcc)         budget_us += t.msrc_dss_tcc_us + TccOverhead;
    if (e.dss)         budget_us += 2 * (t.msrc_dss_tcc_us + DssOverhead);
    else if (e.msrc)   budget_us += t.msrc_dss_tcc_us + MsrcOverhead;
    if (e.pre_range)   budget_us += t.pre_range_us + PreRangeOverhead;
    if (e.final_range) budget_us += t.final_range_us + FinalRangeOverhead;

    dev->measurement_timing_budget_us = budget_us;
    return budget_us;
}

static bool SetMeasurementTimingBudget(VL53L0X_Dev_t *dev, uint32_t budget_us)
{
    SeqEnables  e;
    SeqTimeouts t;
    const uint16_t StartOverhead = 1320, EndOverhead = 960, MsrcOverhead = 660,
                   TccOverhead = 590, DssOverhead = 690, PreRangeOverhead = 660,
                   FinalRangeOverhead = 550;
    const uint32_t MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget) return false;

    GetSequenceStepEnables(dev, &e);
    GetSequenceStepTimeouts(dev, &e, &t);

    uint32_t used_us = StartOverhead + EndOverhead;

    if (e.tcc)       used_us += t.msrc_dss_tcc_us + TccOverhead;
    if (e.dss)       used_us += 2 * (t.msrc_dss_tcc_us + DssOverhead);
    else if (e.msrc) used_us += t.msrc_dss_tcc_us + MsrcOverhead;
    if (e.pre_range) used_us += t.pre_range_us + PreRangeOverhead;

    if (e.final_range)
    {
        used_us += FinalRangeOverhead;
        if (used_us > budget_us) return false;

        uint32_t final_us = budget_us - used_us;
        uint32_t final_mclks = TimeoutUsToMclks((uint32_t)final_us, (uint8_t)t.final_range_vcsel_period_pclks);
        if (e.pre_range) final_mclks += t.pre_range_mclks;

        WriteReg16(dev, REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, EncodeTimeout((uint16_t)final_mclks));
        dev->measurement_timing_budget_us = budget_us;
    }
    return true;
}

/* ---- SPAD info + reference SPAD setup (verbatim algorithm) -------------- */

static bool GetSpadInfo(VL53L0X_Dev_t *dev, uint8_t *count, bool *type_is_aperture)
{
    uint8_t tmp = 0;

    WriteReg8(dev, 0x80, 0x01);
    WriteReg8(dev, 0xFF, 0x01);
    WriteReg8(dev, 0x00, 0x00);
    WriteReg8(dev, 0xFF, 0x06);
    ReadReg8(dev, 0x83, &tmp);
    WriteReg8(dev, 0x83, (uint8_t)(tmp | 0x04));
    WriteReg8(dev, 0xFF, 0x07);
    WriteReg8(dev, 0x81, 0x01);
    WriteReg8(dev, 0x80, 0x01);
    WriteReg8(dev, 0x94, 0x6B);
    WriteReg8(dev, 0x83, 0x00);
    WriteReg8(dev, 0x83, 0x01);

    ReadReg8(dev, 0x92, &tmp);
    *count = tmp & 0x7F;
    *type_is_aperture = (bool)((tmp >> 7) & 0x01);

    WriteReg8(dev, 0x81, 0x00);
    WriteReg8(dev, 0xFF, 0x06);
    ReadReg8(dev, 0x83, &tmp);
    WriteReg8(dev, 0x83, (uint8_t)(tmp & ~0x04));
    WriteReg8(dev, 0xFF, 0x01);
    WriteReg8(dev, 0x00, 0x01);
    WriteReg8(dev, 0xFF, 0x00);
    WriteReg8(dev, 0x80, 0x00);

    return true;
}

/* ---- Reference calibration (VHV + phase) --------------------------------
 * Short, standard sequence: trigger with the VHV init byte set, wait for
 * the interrupt status bit, clear it, then stop.                        */
static bool PerformSingleRefCalibration(VL53L0X_Dev_t *dev, uint8_t vhv_init_byte)
{
    uint8_t status_reg = 0;
    uint32_t t0 = HAL_GetTick();

    WriteReg8(dev, REG_SYSRANGE_START, (uint8_t)(0x01 | vhv_init_byte));

    do
    {
        ReadReg8(dev, REG_RESULT_INTERRUPT_STATUS, &status_reg);
        if ((HAL_GetTick() - t0) > IO_TIMEOUT_MS) return false;
    } while ((status_reg & 0x07) == 0);

    WriteReg8(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    WriteReg8(dev, REG_SYSRANGE_START, 0x00);

    return true;
}

/* ---- Public API ---------------------------------------------------------- */

HAL_StatusTypeDef VL53L0X_SetAddress(VL53L0X_Dev_t *dev, uint8_t new_addr_8bit)
{
    HAL_StatusTypeDef st = WriteReg8(dev, REG_I2C_SLAVE_DEVICE_ADDRESS,
                                      (uint8_t)((new_addr_8bit >> 1) & 0x7F));
    if (st == HAL_OK)
    {
        dev->address = new_addr_8bit;
    }
    return st;
}

HAL_StatusTypeDef VL53L0X_Init(VL53L0X_Dev_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr_8bit)
{
    dev->hi2c    = hi2c;
    dev->address = addr_8bit;

    /* ---- DataInit ---- */

    /* 2.8V I/O mode — most breakout boards need this bit set. */
    uint8_t vhv = 0;
    ReadReg8(dev, REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, &vhv);
    WriteReg8(dev, REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, (uint8_t)(vhv | 0x01));

    WriteReg8(dev, 0x88, 0x00);
    WriteReg8(dev, 0x80, 0x01);
    WriteReg8(dev, 0xFF, 0x01);
    WriteReg8(dev, 0x00, 0x00);
    ReadReg8(dev, 0x91, &dev->stop_variable);
    WriteReg8(dev, 0x00, 0x01);
    WriteReg8(dev, 0xFF, 0x00);
    WriteReg8(dev, 0x80, 0x00);

    /* Disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks. */
    uint8_t msrc_ctrl = 0;
    ReadReg8(dev, REG_MSRC_CONFIG_CONTROL, &msrc_ctrl);
    WriteReg8(dev, REG_MSRC_CONFIG_CONTROL, (uint8_t)(msrc_ctrl | 0x12));

    /* Final range signal rate limit: 0.25 Mcps, Q9.7 fixed point. */
    WriteReg16(dev, REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint16_t)(0.25f * (1 << 7)));

    WriteReg8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);

    /* ---- StaticInit: SPAD configuration ---- */

    uint8_t spad_count = 0;
    bool    spad_is_aperture = false;
    if (!GetSpadInfo(dev, &spad_count, &spad_is_aperture)) return HAL_ERROR;

    uint8_t ref_spad_map[6];
    ReadMulti(dev, REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    WriteReg8(dev, 0xFF, 0x01);
    WriteReg8(dev, REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    WriteReg8(dev, REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    WriteReg8(dev, 0xFF, 0x00);
    WriteReg8(dev, REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad = spad_is_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;
    for (uint8_t i = 0; i < 48; i++)
    {
        if (i < first_spad || spads_enabled == spad_count)
        {
            ref_spad_map[i / 8] &= (uint8_t)~(1 << (i % 8));
        }
        else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
        {
            spads_enabled++;
        }
    }
    WriteMulti(dev, REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    /* ---- StaticInit: default tuning settings ----
     * Verbatim register sequence from the verified reference implementation.
     * Each of these addresses specific analog/timing configuration inside
     * the sensor — do not reorder or omit entries.                         */
    WriteReg8(dev, 0xFF, 0x01); WriteReg8(dev, 0x00, 0x00); WriteReg8(dev, 0xFF, 0x00);
    WriteReg8(dev, 0x09, 0x00); WriteReg8(dev, 0x10, 0x00); WriteReg8(dev, 0x11, 0x00);
    WriteReg8(dev, 0x24, 0x01); WriteReg8(dev, 0x25, 0xFF); WriteReg8(dev, 0x75, 0x00);
    WriteReg8(dev, 0xFF, 0x01); WriteReg8(dev, 0x4E, 0x2C); WriteReg8(dev, 0x48, 0x00);
    WriteReg8(dev, 0x30, 0x20); WriteReg8(dev, 0xFF, 0x00); WriteReg8(dev, 0x30, 0x09);
    WriteReg8(dev, 0x54, 0x00); WriteReg8(dev, 0x31, 0x04); WriteReg8(dev, 0x32, 0x03);
    WriteReg8(dev, 0x40, 0x83); WriteReg8(dev, 0x46, 0x25); WriteReg8(dev, 0x60, 0x00);
    WriteReg8(dev, 0x27, 0x00); WriteReg8(dev, 0x50, 0x06); WriteReg8(dev, 0x51, 0x00);
    WriteReg8(dev, 0x52, 0x96); WriteReg8(dev, 0x56, 0x08); WriteReg8(dev, 0x57, 0x30);
    WriteReg8(dev, 0x61, 0x00); WriteReg8(dev, 0x62, 0x00); WriteReg8(dev, 0x64, 0x00);
    WriteReg8(dev, 0x65, 0x00); WriteReg8(dev, 0x66, 0xA0); WriteReg8(dev, 0xFF, 0x01);
    WriteReg8(dev, 0x22, 0x32); WriteReg8(dev, 0x47, 0x14); WriteReg8(dev, 0x49, 0xFF);
    WriteReg8(dev, 0x4A, 0x00); WriteReg8(dev, 0xFF, 0x00); WriteReg8(dev, 0x7A, 0x0A);
    WriteReg8(dev, 0x7B, 0x00); WriteReg8(dev, 0x78, 0x21); WriteReg8(dev, 0xFF, 0x01);
    WriteReg8(dev, 0x23, 0x34); WriteReg8(dev, 0x42, 0x00); WriteReg8(dev, 0x44, 0xFF);
    WriteReg8(dev, 0x45, 0x26); WriteReg8(dev, 0x46, 0x05); WriteReg8(dev, 0x40, 0x40);
    WriteReg8(dev, 0x0E, 0x06); WriteReg8(dev, 0x20, 0x1A); WriteReg8(dev, 0x43, 0x40);
    WriteReg8(dev, 0xFF, 0x00); WriteReg8(dev, 0x34, 0x03); WriteReg8(dev, 0x35, 0x44);
    WriteReg8(dev, 0xFF, 0x01); WriteReg8(dev, 0x31, 0x04); WriteReg8(dev, 0x4B, 0x09);
    WriteReg8(dev, 0x4C, 0x05); WriteReg8(dev, 0x4D, 0x04); WriteReg8(dev, 0xFF, 0x00);
    WriteReg8(dev, 0x44, 0x00); WriteReg8(dev, 0x45, 0x20); WriteReg8(dev, 0x47, 0x08);
    WriteReg8(dev, 0x48, 0x28); WriteReg8(dev, 0x67, 0x00); WriteReg8(dev, 0x70, 0x04);
    WriteReg8(dev, 0x71, 0x01); WriteReg8(dev, 0x72, 0xFE); WriteReg8(dev, 0x76, 0x00);
    WriteReg8(dev, 0x77, 0x00); WriteReg8(dev, 0xFF, 0x01); WriteReg8(dev, 0x0D, 0x01);
    WriteReg8(dev, 0xFF, 0x00); WriteReg8(dev, 0x80, 0x01); WriteReg8(dev, 0x01, 0xF8);
    WriteReg8(dev, 0xFF, 0x01); WriteReg8(dev, 0x8E, 0x01); WriteReg8(dev, 0x00, 0x01);
    WriteReg8(dev, 0xFF, 0x00); WriteReg8(dev, 0x80, 0x00);

    /* Interrupt config: new sample ready, active low polarity. */
    WriteReg8(dev, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    uint8_t gpio_hv = 0;
    ReadReg8(dev, REG_GPIO_HV_MUX_ACTIVE_HIGH, &gpio_hv);
    WriteReg8(dev, REG_GPIO_HV_MUX_ACTIVE_HIGH, (uint8_t)(gpio_hv & ~0x10));
    WriteReg8(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    dev->measurement_timing_budget_us = GetMeasurementTimingBudget(dev);

    /* Disable MSRC and TCC by default, then re-apply the timing budget
     * so the final-range timeout accounts for the steps actually
     * enabled now.                                                       */
    WriteReg8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);
    SetMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

    /* ---- Reference calibration (VHV + phase) ---- */
    WriteReg8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!PerformSingleRefCalibration(dev, 0x40)) return HAL_ERROR;

    WriteReg8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!PerformSingleRefCalibration(dev, 0x00)) return HAL_ERROR;

    WriteReg8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    return HAL_OK;
}

HAL_StatusTypeDef VL53L0X_ReadRangeSingleMillimeters(VL53L0X_Dev_t *dev, uint16_t *out_mm)
{
    WriteReg8(dev, 0x80, 0x01);
    WriteReg8(dev, 0xFF, 0x01);
    WriteReg8(dev, 0x00, 0x00);
    WriteReg8(dev, 0x91, dev->stop_variable);
    WriteReg8(dev, 0x00, 0x01);
    WriteReg8(dev, 0xFF, 0x00);
    WriteReg8(dev, 0x80, 0x00);

    WriteReg8(dev, REG_SYSRANGE_START, 0x01);

    uint32_t t0 = HAL_GetTick();
    uint8_t start_reg = 0;
    do
    {
        ReadReg8(dev, REG_SYSRANGE_START, &start_reg);
        if ((HAL_GetTick() - t0) > IO_TIMEOUT_MS) return HAL_TIMEOUT;
    } while (start_reg & 0x01);

    t0 = HAL_GetTick();
    uint8_t status_reg = 0;
    do
    {
        ReadReg8(dev, REG_RESULT_INTERRUPT_STATUS, &status_reg);
        if ((HAL_GetTick() - t0) > IO_TIMEOUT_MS) return HAL_TIMEOUT;
    } while ((status_reg & 0x07) == 0);

    uint16_t range = 0;
    HAL_StatusTypeDef st = ReadReg16(dev, (uint8_t)(REG_RESULT_RANGE_STATUS + 10), &range);
    if (st != HAL_OK) return st;

    WriteReg8(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    *out_mm = range;
    return HAL_OK;
}
