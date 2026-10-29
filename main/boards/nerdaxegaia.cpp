#include <math.h>
#include "serial.h"
#include "board.h"
#include "nvs_config.h"
#include "nerdaxegaia.h"

#include "drivers/i2c_master.h"
#include "drivers/nerdaxe/DS4432U.h"
#include "drivers/nerdaxe/EMC2101.h"
#include "drivers/nerdaxe/INA260.h"
#include "drivers/nerdaxe/adc.h"
#include "drivers/nerdaxe/TPS546.h"
#include "drivers/nerdaxe/pmbus_commands.h"

#include "esp_log.h"
#include "esp_timer.h"

#define BM1373_RST_PIN GPIO_NUM_1
#define LDO_EN_PIN GPIO_NUM_12   // LDO enable (active-high) — new Gaia board
#define GAIA_POWER_OFFSET 5

bool tempinit_gaia = false;

static const char* TAG="nerdaxeGaia";

#define MAX(a,b) ((a)>(b)?(a):(b))

// Direct PMBus status reads used by the diagnostic build.  The normal TPS546
// driver keeps its SMBus helpers private; using the shared I2C wrapper here lets
// Gaia inspect the standard latched status registers without changing any TPS
// configuration or protection limits.
static uint8_t gaia_pmbus_read_byte(uint8_t command)
{
    uint8_t value = 0xff;
    if (i2c_master_register_read(TPS546_I2CADDR, command, &value, 1) != ESP_OK) {
        return 0xff;
    }
    return value;
}

static uint16_t gaia_pmbus_read_word(uint8_t command)
{
    uint8_t data[2] = {0xff, 0xff};
    if (i2c_master_register_read(TPS546_I2CADDR, command, data, sizeof(data)) != ESP_OK) {
        return 0xffff;
    }
    // PMBus word reads are LSB first.
    return (static_cast<uint16_t>(data[1]) << 8) | data[0];
}

// W5500 ethernet interposer pins (see eth-interposer FIRMWARE-GAIA.md):
// SCLK=GPIO10 (JP2 2-3), MOSI=GPIO3 (JP3 2-3), MISO=GPIO16, CS=GPIO21.
// RST=-1 (reset is an RC on the interposer, no GPIO) and IRQ=-1 (polling). This
// deliberately avoids the driver defaults (GPIO2/12/13/11) which on the Gaia are
// /VDD, LDO_EN, PMB_ALRT and PGOOD — driving any of those would break power-up.
static const EthPins kEthPins = { .sclk = GPIO_NUM_10, .mosi = GPIO_NUM_3, .miso = GPIO_NUM_16,
                                  .cs = GPIO_NUM_21, .rst = GPIO_NUM_NC, .irq = GPIO_NUM_NC };

const EthPins *NerdaxeGaia::getEthPins()
{
    return &kEthPins;
}

NerdaxeGaia::NerdaxeGaia() : NerdAxe() {
    m_deviceModel = "NerdAxeGaia";
    m_miningAgent = "NerdAxe";
    m_asicModel = "BM1373";
    m_version = 200;
    m_asicCount = 1;

    m_asicJobIntervalMs = 1500;
    m_asicFrequencies = {300, 320, 340, 350, 360, 380, 400, 420, 440};
    m_asicVoltages = {900, 920, 940, 960, 980, 1000, 1020, 1040};
    m_defaultAsicFrequency = m_asicFrequency = 350;
    m_defaultAsicVoltageMillis = m_asicVoltageMillis = 940;
    m_absMinAsicVoltageMillis = 800;    // hard floor for core voltage (mV)
    m_absMaxAsicVoltageMillis = 1300;   // hard ceiling for core voltage (mV)
    // m_absMaxAsicFrequency = 750;
    m_initVoltageMillis = 1000;
    m_fanInvertPolarity = false;
    m_fanPerc = 100;
    m_flipScreen = false;   // Gaia panel mounted rotated 180° vs NerdAxe → flip default
    m_vr_maxTemp = TPS546_THROTTLE_TEMP; //Set max voltage regulator temp

    m_pidSettings[0].targetTemp = 60;
    m_pidSettings[0].p =  600; // 6.00
    m_pidSettings[0].i =   10; // 0.1
    m_pidSettings[0].d = 1000; // 10.00

    m_maxPin = 40.0;   // web power-gauge reference ceiling (W) — display only, not a runtime cutoff
    m_minPin = 5.0;
    m_maxVin = 13.2;   // 12V input rail (+10%)
    m_minVin = 10.8;   // 12V input rail (-10%)
    m_minCurrentA = 0.0f;
    m_maxCurrentA = 6.0f;

    m_asicMaxDifficulty = 2048;
    m_asicMinDifficulty = 512;
    m_asicMinDifficultyDualPool = 256;

#ifdef NERDAXEGAIA
    m_theme = new ThemeNerdaxegaia();
#endif

    // Default web dashboard theme (Nebular) for this board = the Gaia theme.
    // Served to the browser via the API as `defaultTheme`; the web applies it
    // when there is no valid theme saved in localStorage. (This is the web
    // theme, separate from m_theme above, which is the on-device LCD theme.)
    m_defaultTheme = "gaia";

    m_swarmColorName = "#e7cf00"; // yellow

    m_asics = new BM1373();
    m_hasHashCounter = true;
    m_vrFrequency = m_defaultVrFrequency = m_asics->getDefaultVrFrequency();

    m_hasEth = isEthConnected();
}

void NerdaxeGaia::resetVrDiagnostics()
{
    m_vrDiagnostics = GaiaVrDiagnostics{};
}

bool NerdaxeGaia::initBoard()
{
    Board::initBoard();

    // Bring up the LDO FIRST. On the Gaia the TPS546 takes its logic supply
    // from the LDO rail, so it must be powered before ANY TPS546 access
    // (set_vin_config / init below) — otherwise the regulator answers with a
    // garbage device-ID, init fails, and initBoard bails out before the RESET
    // and LDO pins are even configured. Enable it and keep it on continuously
    // (this is exactly what the known-good "GPIO12 jumpered high" does).
    gpio_pad_select_gpio(LDO_EN_PIN);
    gpio_set_direction(LDO_EN_PIN, GPIO_MODE_OUTPUT);
    LDO_enable();
    vTaskDelay(pdMS_TO_TICKS(100));   // let the LDO rail settle before I2C

    ADC_init();
    SERIAL_init();

    // Init I2C
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C initializing failed");
        return false;
    }

    EMC2101_init(m_fanInvertPolarity);
    EMC2101_set_ideality_factor(EMC2101_IDEALITY_1_0319);
    EMC2101_set_beta_compensation(EMC2101_BETA_11);
    setFanSpeed(m_fanPerc);

    // 12V input rail: widen the TPS546 VIN thresholds so the input over-voltage
    // fault (5V default = 6V) doesn't trip at 12V. Board-aware — only the Gaia
    // calls this; the shared 5V defaults stay for the NerdAxeGamma. Must run
    // BEFORE TPS546_init() so the correct limits are programmed from the start.
    TPS546_set_vin_config(/*on*/ 10.5f, /*off*/ 9.5f, /*uv_warn*/ 10.5f, /*ov_fault*/ 14.0f);

    // Raise the output over-current protection: the shared driver defaults to
    // 25A warn / 30A fault, which trips around 440MHz @ 1V (~31A). The Gaia's
    // TPS546D24A is rated 40A, so use a conservative 28A warn / 33A fault.
    // Board-specific; must run BEFORE TPS546_init().
    TPS546_set_iout_config(/*warn*/ 28.0f, /*fault*/ 33.0f);

    //Init voltage controller
    if (TPS546_init() != ESP_OK) {
        ESP_LOGE(TAG, "TPS546 init failed!");
        return ESP_FAIL;
    }
    TPS546_set_frequency(400);
    setVoltage(0.0);

    gpio_pad_select_gpio(BM1373_RST_PIN);
    gpio_set_direction(BM1373_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BM1373_RST_PIN, 0);

    // (LDO is already enabled at the top of initBoard, before the TPS546.)

    return true;
}

void NerdaxeGaia::shutdown() {
    setVoltage(0.0);

    // let the core rail collapse before cutting the LDO
    vTaskDelay(pdMS_TO_TICKS(500));

    LDO_disable();

    vTaskDelay(pdMS_TO_TICKS(500));

    Board::shutdown();
}

void NerdaxeGaia::LDO_enable() {
    ESP_LOGI(TAG, "Enable LDO");
    gpio_set_level(LDO_EN_PIN, 1);
}

void NerdaxeGaia::LDO_disable() {
    ESP_LOGI(TAG, "Disable LDO");
    gpio_set_level(LDO_EN_PIN, 0);
}

bool NerdaxeGaia::initAsics() {

    resetVrDiagnostics();

    // Core buck off, but keep the LDO ON (already enabled in initBoard and
    // never cycled): the chip runs on the LDO alone during the self-test, so
    // cutting it here — as the old sequence did — left the chip unpowered and
    // it failed to enumerate. This matches the known-good jumpered-high case.
    setVoltage(0.0);
    LDO_enable();   // re-assert (no-op if already high); never disable

    // wait 500ms
    vTaskDelay(pdMS_TO_TICKS(500));

    // set reset low
    gpio_set_level(BM1373_RST_PIN, 0);

    // wait 250ms
    vTaskDelay(pdMS_TO_TICKS(250));

    // LDO already on and settled; give the reset a moment before the core ramp
    vTaskDelay(pdMS_TO_TICKS(100));

    // set the init voltage
    // use the higher voltage for initialization
    setVoltage((float) MAX(m_initVoltageMillis, m_asicVoltageMillis) / 1000.0f);

    // wait 500ms
    vTaskDelay(pdMS_TO_TICKS(500));

    m_isBuckInitialized = true;

    // release reset pin
    gpio_set_level(BM1373_RST_PIN, 1);

    // delay for 250ms
    vTaskDelay(pdMS_TO_TICKS(250));

    SERIAL_clear_buffer();
    m_chipsDetected = m_asics->init(m_asicFrequency, m_asicCount, m_asicMaxDifficulty, m_vrFrequency);
    if (!m_chipsDetected) {
        ESP_LOGE(TAG, "error initializing asics!");
        return false;
    }
    int maxBaud = m_asics->setMaxBaud();
    // no idea why a delay is needed here starting with esp-idf 5.4 🙈
    vTaskDelay(pdMS_TO_TICKS(500));
    SERIAL_set_baud(maxBaud);
    SERIAL_clear_buffer();

    vTaskDelay(pdMS_TO_TICKS(500));

    m_isInitialized = true;
    return true;
}

bool NerdaxeGaia::setVoltage(float core_voltage)
{
    if (!validateVoltage(core_voltage)) {
        return false;
    }

    ESP_LOGI(TAG, "Set ASIC voltage = %.3fV", core_voltage);
    return TPS546_set_vout(core_voltage);
}

float NerdaxeGaia::getTemperature(int index) {

    if (!m_isInitialized) {
        return EMC2101_get_internal_temp() + 5;
    }

    if (index > 0) {
        return 0.0f;
    }

    //Reading ASIC temp
    float asic_temp = EMC2101_get_external_temp();
    ESP_LOGI(TAG, "Read ASIC temp = %.3fºC", asic_temp);
    return asic_temp; //External board Temp
}

float NerdaxeGaia::getVRTemp() {
    //Reading voltage regulator temp
    float vr_temp = TPS546_get_temperature();
    ESP_LOGI(TAG, "Read vr temp = %.3fºC", vr_temp);
    if (isfinite(vr_temp)) {
        m_vrDiagnostics.lastVrTempC = vr_temp;
        m_vrDiagnostics.lastSampleUs = esp_timer_get_time();
    }
    return vr_temp; //- vr_temp (voltage regulator temp)
}

float NerdaxeGaia::getVin() {
    return TPS546_get_vin();
}

float NerdaxeGaia::getIin() {
    float vin = getVin();
    if (!vin) {
        return 0.0f;
    }

    return getPin() / vin;
}

float NerdaxeGaia::getPin() {
    const float vout = TPS546_get_vout();
    const float iout = getIout();
    return (vout * iout) + GAIA_POWER_OFFSET;
}

float NerdaxeGaia::getVout() {
    float vout = ADC_get_vcore() / 1000.0f;
    if (isfinite(vout)) {
        m_vrDiagnostics.lastVoutV = vout;
        if (vout > 0.05f) {
            m_vrDiagnostics.lastNonZeroVoutV = vout;
        }
        m_vrDiagnostics.lastSampleUs = esp_timer_get_time();
    }
    return vout;
}

float NerdaxeGaia::getIout() {
    float iout = TPS546_get_iout();
    if (isfinite(iout)) {
        m_vrDiagnostics.lastIoutA = iout;
        if (iout > 0.05f) {
            m_vrDiagnostics.lastNonZeroIoutA = iout;
            if (iout > m_vrDiagnostics.maxIoutA) {
                m_vrDiagnostics.maxIoutA = iout;
            }
        }
        m_vrDiagnostics.sampleCount++;
        m_vrDiagnostics.lastSampleUs = esp_timer_get_time();
    }
    return iout;
}

float NerdaxeGaia::getPout() {
    return getPin();
}

Board::Error NerdaxeGaia::getFault(uint32_t *status)
{
    const uint16_t status_word = gaia_pmbus_read_word(PMBUS_STATUS_WORD);
    const uint8_t status_byte = gaia_pmbus_read_byte(PMBUS_STATUS_BYTE);
    const uint8_t status_iout = gaia_pmbus_read_byte(PMBUS_STATUS_IOUT);
    const uint8_t status_vout = gaia_pmbus_read_byte(PMBUS_STATUS_VOUT);
    const uint8_t status_input = gaia_pmbus_read_byte(PMBUS_STATUS_INPUT);
    const uint8_t status_temp = gaia_pmbus_read_byte(PMBUS_STATUS_TEMPERATURE);
    const uint8_t status_mfr = gaia_pmbus_read_byte(PMBUS_STATUS_MFR_SPECIFIC);

    *status = (static_cast<uint32_t>(status_byte) << 24) |
              (static_cast<uint32_t>(status_iout) << 16) |
              (static_cast<uint32_t>(status_vout) << 8) |
              static_cast<uint32_t>(status_input);

    Board::Error error = Board::Error::NONE;

    // If the regulator itself cannot be reached, every PMBus status read comes
    // back as 0xFF.  Treat that separately from a real latched fault bit.
    if (status_word == 0xffff && status_byte == 0xff && status_iout == 0xff &&
        status_vout == 0xff && status_input == 0xff && status_temp == 0xff && status_mfr == 0xff) {
        error = Board::Error::PSU_FAULT;
    } else if (status_iout != 0xff && (status_iout & 0x80)) {
        // PMBus STATUS_IOUT bit 7 = output over-current fault.
        error = Board::Error::IOUT_OC_FAULT;
    } else if (status_vout != 0xff && (status_vout & 0x90)) {
        // Bit 7 = VOUT over-voltage fault, bit 4 = VOUT under-voltage fault.
        error = Board::Error::VOUT_FAULT;
    } else if (status_temp != 0xff && (status_temp & 0x80)) {
        // Bit 7 = over-temperature fault.
        error = Board::Error::VREG_TEMP_FAULT;
    } else if (status_input != 0xff && (status_input & 0x94)) {
        // Bit 7 = VIN OV fault, bit 4 = VIN UV fault, bit 2 = IIN OC fault.
        error = Board::Error::PSU_FAULT;
    }

    // Keep a frozen copy of the first regulator fault.  PMBus fault bits are
    // latched, but freezing the report also protects it from later UI/settings
    // activity and gives us the exact configuration that was active at capture.
    if (error != Board::Error::NONE && !m_vrDiagnostics.faultLatched) {
        m_vrDiagnostics.faultLatched = true;
        m_vrDiagnostics.fault = error;
        m_vrDiagnostics.faultCapturedUs = esp_timer_get_time();
        m_vrDiagnostics.faultFrequencyMHz = static_cast<uint16_t>(m_asicFrequency);
        m_vrDiagnostics.faultCoreVoltageMv = static_cast<uint16_t>(m_asicVoltageMillis);
        m_vrDiagnostics.faultIoutA = TPS546_get_iout();
        m_vrDiagnostics.faultVoutV = ADC_get_vcore() / 1000.0f;
        m_vrDiagnostics.faultVrTempC = TPS546_get_temperature();

        m_vrDiagnostics.statusWord = status_word;
        m_vrDiagnostics.statusByte = status_byte;
        m_vrDiagnostics.statusIout = status_iout;
        m_vrDiagnostics.statusVout = status_vout;
        m_vrDiagnostics.statusInput = status_input;
        m_vrDiagnostics.statusTemperature = status_temp;
        m_vrDiagnostics.statusMfrSpecific = status_mfr;

        ESP_LOGE(TAG,
                 "VR diagnostic fault captured: error=%s word=%04X byte=%02X iout=%02X vout=%02X input=%02X temp=%02X mfr=%02X lastIout=%.2fA maxIout=%.2fA lastVout=%.3fV vrTemp=%.2fC",
                 Board::errorToStr(error), status_word, status_byte, status_iout, status_vout, status_input,
                 status_temp, status_mfr, m_vrDiagnostics.lastNonZeroIoutA, m_vrDiagnostics.maxIoutA,
                 m_vrDiagnostics.lastNonZeroVoutV, m_vrDiagnostics.faultVrTempC);
    } else if (!m_vrDiagnostics.faultLatched) {
        // Before a fault, retain the latest status values so the endpoint is
        // still useful while the unit is running normally.
        m_vrDiagnostics.statusWord = status_word;
        m_vrDiagnostics.statusByte = status_byte;
        m_vrDiagnostics.statusIout = status_iout;
        m_vrDiagnostics.statusVout = status_vout;
        m_vrDiagnostics.statusInput = status_input;
        m_vrDiagnostics.statusTemperature = status_temp;
        m_vrDiagnostics.statusMfrSpecific = status_mfr;
    }

    return error;
}
