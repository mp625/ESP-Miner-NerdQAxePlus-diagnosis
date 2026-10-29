#include <cstring>
#include <math.h>

#include "nerdaxegaia.h"
#include "global_state.h"

#include "drivers/i2c_master.h"
#include "drivers/nerdaxe/adc.h"
#include "drivers/nerdaxe/TPS546.h"
#include "drivers/nerdaxe/pmbus_commands.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GAIA_PGOOD_PIN GPIO_NUM_11
#define GAIA_PMB_ALERT_PIN GPIO_NUM_13

static const char *TAG = "gaiaVrDiagV2";

static uint8_t diag_read_byte(uint8_t command)
{
    uint8_t value = 0xff;
    if (i2c_master_register_read(TPS546_I2CADDR, command, &value, 1) != ESP_OK) {
        return 0xff;
    }
    return value;
}

static uint16_t diag_read_word(uint8_t command)
{
    uint8_t data[2] = {0xff, 0xff};
    if (i2c_master_register_read(TPS546_I2CADDR, command, data, sizeof(data)) != ESP_OK) {
        return 0xffff;
    }
    return (static_cast<uint16_t>(data[1]) << 8) | data[0];
}

static Board::Error decode_fault(uint8_t statusIout,
                                uint8_t statusVout,
                                uint8_t statusInput,
                                uint8_t statusTemp)
{
    if (statusIout != 0xff && (statusIout & 0x80)) {
        return Board::Error::IOUT_OC_FAULT;
    }
    if (statusVout != 0xff && (statusVout & 0x90)) {
        return Board::Error::VOUT_FAULT;
    }
    if (statusTemp != 0xff && (statusTemp & 0x80)) {
        return Board::Error::VREG_TEMP_FAULT;
    }
    if (statusInput != 0xff && (statusInput & 0x94)) {
        return Board::Error::PSU_FAULT;
    }
    return Board::Error::NONE;
}

void NerdaxeGaia::startVrDiagnosticMonitor()
{
    if (m_vrDiagTaskRunning) {
        return;
    }

    m_vrDiagTaskRunning = true;
    xTaskCreate(vrDiagnosticTask, "gaia vr diag", 4096, this, 11, nullptr);
}

void NerdaxeGaia::vrDiagnosticTask(void *arg)
{
    auto *gaia = static_cast<NerdaxeGaia *>(arg);

    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << GAIA_PGOOD_PIN) | (1ULL << GAIA_PMB_ALERT_PIN);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);

    ESP_LOGI(TAG, "V2 monitor armed: PGOOD=GPIO11, PMB_ALERT=GPIO13");

    bool activeWindow = false;
    bool previousPgood = false;
    bool previousAlert = true;
    uint64_t windowStartUs = 0;
    uint64_t lastIoutReadUs = 0;

    for (;;) {
        const uint64_t nowUs = esp_timer_get_time();
        const float vout = ADC_get_vcore() / 1000.0f;
        const bool pgood = gpio_get_level(GAIA_PGOOD_PIN) != 0;
        const bool alertInactive = gpio_get_level(GAIA_PMB_ALERT_PIN) != 0; // ALERT is active-low

        if (!gaia->m_vrDiagnostics.faultLatched) {
            gaia->m_vrDiagnostics.lastSampleUs = nowUs;
            gaia->m_vrDiagnostics.lastVoutV = vout;
            if (isfinite(vout) && vout > 0.05f) {
                gaia->m_vrDiagnostics.lastNonZeroVoutV = vout;
            }
        }

        if (!activeWindow && vout > 0.20f) {
            activeWindow = true;
            windowStartUs = nowUs;
            previousPgood = pgood;
            previousAlert = alertInactive;
            ESP_LOGI(TAG, "startup window detected at Vcore=%.3fV PGOOD=%d ALERT=%d",
                     vout, (int)pgood, (int)!alertInactive);
        }

        if (activeWindow && !gaia->m_vrDiagnostics.faultLatched) {
            // Read current at 20 Hz during the critical startup window.
            if ((nowUs - lastIoutReadUs) >= 50000ULL) {
                const float iout = TPS546_get_iout();
                lastIoutReadUs = nowUs;
                if (isfinite(iout)) {
                    gaia->m_vrDiagnostics.lastIoutA = iout;
                    if (iout > 0.05f) {
                        gaia->m_vrDiagnostics.lastNonZeroIoutA = iout;
                        if (iout > gaia->m_vrDiagnostics.maxIoutA) {
                            gaia->m_vrDiagnostics.maxIoutA = iout;
                        }
                    }
                    gaia->m_vrDiagnostics.sampleCount++;
                }
            }

            const bool pgoodFell = previousPgood && !pgood;
            const bool alertAsserted = previousAlert && !alertInactive;
            const bool vcoreCollapsed = gaia->m_vrDiagnostics.lastNonZeroVoutV > 0.20f && vout < 0.10f;

            if (pgoodFell || alertAsserted || vcoreCollapsed) {
                // Read the PMBus status immediately, before the regular telemetry task
                // or later recovery activity can obscure the originating fault bits.
                const uint16_t statusWord = diag_read_word(PMBUS_STATUS_WORD);
                const uint8_t statusByte = diag_read_byte(PMBUS_STATUS_BYTE);
                const uint8_t statusIout = diag_read_byte(PMBUS_STATUS_IOUT);
                const uint8_t statusVout = diag_read_byte(PMBUS_STATUS_VOUT);
                const uint8_t statusInput = diag_read_byte(PMBUS_STATUS_INPUT);
                const uint8_t statusTemp = diag_read_byte(PMBUS_STATUS_TEMPERATURE);
                const uint8_t statusMfr = diag_read_byte(PMBUS_STATUS_MFR_SPECIFIC);

                gaia->m_vrDiagnostics.faultLatched = true;
                gaia->m_vrDiagnostics.fault = decode_fault(statusIout, statusVout, statusInput, statusTemp);
                gaia->m_vrDiagnostics.faultCapturedUs = nowUs;
                gaia->m_vrDiagnostics.faultFrequencyMHz = static_cast<uint16_t>(gaia->m_asicFrequency);
                gaia->m_vrDiagnostics.faultCoreVoltageMv = static_cast<uint16_t>(gaia->m_asicVoltageMillis);
                gaia->m_vrDiagnostics.faultIoutA = TPS546_get_iout();
                gaia->m_vrDiagnostics.faultVoutV = ADC_get_vcore() / 1000.0f;
                gaia->m_vrDiagnostics.faultVrTempC = TPS546_get_temperature();
                gaia->m_vrDiagnostics.statusWord = statusWord;
                gaia->m_vrDiagnostics.statusByte = statusByte;
                gaia->m_vrDiagnostics.statusIout = statusIout;
                gaia->m_vrDiagnostics.statusVout = statusVout;
                gaia->m_vrDiagnostics.statusInput = statusInput;
                gaia->m_vrDiagnostics.statusTemperature = statusTemp;
                gaia->m_vrDiagnostics.statusMfrSpecific = statusMfr;

                ESP_LOGE(TAG,
                         "FAST CAPTURE trigger=%s%s%s dt=%.3fms Vcore=%.3fV Iout=%.2fA maxIout=%.2fA PGOOD=%d ALERT=%d WORD=%04X BYTE=%02X IOUT=%02X VOUT=%02X INPUT=%02X TEMP=%02X MFR=%02X",
                         pgoodFell ? "PGOOD " : "",
                         alertAsserted ? "ALERT " : "",
                         vcoreCollapsed ? "VCORE " : "",
                         (nowUs - windowStartUs) / 1000.0,
                         gaia->m_vrDiagnostics.faultVoutV,
                         gaia->m_vrDiagnostics.faultIoutA,
                         gaia->m_vrDiagnostics.maxIoutA,
                         (int)pgood, (int)!alertInactive,
                         statusWord, statusByte, statusIout, statusVout,
                         statusInput, statusTemp, statusMfr);
            }

            previousPgood = pgood;
            previousAlert = alertInactive;

            // If startup completed without a fault, keep watching until the next
            // Vcore cycle instead of continuously polling the regulator forever.
            if ((nowUs - windowStartUs) > 15000000ULL && vout > 0.20f) {
                activeWindow = false;
            }
        }

        if (activeWindow && vout < 0.05f && !gaia->m_vrDiagnostics.faultLatched) {
            activeWindow = false;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// Start a tiny bootstrap task before app_main. It does not touch the board until
// SYSTEM_MODULE has installed a NerdAxeGaia instance and initBoard() has had time
// to initialize GPIO/I2C. This keeps the diagnostic patch isolated from normal
// startup sequencing.
static void gaia_diag_bootstrap(void *)
{
    for (;;) {
        Board *board = SYSTEM_MODULE.getBoard();
        if (board && strcmp(board->getDeviceModel(), "NerdAxeGaia") == 0) {
            vTaskDelay(pdMS_TO_TICKS(250));
            static_cast<NerdaxeGaia *>(board)->startVrDiagnosticMonitor();
            vTaskDelete(nullptr);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

__attribute__((constructor)) static void gaia_diag_register_bootstrap()
{
#ifdef NERDAXEGAIA
    xTaskCreate(gaia_diag_bootstrap, "gaia diag bootstrap", 2048, nullptr, 2, nullptr);
#endif
}
