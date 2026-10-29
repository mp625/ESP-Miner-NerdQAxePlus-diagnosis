#pragma once

#include <stdint.h>

#include "asic.h"
#include "bm1373.h"
#include "board.h"
#include "nerdaxe.h"

// Snapshot used by the Gaia VR diagnostic build. It deliberately contains
// only regulator/board telemetry and no Wi-Fi or pool credentials.
struct GaiaVrDiagnostics {
    bool faultLatched = false;
    Board::Error fault = Board::Error::NONE;

    uint64_t lastSampleUs = 0;
    uint64_t faultCapturedUs = 0;
    uint32_t sampleCount = 0;

    float lastIoutA = 0.0f;
    float lastNonZeroIoutA = 0.0f;
    float maxIoutA = 0.0f;

    float lastVoutV = 0.0f;
    float lastNonZeroVoutV = 0.0f;
    float lastVrTempC = 0.0f;

    float faultIoutA = 0.0f;
    float faultVoutV = 0.0f;
    float faultVrTempC = 0.0f;
    uint16_t faultFrequencyMHz = 0;
    uint16_t faultCoreVoltageMv = 0;

    uint16_t statusWord = 0xffff;
    uint8_t statusByte = 0xff;
    uint8_t statusIout = 0xff;
    uint8_t statusVout = 0xff;
    uint8_t statusInput = 0xff;
    uint8_t statusTemperature = 0xff;
    uint8_t statusMfrSpecific = 0xff;
};

class NerdaxeGaia : public NerdAxe {
  protected:
    int m_initVoltageMillis;

    // W5500 interposer present (auto-detected in the ctor).
    bool m_hasEth = false;

    // LDO enable line (GPIO12) — power sequencing helpers
    void LDO_enable();
    void LDO_disable();

    GaiaVrDiagnostics m_vrDiagnostics;
    bool m_vrDiagTaskRunning = false;
    void resetVrDiagnostics();
    static void vrDiagnosticTask(void *arg);

  public:
    NerdaxeGaia();

    virtual bool initBoard();
    virtual bool initAsics();

    virtual void shutdown();

    virtual bool setVoltage(float core_voltage);

    virtual float getTemperature(int index);
    virtual float getVRTemp();
    virtual bool isPIDAvailable() { return true; }

    virtual float getVin();
    virtual float getIin();
    virtual float getPin();
    virtual float getVout();
    virtual float getIout();
    virtual float getPout();

    // Decode latched TPS546D24A PMBus fault registers. The common power
    // management task already calls getFault() every telemetry cycle; Gaia did
    // not previously override it, so regulator faults were invisible there.
    virtual Board::Error getFault(uint32_t *status) override;

    GaiaVrDiagnostics getVrDiagnostics() const { return m_vrDiagnostics; }

    virtual bool hasEthernet() override { return m_hasEth; }
    virtual const EthPins *getEthPins() override;
};
