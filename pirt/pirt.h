/*
   INDI Raspberry Pi based mount driver.
   The driver itself acts as a telescope mount reading the positions from SSI-based absolute encoders from
   the on-board SPI interfaces and driving DC motors via PWM over GPIO pins
   "Pi Radiotelescope Driver"

*/

/** \file pirt.h
    \brief A customized INDI telescope device with full functionality of a telescope mount.
    \author Hans-Georg Zaunick
*/

#pragma once

#include "inditelescope.h"
#include <ads1115_measurement.h>
#include <axis.h>
#include <rpi_temperatures.h>
#include <voltage_monitor.h>

#include <map>

struct HorCoords {
    HorCoords()
    {
        Alt.registerGimbalFlipCallback([this]() { this->Az.gimbalFlip(); });
    }
    HorCoords(double az, double alt)
    {
        Alt.registerGimbalFlipCallback([this]() { this->Az.gimbalFlip(); });
        Az.setValue(az);
        Alt.setValue(alt);
    }
    PiRaTe::RotAxis Az { 0., 360., 360. };
    PiRaTe::RotAxis Alt { -90., 90., 360. };
};

struct EquCoords {
    EquCoords()
    {
        Dec.registerGimbalFlipCallback([this]() { this->Ra.gimbalFlip(); });
    }
    EquCoords(double ra, double dec)
    {
        Dec.registerGimbalFlipCallback([this]() { this->Ra.gimbalFlip(); });
        Ra.setValue(ra);
        Dec.setValue(dec);
    }
    PiRaTe::RotAxis Ra { 0., 24., 24. };
    PiRaTe::RotAxis Dec { -90., 90., 360. };
};

class GPIO;

namespace PiRaTe {
    class SsiPosEncoder;
    class MotorDriver;
    class i2cDevice;
    class ADS1115;
}


class PiRT : public INDI::Telescope {
public:
    enum {
        SYSTEM_EQ,
        SYSTEM_HOR,
        SYSTEM_GAL
    } TargetCoordSystem;

    enum {
        AXIS_AZ,
        AXIS_ALT
    };

    PiRT();
    //~PiRT() override;

    bool Connect() override;
    bool Disconnect() override;
    void TimerHit() override;
    virtual bool ISNewSwitch(const char* dev, const char* name, ISState* states, char* names[], int n) override;
    virtual bool ISNewNumber(const char* dev, const char* name, double values[], char* names[], int n) override;
    virtual bool ISSnoopDevice(XMLEle* root) override;
    virtual bool saveConfigItems(FILE *fp) override;

    [[nodiscard]] auto isTracking() const -> bool;

protected:
    bool Handshake() override;
    const char* getDefaultName() override;
    bool initProperties() override;
    bool updateProperties() override;

    // Telescope specific functions
    bool ReadScopeStatus() override;
    bool Goto(double, double) override;
    bool GotoHor(double, double);
    bool Abort() override;
    bool SetTrackMode(uint8_t mode) override;
    bool SetTrackEnabled(bool enabled) override;
    bool Park() override;
    bool UnPark() override;
    bool MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) override;
    bool MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) override;

private:
    void Hor2Equ(double az, double alt, double* ra, double* dec);
    void Hor2Equ(const HorCoords& hor_coords, double* ra, double* dec);
    void Equ2Hor(double ra, double dec, double* az, double* alt);
    HorCoords Equ2Hor(const EquCoords& equ_coords);
    EquCoords Hor2Equ(const HorCoords& hor_coords);
    bool isInAbsoluteTurnRangeAz(double absRev);
    bool isInAbsoluteTurnRangeAlt(double absRev);

    void updatePosition();
    void updateMotorStatus();
    void updateMonitoring();
    void updateTemperatures(PiRaTe::RpiTemperatureMonitor::TemperatureItem item);
    void updateTime();
    auto upTime() const -> std::chrono::duration<long, std::ratio<1>>;

    ILight ScopeStatusL[5];
    ILightVectorProperty ScopeStatusLP;
    INumber HorN[2];
    INumberVectorProperty HorNP;
    INumber JDN;
    INumberVectorProperty JDNP;

    INumber EncoderBitRateN;
    INumberVectorProperty EncoderBitRateNP;

    INumber AzEncoderN[5];
    INumber ElEncoderN[5];
    INumberVectorProperty AzEncoderNP;
    INumberVectorProperty ElEncoderNP;
    INumber AzEncSettingN[2], ElEncSettingN[2];
    INumberVectorProperty AzEncSettingNP, ElEncSettingNP;

    INumber AzAxisSettingN[2], ElAxisSettingN[2];
    INumberVectorProperty AzAxisSettingNP, ElAxisSettingNP;

    INumber MotorStatusN[2];
    INumberVectorProperty MotorStatusNP;

    INumber MotorThresholdN[2];
    INumberVectorProperty MotorThresholdNP;

    INumber MotorCurrentN[2];
    INumberVectorProperty MotorCurrentNP;

    INumber MotorCurrentLimitN[2];
    INumberVectorProperty MotorCurrentLimitNP;

    INumber VoltageMonitorN[64];
    INumberVectorProperty VoltageMonitorNP;

    INumber VoltageMeasurementN[16];
    INumberVectorProperty VoltageMeasurementNP;
    INumber MeasurementIntTimeN;
    INumberVectorProperty MeasurementIntTimeNP;

    INumber TempMonitorN[64];
    INumberVectorProperty TempMonitorNP;

    INumber AxisAbsTurnsN[2];
    INumberVectorProperty AxisAbsTurnsNP;

    ISwitch OutputSwitchS[16];
    ISwitchVectorProperty OutputSwitchSP;

    ILight GpioInputL[16];
    ILightVectorProperty GpioInputLP;

    INumber DriverUpTimeN;
    INumberVectorProperty DriverUpTimeNP;

    ILight WeatherStatusN;
    ILightVectorProperty WeatherStatusNP;

    ISwitch ErrorResetS;
    ISwitchVectorProperty ErrorResetSP;

    bool fIsTracking { false };

    double axisRatio[2] { 1., 1. };
    double axisOffset[2] { 0., 0. };

    IPState lastHorState;
    uint8_t DBG_SCOPE { INDI::Logger::DBG_IGNORE };

    std::shared_ptr<GPIO> gpio { nullptr };
    std::unique_ptr<PiRaTe::SsiPosEncoder> az_encoder { nullptr };
    std::unique_ptr<PiRaTe::SsiPosEncoder> el_encoder { nullptr };
    std::unique_ptr<PiRaTe::MotorDriver> az_motor { nullptr };
    std::unique_ptr<PiRaTe::MotorDriver> el_motor { nullptr };
    std::map<std::uint8_t, std::shared_ptr<PiRaTe::i2cDevice>> i2cDeviceMap {};
    std::shared_ptr<PiRaTe::RpiTemperatureMonitor> tempMonitor { nullptr };
    HorCoords currentHorizontalCoords { 0., 90. };
    HorCoords targetHorizontalCoords { 0., 90. };
    EquCoords targetEquatorialCoords { 0., 0. };

    std::vector<std::shared_ptr<PiRaTe::Ads1115VoltageMonitor>> voltageMonitors {};
    std::vector<std::shared_ptr<PiRaTe::Ads1115Measurement>> voltageMeasurements {};
    std::chrono::time_point<std::chrono::system_clock> fStartTime {};
    unsigned int targetPointingCycles { 0 };
};
