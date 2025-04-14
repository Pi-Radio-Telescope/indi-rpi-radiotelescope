#ifndef VOLTAGEMONITOR_H
#define VOLTAGEMONITOR_H

#include <chrono>
#include <functional>
#include <inttypes.h> // uint8_t, etc
#include <iomanip>
#include <iostream>
#include <list>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gpioif.h"
#include "utility.h"

class ADS1115;

namespace PiRaTe {

class Ads1115VoltageMonitor {
public:
    Ads1115VoltageMonitor() = delete;

    Ads1115VoltageMonitor(std::string name,
        std::shared_ptr<ADS1115> adc,
        std::uint8_t adc_channel,
        double nominalVoltage,
        double divider_ratio = 1.,
        double max_abs_tolerance = 0.1);

    ~Ads1115VoltageMonitor();

    [[nodiscard]] auto isFault() -> bool;
    [[nodiscard]] auto isInitialized() const -> bool { return fActiveLoop; }
    [[nodiscard]] auto hasAdc() const -> bool { return (fAdc != nullptr); }
    [[nodiscard]] auto currentVoltage() -> double;
    [[nodiscard]] auto meanVoltage() -> double;
    [[nodiscard]] auto nominalVoltage() const -> double { return fNominalVoltage; }
    [[nodiscard]] auto lowLimit() const -> double { return fLoLimit; }
    [[nodiscard]] auto highLimit() const -> double { return fHiLimit; }
    [[nodiscard]] auto dividerRatio() const -> double { return fDividerRatio; }
    [[nodiscard]] auto name() const -> std::string { return fName; }

    void registerVoltageReadyCallback(std::function<void(double)> fn) { fVoltageReadyFn = fn; }

private:
    void threadLoop();

    std::string fName { "GND" };
    std::shared_ptr<ADS1115> fAdc { nullptr };
    bool fUpdated { false };
    bool fActiveLoop { false };
    std::uint8_t fAdcChannel { 0 };

    std::unique_ptr<std::thread> fThread { nullptr };

    std::mutex fMutex;
    std::function<void(double)> fVoltageReadyFn {};

    double fVoltage { 0. };
    Ringbuffer<double, 100> fBuffer {};

    double fNominalVoltage { 0. };
    double fLoLimit { -0.001 };
    double fHiLimit { 0.001 };
    double fDividerRatio { 1. };
};

} // namespace PiRaTe

#endif // #ifdef VOLTAGEMONITOR_H
