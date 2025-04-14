#ifndef MEASUREMENT_H
#define MEASUREMENT_H

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

class Ads1115Measurement {
public:
    struct Sample {
        std::chrono::time_point<std::chrono::system_clock> time;
        double value;
    };

    Ads1115Measurement() = delete;

    Ads1115Measurement(std::string name,
        std::shared_ptr<ADS1115> adc,
        std::uint8_t adc_channel,
        double factor = 1.,
        std::chrono::milliseconds integration_time = std::chrono::milliseconds(1000));

    ~Ads1115Measurement();

    [[nodiscard]] auto isFault() -> bool;
    [[nodiscard]] auto isInitialized() const -> bool { return fActiveLoop; }
    [[nodiscard]] auto hasAdc() const -> bool { return (fAdc != nullptr); }
    [[nodiscard]] auto currentValue() -> double;
    [[nodiscard]] auto meanValue() -> double;
    [[nodiscard]] auto factor() const -> double { return fFactor; }
    [[nodiscard]] auto name() const -> std::string { return fName; }
    void setIntTime(std::chrono::milliseconds ms);
    void setFactor(double factor);

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

    double fValue { 0. };
    std::deque<Sample> fIntegrationBuffer {};

    double fFactor { 1. };
    std::chrono::milliseconds fIntTime { 1000 };
};

} // namespace PiRaTe

#endif // #ifdef MEASUREMENT_H
