#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <string>
#include <unistd.h>

#include "voltage_monitor.h"

#include "ads1115.h"

#define DEFAULT_VERBOSITY 1

namespace PiRaTe {

constexpr std::chrono::milliseconds loop_delay { 100 };

Ads1115VoltageMonitor::Ads1115VoltageMonitor(std::string name,
    std::shared_ptr<ADS1115> adc,
    std::uint8_t adc_channel,
    double nominalVoltage,
    double divider_ratio,
    double max_abs_tolerance)
    : fName { std::move(name) }
    , fAdc { adc }
    , fAdcChannel { adc_channel }
    , fNominalVoltage { nominalVoltage }
    , fDividerRatio { divider_ratio }
{
    // initialize ADC if one was supplied in the argument list
    if (fAdc != nullptr && fAdc->devicePresent()) {
        //fAdc->setPga(ADS1115::PGA4V);
        //fAdc->setRate(ADS1115::RATE860);
        //fAdc->setAGC(true);
    } else {
        return;
    }
    fLoLimit = fNominalVoltage - max_abs_tolerance;
    fHiLimit = fNominalVoltage + max_abs_tolerance;
    fActiveLoop = true;
    // since C++14 using std::make_unique
    // fThread = std::make_unique<std::thread>( [this]() { this->readLoop(); } );
    // C++11 is unfortunately more unconvenient with move from a locally generated pointer
    std::unique_ptr<std::thread> thread(new std::thread([this]() { this->threadLoop(); }));
    fThread = std::move(thread);
}

Ads1115VoltageMonitor::~Ads1115VoltageMonitor()
{
    if (!fActiveLoop)
        return;
    fActiveLoop = false;
    if (fThread != nullptr)
        fThread->join();
}

// this is the background thread loop
void Ads1115VoltageMonitor::threadLoop()
{
    auto lastReadOutTime = std::chrono::system_clock::now();
    while (fActiveLoop) {
        auto currentTime = std::chrono::system_clock::now();

        if (hasAdc()) {
            if ([[maybe_unused]] bool readout_guard = true) {
                //std::lock_guard<std::mutex> lock(fMutex);
                // read current voltage from adc
                fMutex.lock();
                fVoltage = fAdc->readVoltage(fAdcChannel) * fDividerRatio;
                fMutex.unlock();
            }
            fMutex.lock();
            fBuffer.add(fVoltage);
            fUpdated = true;
            fMutex.unlock();
            if (fVoltageReadyFn)
                fVoltageReadyFn(fVoltage);
        }
        std::this_thread::sleep_for(loop_delay);
    }
}

auto Ads1115VoltageMonitor::currentVoltage() -> double
{
    std::lock_guard<std::mutex> lock(fMutex);
    fUpdated = false;
    return (fVoltage);
}

auto Ads1115VoltageMonitor::meanVoltage() -> double
{
    std::lock_guard<std::mutex> lock(fMutex);
    fUpdated = false;
    return (fBuffer.mean());
}

} // namespace PiRaTe
