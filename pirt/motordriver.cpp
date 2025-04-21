#include <algorithm>
#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <string>
#include <unistd.h>

#include "ads1115.h"
#include "motordriver.h"
#include "utility.h"

#define DEFAULT_VERBOSITY 1

namespace PiRaTe {

constexpr std::chrono::milliseconds loop_delay { 10 };
constexpr std::chrono::milliseconds ramp_time { 100 };
constexpr std::size_t adc_measurement_rate_loop_cycles { 10 };
constexpr double ramp_increment { static_cast<double>(loop_delay.count()) / ramp_time.count() };
// constexpr unsigned int HW_PWM1_PIN { 12 };
// constexpr unsigned int HW_PWM2_PIN { 13 };
constexpr double MOTOR_CURRENT_FACTOR { 1. / 0.14 }; //< conversion factor for motor current sense in A/V

MotorDriver::MotorDriver(
    std::shared_ptr<Gpio> gpio,
    std::shared_ptr<sysfspwm::PWM> pwm,
    Pins pins,
    bool invertDirection,
    std::shared_ptr<ADS1115> adc,
    std::uint8_t adc_channel)
    : fGpio { gpio }
    , fPwm { pwm }
    , fPins { pins }
    , fAdc { adc }
    , fCurrentDir { false }
    , fInverted { invertDirection }
    , fAdcChannel { adc_channel }
{
    if (fGpio == nullptr) {
        std::cerr << "Error: no valid GPIO instance.\n";
        return;
    }

    if (fPwm == nullptr) {
        std::cerr << "Error: no valid PWM instance.\n";
        return;
    }

    if (!fGpio->is_initialised()) {
        std::cerr << "Error: gpio interface not initialized.\n";
        return;
    }

    if ( fPins.Dir < 0 && !hasDualDir() ) {
        std::cerr << "Error: mandatory gpio pins for motor control undefined.\n";
        return;
    }

    // set pin directions
    if (fPins.Dir > 0) {
        fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Dir), Gpio::Direction::DIRECTION_OUTPUT);
        fGpio->set_gpio_state(static_cast<unsigned int>(fPins.Dir), (fInverted) ? !fCurrentDir : fCurrentDir);
    }
    if (hasDualDir()) {
        fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.DirA), Gpio::Direction::DIRECTION_OUTPUT);
        fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.DirB), Gpio::Direction::DIRECTION_OUTPUT);
        fGpio->set_gpio_state(static_cast<unsigned int>(fPins.DirA), (fInverted) ? !fCurrentDir : fCurrentDir);
        fGpio->set_gpio_state(static_cast<unsigned int>(fPins.DirB), (fInverted) ? fCurrentDir : !fCurrentDir);
    }

    //fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Pwm), true);

    if (fPins.Enable > 0) {
        fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Enable), Gpio::Direction::DIRECTION_OUTPUT);
        fGpio->set_gpio_state(static_cast<unsigned int>(fPins.Enable), true);
    }
    if (fPins.Fault > 0) {
        fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Fault), Gpio::Direction::DIRECTION_INPUT);
        fGpio->set_gpio_pullup(static_cast<unsigned int>(fPins.Fault));
    }

    // initialize ADC if one was supplied in the argument list
    if (fAdc != nullptr && fAdc->devicePresent()) {
        //fAdc->setPga(ADS1115::PGA4V);
        //fAdc->setRate(ADS1115::RATE860);
        //fAdc->setAGC(true);
        const std::lock_guard<std::mutex> lock(fMutex);
        measureVoltageOffset();
    }

    fActiveLoop = true;
    // since C++14 using std::make_unique
    // fThread = std::make_unique<std::thread>( [this]() { this->readLoop(); } );
    // C++11 is unfortunately more unconvenient with move from a locally generated pointer
    std::unique_ptr<std::thread> thread(new std::thread([this]() { this->threadLoop(); }));
    fThread = std::move(thread);
}

MotorDriver::~MotorDriver()
{
    if (!fActiveLoop)
        return;
    fActiveLoop = false;
    if (fThread != nullptr)
        fThread->join();
    if (fPwm != nullptr) {
        fPwm->set_enabled(false);
    }
    if (fGpio != nullptr && fGpio->is_initialised()) {
        if (fPins.Dir > 0)
            fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Dir), Gpio::Direction::DIRECTION_INPUT);
        if (fPins.DirA > 0)
            fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.DirA), Gpio::Direction::DIRECTION_INPUT);
        if (fPins.DirB > 0)
            fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.DirB), Gpio::Direction::DIRECTION_INPUT);
        //fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Pwm), false);
        if (fPins.Enable > 0) {
            fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Enable), Gpio::Direction::DIRECTION_INPUT);
        }
        if (fPins.Fault > 0) {
            fGpio->set_gpio_pullup(static_cast<unsigned int>(fPins.Fault), false);
        }
    }
}

// this is the background thread loop
void MotorDriver::threadLoop()
{
    std::size_t cycle_counter { adc_measurement_rate_loop_cycles };
    auto lastReadOutTime = std::chrono::system_clock::now();
    while (fActiveLoop) {
        auto currentTime = std::chrono::system_clock::now();

        if (hasFaultSense() && isFault()) {
            // fault condition, switch off and deactivate everything
            emergencyStop();
        } else {
            const std::lock_guard<std::mutex> lock(fMutex);
            if (fTargetDutyCycle != fCurrentDutyCycle) {
                fCurrentDutyCycle += ramp_increment * sgn(fTargetDutyCycle - fCurrentDutyCycle);
                if (std::abs(fTargetDutyCycle - fCurrentDutyCycle) < ramp_increment) {
                    fCurrentDutyCycle = fTargetDutyCycle;
                }
                setSpeed(fCurrentDutyCycle);
            }
        }
        if (hasAdc() && !cycle_counter--) {
            double voltage { 0. };
            double conv_time { 0. };
            if ([[maybe_unused]] bool readout_guard = true) {
                // read current from adc
                fMutex.lock();
                voltage = fAdc->readVoltage(fAdcChannel);
                conv_time = fAdc->getLastConvTime();
                fMutex.unlock();
            }
            if (std::abs(fCurrentDutyCycle) < ramp_increment)
                fOffsetBuffer.add(voltage);
            double _current = (voltage - fOffsetBuffer.mean()) * MOTOR_CURRENT_FACTOR;
            fMutex.lock();
            fCurrent = _current;
            if (_current > fMaxCurrent)
                fMaxCurrent = _current;
            fUpdated = true;
            fMutex.unlock();
            cycle_counter = adc_measurement_rate_loop_cycles;
            std::this_thread::sleep_for(std::chrono::milliseconds(std::max(loop_delay.count() - static_cast<long long int>(conv_time), 1LL)));
        } else {
            std::this_thread::sleep_for(loop_delay);
        }
    }
}

void MotorDriver::measureVoltageOffset()
{

    for (unsigned int i = 0; i < 10; i++) {
        double voltage = fAdc->readVoltage(fAdcChannel);
        fOffsetBuffer.add(voltage);
    }
}

auto MotorDriver::readCurrent() -> double
{
    std::lock_guard<std::mutex> lock(fMutex);
    fUpdated = false;
    return (fCurrent);
}

auto MotorDriver::readMaxCurrent() -> double
{
    std::lock_guard<std::mutex> lock(fMutex);
    return (fMaxCurrent);
}

void MotorDriver::resetMaxCurrent()
{
    std::lock_guard<std::mutex> lock(fMutex);
    fMaxCurrent = 0.;
}

auto MotorDriver::isFault() -> bool
{
    if (!isInitialized() || !fGpio->is_initialised())
        return true;
    if (!hasFaultSense())
        return false;
    return (fGpio->get_gpio_state(static_cast<unsigned int>(fPins.Fault)) == false);
}

void MotorDriver::setSpeed(float speed_ratio)
{
    const bool dir { (speed_ratio < 0.) };
    // set pins
    if (dir != fCurrentDir) {
        if (fPins.Dir > 0)
            fGpio->set_gpio_state(static_cast<unsigned int>(fPins.Dir), (fInverted) ? !dir : dir);
        if (hasDualDir()) {
            fGpio->set_gpio_state(static_cast<unsigned int>(fPins.DirA), (fInverted) ? !dir : dir);
            fGpio->set_gpio_state(static_cast<unsigned int>(fPins.DirB), (fInverted) ? dir : !dir);
        }
        fCurrentDir = dir;
    }
    float abs_speed_ratio = std::abs(std::clamp(speed_ratio, -1.f, 1.f));
    if (fPwm != nullptr) {
        // use hardware pwm
        fPwm->set_frequency_and_ratio(fPwmFreq, abs_speed_ratio);
        if (abs_speed_ratio > 0.) fPwm->set_enabled(true);
        else fPwm->set_enabled(false);
        return;
    }
}

void MotorDriver::move(float speed_ratio)
{
    const std::lock_guard<std::mutex> lock(fMutex);
    fTargetDutyCycle = std::clamp(speed_ratio, -1.f, 1.f);
}

void MotorDriver::stop()
{
    this->move(0.);
}

void MotorDriver::emergencyStop()
{
    this->move(0.);
    setEnabled(true);
}

void MotorDriver::setEnabled(bool enable)
{
    if (hasEnable()) {
        fGpio->set_gpio_state(static_cast<unsigned int>(fPins.Enable), enable);
    }
}

void MotorDriver::setPwmFrequency(unsigned int freq)
{
    if (freq == fPwmFreq)
        return;
    if (fPwm != nullptr) {
        fMutex.lock();
        float abs_speed_ratio = std::abs(std::clamp(fCurrentDutyCycle, -1.f, 1.f));
        fPwm->set_frequency_and_ratio(freq, abs_speed_ratio);
        fMutex.unlock();
    }
    fPwmFreq = freq;
}

auto MotorDriver::currentSpeed() -> float
{
    const std::lock_guard<std::mutex> lock(fMutex);
    const float speed = fCurrentDutyCycle;
    return speed;
}

} // namespace PiRaTe
