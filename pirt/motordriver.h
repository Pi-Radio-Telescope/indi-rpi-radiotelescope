#pragma once

#include <chrono>
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
#include "sysfspwm.hpp"
#include "utility.h"

class Gpio;

namespace PiRaTe {

class ADS1115;

constexpr unsigned int DEFAULT_PWM_FREQ { 20'000 };
constexpr unsigned int OFFSET_RINGBUFFER_DEPTH { 16 };

/**
 * @brief Interface class for control of PWM-based DC motor driver boards.
 * This class provides the interface to control a single channel DC motor control via the RPi's GPIO interface.
 * The assignment of the different GPIO pins is set in the constructor call by provision of a {@link MotorDriver::Pins} struct.
 * For a configuration with minimum functionality, at least the Pwm and Dir pins must be defined. In case, the direction signal
 * is expected as differential signal (DirA=normal signal, DirB=inverted signal), the DirA and DirB pins must be
 * defined. The Dir pin is ignored in this case. Enable and Fault signals are not mandatory, but used and evaluated
 * when defined. Set unused signals to -1.
 * Some motor driver modules provide an analog signal for the supervision of the motor current. If this shall be
 * measured, a shared pointer to an instance of an {@link ADS1115} class can be provided additionally in the constructor.
 * It is assumed, that the motor driver's current-supervision signal is connected to one input channel of the ADC.
 * Specify the corresponding ADS1115 channel in the constructor in this case.
 * @note In order to use the hardware pwm interface, the sysfs entries have to be created first by
 * enabling the pwm-2chan overlay in /boot/config.txt with the following entry:
 * @verbatim dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4 @endverbatim
 * @author HG Zaunick
 */
class MotorDriver {
public:
    /**
	* @brief Struct for storing the configuration of the motor driver gpio pins.
	* In this struct all gpio pins are defined which are connected to the actual motor driver hardware.
	* @note Unused pins must be set to -1.
	*/
    struct Pins {
        //int Pwm; ///< GPIO pin of the PWM signal (output)
        int Dir; ///< GPIO pin of the direction signal (output). This field must be defined, when only one direction signal is used
        int DirA; ///< GPIO pin of the direction signal (output, normal polarity). Use this field together with the DirB pin when two phase shifted signals are used for the direction
        int DirB; ///< GPIO pin of the direction signal (output, inverted polarity). Use this field together with the DirA pin when two phase shifted signals are used for the direction
        int Enable; ///< GPIO pin of the enable signal (output)
        int Fault; ///< GPIO pin of the fault signal (low-active input). The internal pull-up will be enabled when using this signal)
    };

    MotorDriver() = delete;
    /**
	* @brief The main constructor.
	* Initializes an object with the given gpio and pwm object pointers
    * and the gpio pin configuration.
	* @param gpio shared pointer to an initialized GPIO object
	* @param pwm shared pointer to a PWM object (must be already exported)
	* @param invertDirection flag which indicates, that positive/negative direction will be swapped
	* @param adc shared_ptr object to an initialized instance of {@link ADS1115} ADC (not mandatory)
	* @param adc_channel channel to use for supervision of motor current in case an adc is specified
	* @throws std::exception if the supplied gpio or pwm objects are not initialized
	*/

    MotorDriver(
        std::shared_ptr<Gpio> gpio,
        std::shared_ptr<sysfspwm::PWM> pwm,
        Pins pins,
        bool invertDirection = false,
        std::shared_ptr<ADS1115> adc = nullptr,
        std::uint8_t adc_channel = 0);

    ~MotorDriver();

    [[nodiscard]] auto getPinConfig() const -> Pins { return fPins; }
    void setPwmFrequency(unsigned int freq);

    void move(float speed_ratio);
    void stop();
    void emergencyStop();
    [[nodiscard]] auto isFault() -> bool;
    [[nodiscard]] auto isInitialized() const -> bool { return fActiveLoop; }
    [[nodiscard]] auto currentSpeed() -> float;
    [[nodiscard]] auto hasFaultSense() const -> bool { return (fPins.Fault > 0); }
    [[nodiscard]] auto hasEnable() const -> bool { return (fPins.Enable > 0); }
    [[nodiscard]] auto hasDualDir() const -> bool { return ((fPins.DirA > 0) && (fPins.DirB > 0)); }
    [[nodiscard]] auto hasAdc() const -> bool { return (fAdc != nullptr); }
    [[nodiscard]] auto readCurrent() -> double;
    [[nodiscard]] auto readMaxCurrent() -> double;
    void resetMaxCurrent();

    void setEnabled(bool enable);
    [[nodiscard]] auto adc() -> std::shared_ptr<ADS1115>& { return fAdc; }

private:
    void threadLoop();
    void setSpeed(float speed_ratio);
    void measureVoltageOffset();

    std::shared_ptr<Gpio> fGpio { nullptr };
    std::shared_ptr<sysfspwm::PWM> fPwm { nullptr };
    Pins fPins;
    std::shared_ptr<ADS1115> fAdc { nullptr };
    unsigned int fPwmFreq { DEFAULT_PWM_FREQ };
    bool fUpdated { false };
    float fCurrentDutyCycle { 0. };
    float fTargetDutyCycle { 0. };
    bool fActiveLoop { false };
    bool fCurrentDir { false };
    bool fInverted { false };
    std::uint8_t fAdcChannel { 0 };
    double fCurrent { 0. };
    double fMaxCurrent { 0. };

    std::unique_ptr<std::thread> fThread { nullptr };

    std::mutex fMutex;

    Ringbuffer<double, OFFSET_RINGBUFFER_DEPTH> fOffsetBuffer {};
};

} // namespace PiRaTe
