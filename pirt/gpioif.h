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
#include <memory>
#include <functional>
#include <map>

#include <gpiod.hpp>

namespace PiRaTe {

constexpr char DEFAULT_GPIO_DEVPATH[] { "/dev/gpiochip0" };    

/**
 * @brief GPIO interface class.
 * This class encapsulates access to the Raspberry Pi GPIO interface based on the gpiod userspace kernel device /dev/gpiochipX.
 * @note 
 * @author HG Zaunick
 */
class Gpio {
public:
    typedef int direction_t;
    struct Direction : private gpiod::line {
    public:
        using gpiod::line::DIRECTION_INPUT;
        using gpiod::line::DIRECTION_OUTPUT;
    };
    struct PinBias : private gpiod::line_request {
    public:    
        using gpiod::line_request::FLAG_ACTIVE_LOW;
        using gpiod::line_request::FLAG_OPEN_SOURCE;
        using gpiod::line_request::FLAG_OPEN_DRAIN;
        using gpiod::line_request::FLAG_BIAS_DISABLE;
        using gpiod::line_request::FLAG_BIAS_PULL_DOWN;
        using gpiod::line_request::FLAG_BIAS_PULL_UP;
    };
    struct EventEdge : private gpiod::line_request {
    public:
        using gpiod::line_request::EVENT_FALLING_EDGE;
        using gpiod::line_request::EVENT_RISING_EDGE;
        using gpiod::line_request::EVENT_BOTH_EDGES;
    };

    static constexpr unsigned int UNDEFINED_GPIO { 256 };

    Gpio(const std::string& gpio_chip_devpath);
    ~Gpio();

    gpiod::chip& chip() { return fChip; }
    const gpiod::chip& chip() const { return fChip; }
    
    bool isInhibited() const { return inhibit; }

    typedef std::chrono::nanoseconds timestamp_t;
    typedef std::function<void(unsigned int, timestamp_t)> event_callback_t;

    void start();
    void stop();
    bool is_initialised();
    bool setPinInput(unsigned int gpio, std::bitset<32> flags = {});
    bool setPinOutput(unsigned int gpio, bool initState, std::bitset<32> flags = {});

    bool setPinBias(unsigned int gpio, std::bitset<32> bias_flags);
    bool setPinState(unsigned int gpio, bool state);
    bool getPinState(unsigned int gpio);
    bool registerInterrupt(unsigned int gpio, int edge, std::bitset<32> bias_flags);
    bool unRegisterInterrupt(unsigned int gpio);
    void setInhibited(bool inh = true) { inhibit = inh; }
    void set_event_callback(event_callback_t cb) { fEventCallback = cb; }
    
    auto set_gpio_direction(unsigned int gpio_pin, direction_t direction) -> bool;
    auto set_gpio_state(unsigned int gpio_pin, bool state) -> bool;
    auto get_gpio_state(unsigned int gpio_pin) -> bool;
    auto set_gpio_pullup(unsigned int gpio_pin, bool pullup_enable = true) -> bool;
    auto set_gpio_pulldown(unsigned int gpio_pin, bool pulldown_enable = true) -> bool;


private:
    void reloadInterruptSettings();
    [[gnu::hot]] void eventHandler(gpiod::line line);
    [[gnu::hot]] void processEvent(unsigned int gpio, gpiod::line_event event);

    bool inhibit { false };
    int verbose { 0 };
    gpiod::chip fChip {};
    std::map<unsigned int, gpiod::line> fInterruptLineMap {};
    std::map<unsigned int, gpiod::line> fLineMap {};
    bool fThreadRunning { false };
    std::map<unsigned int, std::unique_ptr<std::thread>> fThreads {};
    std::mutex fMutex;
    event_callback_t fEventCallback;
};

} // namespace PiRaTe
