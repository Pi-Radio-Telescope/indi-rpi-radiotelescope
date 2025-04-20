#include <iostream>
#include <stdio.h>
#include <chrono>

#include <unistd.h>

#include "gpioif.h"


#define DEFAULT_VERBOSITY 1

namespace PiRaTe {

constexpr char DEFAULT_GPIO_CONSUMER[] = "PiRaTe";
constexpr std::chrono::seconds LINE_EVENT_TIMEOUT { 10 };

Gpio::Gpio(const std::string& gpio_chip_devpath)
    : fChip(gpio_chip_devpath)
{
    if (!fChip) {
        std::cerr << "error opening gpio chip " << gpio_chip_devpath << "\n";
        throw std::exception();
    }
    std::cout << "opened " << gpio_chip_devpath << ": nlines=" << fChip.num_lines() << "\n";
}

Gpio::~Gpio()
{
    this->stop();
    for (auto [gpio, line] : fInterruptLineMap) {
        line.release();
    }
    for (auto [gpio, line] : fLineMap) {
        line.release();
    }
}

void Gpio::processEvent(unsigned int gpio, gpiod::line_event event)
{
    auto timestamp{ event.timestamp };
    
    if (fEventCallback) fEventCallback(gpio, timestamp);
    
    if (verbose > 3) {
        std::cout << "line event: gpio" << gpio << " edge: "
                 << std::string((event.event_type == gpiod::line_event::RISING_EDGE) ? "rising" : "falling")
                 << " ts=" << timestamp.count() << "ns\n";
    }
}

void Gpio::eventHandler(gpiod::line line)
{
    while (fThreadRunning) {
        if (inhibit) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        const unsigned int gpio { line.offset() };
        if ( line.event_wait(LINE_EVENT_TIMEOUT) ) {
            gpiod::line_event event { line.event_read() };
            std::thread process_event_bg(&Gpio::processEvent, this, gpio, std::move(event));
            process_event_bg.detach();
        } else {
            // a timeout occurred, no event was detected
            // simply go over into the wait loop again
        }
    }
}

bool Gpio::setPinInput(unsigned int gpio, std::bitset<32> flags)
{
    if (!is_initialised()) {
        std::cerr << "Gpio::setPinInput: chip not initialised\n";
        return false;
    }
    auto it = fLineMap.find(gpio);

    if (it != fLineMap.end()) {
        // line object exists, request for input
//         std::cerr << "Gpio::setPinInput: line " << gpio << " found in fLineMap\n";
        it->second.set_direction_input();
//         std::cerr << "Gpio::setPinInput: flags " << flags << "\n";
        it->second.set_flags(flags);
        return true;
    }
    // line was not allocated yet, so do it now
    gpiod::line line = fChip.get_line(gpio);
    // see if this line is already in use
    if (line.is_used()) {
        std::cerr << "Gpio::setPinInput: line " << gpio << " already in use\n";
        return false;
    }

    // request the line
    line.request( { 
        DEFAULT_GPIO_CONSUMER,
        gpiod::line_request::DIRECTION_INPUT,
        flags
    });
//     std::cerr << "Gpio::setPinInput: requested line " << gpio << " direction=" << gpiod::line_request::DIRECTION_INPUT << "\n";
    
    fLineMap.emplace(std::make_pair(gpio, std::move(line)));
    return true;
}

bool Gpio::setPinOutput(unsigned int gpio, bool initState, std::bitset<32> flags)
{
    if (!is_initialised())
        return false;
    auto it = fLineMap.find(gpio);

    if (it != fLineMap.end()) {
        // line object exists, request for output
        it->second.set_direction_output(static_cast<int>(initState));
        it->second.set_flags(flags);
        return true;
    }
    // line was not allocated yet, so do it now
    gpiod::line line = fChip.get_line(gpio);
    // see if this line is already in use
    if (line.is_used()) {
        std::cerr << "Gpio::setPinInput: line " << gpio << " already in use\n";
        return false;
    }

    // request the line
    line.request( { 
        DEFAULT_GPIO_CONSUMER,
        gpiod::line_request::DIRECTION_OUTPUT,
        flags
    });
//     std::cerr << "Gpio::setPinOutput: requested line " << gpio << " direction=" << gpiod::line_request::DIRECTION_OUTPUT << "\n";
    fLineMap.emplace(std::make_pair(gpio, std::move(line)));
    return true;
}

bool Gpio::setPinBias(unsigned int gpio, std::bitset<32> bias_flags)
{
    if (!is_initialised())
        return false;

    auto it = fLineMap.find(gpio);
    if (it != fLineMap.end()) {
        // line object exists, set config
        it->second.set_flags(bias_flags);
//         dir = gpiod_line_direction(it->second);
//         gpiod_line_release(it->second);
//         fLineMap.erase(it);
        return true;
    }
    // line was not allocated yet, so do it now
    gpiod::line line = fChip.get_line(gpio);
    // see if this line is already in use
    if (line.is_used()) {
        std::cerr << "Gpio::setPinInput: line " << gpio << " already in use\n";
        return false;
    }

    // request the line
    line.request( { 
        DEFAULT_GPIO_CONSUMER,
        gpiod::line_request::DIRECTION_AS_IS,
        bias_flags
    });
    fLineMap.emplace(std::make_pair(gpio, std::move(line)));
    return true;
}

bool Gpio::setPinState(unsigned int gpio, bool state)
{
    if (!is_initialised()) {
        std::cerr << "Gpio::setPinState: gpiochip not initialised\n";
        return false;
    }
    auto it = fLineMap.find(gpio);
    if (it != fLineMap.end()) {
        // line object exists, look if it an  output
        if (it->second.direction() != Direction::DIRECTION_OUTPUT) {
            std::cerr << "Gpio::setPinState: trying to set state of non-output line" << gpio << "\n";
            return false;
        }
    }
    // line was not allocated yet, so do it now
    return setPinOutput(gpio, state);
}

bool Gpio::getPinState(unsigned int gpio)
{
    if (!is_initialised())
        return false;
    auto it = fLineMap.find(gpio);
    if (it != fLineMap.end()) {
        // line object exists, look if it is an input
        if (it->second.direction() != Direction::DIRECTION_INPUT) {
            std::cerr << "Gpio::getPinState: trying to get state of non-input line" << gpio << "\n";
            return false;
        }
        return static_cast<bool>(it->second.get_value());
    }
    // line was not allocated yet, so do it now
    if (!setPinInput(gpio)) {
        std::cerr << "Gpio::getPinState: failed to allocate line" << gpio << "\n";
        return false;
    }
    return static_cast<bool>(fLineMap[gpio].get_value());
}

auto Gpio::set_gpio_direction(unsigned int gpio_pin, bool output) -> bool
{
    if (output) return setPinOutput(gpio_pin, 0);
    else return setPinInput(gpio_pin);
}

auto Gpio::set_gpio_state(unsigned int gpio_pin, bool state) -> bool
{
    return setPinState(gpio_pin, state);
}

auto Gpio::get_gpio_state(unsigned int gpio_pin) -> bool
{
    return getPinState(gpio_pin);
}

auto Gpio::set_gpio_pullup(unsigned int gpio_pin, bool pullup_enable) -> bool 
{
    if (pullup_enable) return setPinBias(gpio_pin, PinBias::FLAG_BIAS_PULL_UP);
    else return setPinBias(gpio_pin, PinBias::FLAG_BIAS_DISABLE);
}

auto Gpio::set_gpio_pulldown(unsigned int gpio_pin, bool pulldown_enable) -> bool
{
    if (pulldown_enable) return setPinBias(gpio_pin, PinBias::FLAG_BIAS_PULL_DOWN);
    else return setPinBias(gpio_pin, PinBias::FLAG_BIAS_DISABLE);
}

void Gpio::reloadInterruptSettings()
{
    this->stop();
    this->start();
}

bool Gpio::registerInterrupt(unsigned int gpio, int edge, std::bitset<32> bias_flags)
{
    if (!is_initialised())
        return false;
    auto it = fInterruptLineMap.find(gpio);
    if (it != fInterruptLineMap.end()) {
        // line object exists
        // The following code block shall release the previous line request
        // and re-request this line for events.
        // It is bypassed, since it does not work as intended.
        // The function simply does nothing in this case.
//         return false;
        it->second.release();
        it->second.update();
        it->second.request( { 
            DEFAULT_GPIO_CONSUMER,
            edge,
            bias_flags
        });
        reloadInterruptSettings();
        return true;
    }
    // line was not allocated yet, so do it now
    gpiod::line line = fChip.get_line(gpio);
    // see if this line is already in use
    if (line.is_used()) {
        std::cerr << "Gpio::registerInterrupt: line " << gpio << " already in use\n";
        return false;
    }

    // request the line
    line.request( { 
        DEFAULT_GPIO_CONSUMER,
        edge,
        bias_flags
    });
    fInterruptLineMap.emplace(std::make_pair(gpio, std::move(line)));
    
    reloadInterruptSettings();
    return true;
}

bool Gpio::unRegisterInterrupt(unsigned int gpio)
{
    if (!is_initialised())
        return false;
    auto it = fInterruptLineMap.find(gpio);
    if (it != fInterruptLineMap.end()) {
        it->second.release();
        fInterruptLineMap.erase(it);
        reloadInterruptSettings();
        return true;
    }
    return false;
}

bool Gpio::is_initialised()
{
    return (fChip.operator bool());
}

void Gpio::stop()
{
    fThreadRunning = false;
    for (auto& [gpio, line_thread] : fThreads) {
        if (line_thread)
            line_thread->join();
    }
}

void Gpio::start()
{
    if (fThreadRunning)
        return;
    fThreadRunning = true;

    for (auto& [gpio, line] : fInterruptLineMap) {
        fThreads[gpio] = std::make_unique<std::thread>([this, gpio, line]() { this->eventHandler(line); });
    }
}

} // namespace PiRaTe
