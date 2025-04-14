#include <iostream>
#include <stdio.h>

#include <unistd.h>

#include "gpioif.h"

extern "C" {
#include <pigpiod_if2.h>
}

#define DEFAULT_VERBOSITY 1

// namespace PiRaTe {

GPIO::GPIO(const std::string& host, const std::string& port)
{
    if (fHandle < 0) {
        char* addrStr = const_cast<char*>(host.c_str());
        char* portStr = const_cast<char*>(port.c_str());
        //		fHandle = pigpio_start((char*)"127.0.0.1", (char*)"8888");
        fHandle = pigpio_start(addrStr, portStr);
        if (fHandle < 0) {
            std::cerr << "Could not connect to pigpio daemon. Is pigpiod running?\n";
            return;
        }
    }
}

GPIO::~GPIO()
{
    if (fHandle >= 0) {
        pigpio_stop(fHandle);
    }
    fHandle = -1;
}

auto GPIO::spi_init(SPI_INTERFACE interface, std::uint8_t channel, SPI_MODE mode, unsigned int baudrate, bool lsb_first, bool use_cs) -> int
{
    unsigned int spi_flags = static_cast<unsigned int>(mode) | (static_cast<unsigned int>(lsb_first) << 15);
    if (interface == SPI_INTERFACE::Aux) {
        spi_flags |= 1 << 8;
        //std::cout<<"spi flags: "<<spi_flags<<"\n";
    }
    if (!use_cs) {
        spi_flags |= 0b111 << 5;
    }
    int handle = ::spi_open(fHandle, channel, baudrate, spi_flags);
    if (handle < 0) {
        std::cerr << "Error opening spi interface.\n";
    }
    return handle;
}

auto GPIO::spi_read(unsigned int spi_handle, unsigned int nBytes) -> std::vector<std::uint8_t>
{
    char rx_buffer[nBytes];
    std::lock_guard<std::mutex> guard(fMutex);
    int count = ::spi_read(fHandle, spi_handle, rx_buffer, nBytes);
    if (count <= 0)
        return std::vector<std::uint8_t> {};
    std::vector<std::uint8_t> data(rx_buffer, rx_buffer + std::min(static_cast<unsigned int>(count), nBytes));
    return data;
}

auto GPIO::spi_write(unsigned int spi_handle, const std::vector<std::uint8_t>& data) -> bool
{
    unsigned int nBytes = data.size();
    char tx_buffer[nBytes];
    std::copy(data.begin(), data.end(), tx_buffer);
    std::lock_guard<std::mutex> guard(fMutex);
    int count = ::spi_write(fHandle, spi_handle, tx_buffer, nBytes);
    return (count == static_cast<int>(nBytes));
}

void GPIO::spi_close(int spi_handle)
{
    ::spi_close(fHandle, spi_handle);
}

auto GPIO::pwm_set_frequency(unsigned int gpio_pin, unsigned int freq) -> bool
{
    int res = ::set_PWM_frequency(fHandle, gpio_pin, freq);
    return (res >= 0);
}

auto GPIO::pwm_set_range(unsigned int gpio_pin, unsigned int range) -> bool
{
    int res = ::set_PWM_range(fHandle, gpio_pin, range);
    return (res == 0);
}

auto GPIO::pwm_set_value(unsigned int gpio_pin, unsigned int value) -> bool
{
    int res = ::set_PWM_dutycycle(fHandle, gpio_pin, value);
    return (res == 0);
}

void GPIO::pwm_off(unsigned int gpio_pin)
{
    ::set_PWM_dutycycle(fHandle, gpio_pin, 0);
}

auto GPIO::hw_pwm_set_value(unsigned int gpio_pin, unsigned int freq, std::uint32_t value) -> bool
{
    int res = ::hardware_PWM(fHandle, gpio_pin, freq, value);
    return (res == 0);
}

auto GPIO::set_gpio_direction(unsigned int gpio_pin, bool output) -> bool
{
    int res = ::set_mode(fHandle, gpio_pin, (output) ? PI_OUTPUT : PI_INPUT);
    return (res == 0);
}

auto GPIO::set_gpio_state(unsigned int gpio_pin, bool state) -> bool
{
    int res = ::gpio_write(fHandle, gpio_pin, (state) ? 1U : 0U);
    return (res == 0);
}

auto GPIO::get_gpio_state(unsigned int gpio_pin, bool* err) -> bool
{
    int res = ::gpio_read(fHandle, gpio_pin);
    if (err != nullptr)
        *err = (res < 0);
    return (res > 0);
}

auto GPIO::set_gpio_pullup(unsigned int gpio_pin, bool pullup_enable) -> bool
{
    int res = ::set_pull_up_down(fHandle, gpio_pin, (pullup_enable) ? PI_PUD_UP : PI_PUD_OFF);
    return (res == 0);
}

auto GPIO::set_gpio_pulldown(unsigned int gpio_pin, bool pulldown_enable) -> bool
{
    int res = ::set_pull_up_down(fHandle, gpio_pin, (pulldown_enable) ? PI_PUD_DOWN : PI_PUD_OFF);
    return (res == 0);
}

//} // namespace PiRaTe
