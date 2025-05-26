#include "spidevice.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <fcntl.h> // open
#include <iostream>
#include <linux/spi/spidev.h> // SPI device definitions for linux like systems
#include <linux/types.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

namespace PiRaTe {

spi_device::spi_device(const std::string& device_path)
    : m_device_path { device_path }
    , m_handle { open(device_path.c_str(), O_RDWR) }
{
    if ( m_handle < 0 || !set_config( m_config ) ) {
        set_flag(Flags::Failed);
    } else {
    }
}

spi_device::~spi_device()
{
    close();
}

auto spi_device::is_open() const -> bool
{
    return m_handle > 0;
}

void spi_device::close() const
{
    if ( is_open() ) {
        ::close(m_handle);
    }
}

auto spi_device::set_config( config_t config ) -> bool
{
    if ( !is_open() ) {
        std::cerr<<"spi_device::set_config: device not open\n";
        return false;
    }
    std::uint32_t spi_mode { config.mode };
    if ( ioctl( m_handle, SPI_IOC_WR_MODE32, &spi_mode ) < 0 )
    {
        std::uint8_t spi_mode_8b = spi_mode && 0xff;
        if ( ioctl( m_handle, SPI_IOC_WR_MODE, &spi_mode_8b ) < 0 )
        {
            std::cerr<<"spi_device::set_config: ioctl( m_handle, SPI_IOC_WR_MODE, &spi_mode_8b ) failed\n";
            return false;
        }    
    }
    m_config.mode = config.mode;

    if ( ioctl( m_handle, SPI_IOC_WR_BITS_PER_WORD, &config.bits_per_word ) < 0 )
    {
        return false;
        std::cerr<<"spi_device::set_config: ioctl( m_handle, SPI_IOC_WR_BITS_PER_WORD, &config.bits_per_word ) failed\n";
    }
    m_config.bits_per_word = config.bits_per_word;

    if ( ioctl( m_handle, SPI_IOC_WR_MAX_SPEED_HZ, &config.clk_rate ) < 0 )
    {
        std::cerr<<"spi_device::set_config: ioctl( m_handle, SPI_IOC_WR_MAX_SPEED_HZ, &config.clk_rate ) failed\n";
        return false;
    }
    m_config.clk_rate = config.clk_rate;
    return true;
}

auto spi_device::config() const -> config_t
{
    return m_config;
}

auto spi_device::present() -> bool
{
    return false;
}

auto spi_device::identify() -> bool
{
    return false;
}

auto spi_device::io_errors() const -> std::size_t
{
    return m_io_errors;
}

auto spi_device::transferred_bytes() const -> std::size_t
{
    return m_transferred_bytes;
}

auto spi_device::flag_set(Flags flag) const -> bool
{
    return (m_flags & static_cast<std::uint8_t>(flag)) > 0;
}

void spi_device::set_flag(Flags flag)
{
    m_flags |= static_cast<std::uint8_t>(flag);
}

void spi_device::unset_flag(Flags flag)
{
    m_flags &= ~static_cast<std::uint8_t>(flag); // NOLINT(hicpp-signed-bitwise)
}

void spi_device::lock(bool locked)
{
    m_locked = locked;
}

auto spi_device::locked() const -> bool
{
    return m_locked;
}

auto spi_device::last_access_duration() const -> std::chrono::microseconds
{
    return m_last_duration;
}

void spi_device::set_name(std::string name)
{
    m_name = std::move(name);
}

auto spi_device::name() const -> std::string
{
    return m_name;
}

void spi_device::set_clock_rate(unsigned int clk_rate)
{
    m_config.clk_rate = clk_rate;
}

auto spi_device::clock_rate() const -> unsigned int
{
    return m_config.clk_rate;
}

// template <typename T>
// auto general_transfer(T* tx, T* rx, std::size_t words) -> bool {
//     constexpr std::size_t bits { sizeof(T) * 8U };
//     spi_ioc_transfer spi {};
// 
//     spi.tx_buf          = reinterpret_cast<std::uint64_t>(tx);
//     spi.rx_buf          = reinterpret_cast<std::uint64_t>(rx);
//     spi.len             = words;
//     spi.speed_hz        = m_config.clk_rate;
//     spi.delay_usecs     = 0;
//     spi.bits_per_word   = bits;
//     spi.cs_change       = 0;
//     
//     return( (ioctl(m_handle, SPI_IOC_MESSAGE(1), &spi) > 0) && (m_transferred_bytes += sizeof(T)) );
// }

auto spi_device::read(std::uint8_t* buffer, std::size_t n_bytes) -> bool
{
    if (locked() || !is_open()) {
        return false;
    }

    spi_ioc_transfer spi{};

    memset(&spi, 0, sizeof(spi));

    // for definition of spi_ioc_transfer struct refer to
    // https://elixir.bootlin.com/linux/v6.0.19/source/include/uapi/linux/spi/spidev.h
    spi.tx_buf        = reinterpret_cast<__u64>( nullptr );
    spi.rx_buf        = reinterpret_cast<__u64>( buffer );
    spi.len           = n_bytes;
    spi.speed_hz      = m_config.clk_rate;
    spi.delay_usecs   = 0;
    spi.bits_per_word = 8;
    spi.cs_change     = 0;

    return( (ioctl(m_handle, SPI_IOC_MESSAGE(1), &spi) > 0) && (m_transferred_bytes += n_bytes) );
}

auto spi_device::read(std::uint16_t* buffer, std::size_t n_words) -> bool
{
    if (locked() || !is_open()) {
        return false;
    }

    spi_ioc_transfer spi{};

    memset(&spi, 0, sizeof(spi));

    spi.tx_buf        = reinterpret_cast<__u64>( nullptr );
    spi.rx_buf        = reinterpret_cast<__u64>( buffer );
    spi.len           = n_words*2;
    spi.speed_hz      = m_config.clk_rate;
    spi.delay_usecs   = 0;
    spi.bits_per_word = m_config.bits_per_word;
    spi.cs_change     = 0;

    return( (ioctl(m_handle, SPI_IOC_MESSAGE(1), &spi) > 0) && (m_transferred_bytes += n_words*2) );
}



auto spi_device::write(const std::uint8_t* buffer, std::size_t n_bytes) -> bool
{
    if (locked() || !is_open()) {
        return false;
    }

    spi_ioc_transfer spi{};

    spi.tx_buf        = reinterpret_cast<__u64>( buffer );
    spi.rx_buf        = reinterpret_cast<__u64>( nullptr );
    spi.len           = n_bytes;
    spi.speed_hz      = m_config.clk_rate;
    spi.delay_usecs   = 0;
    spi.bits_per_word = 8;
    spi.cs_change     = 0;

    return( (ioctl(m_handle, SPI_IOC_MESSAGE(1), &spi) > 0) && (m_transferred_bytes += n_bytes) );
}

auto spi_device::write(const std::uint16_t* buffer, std::size_t n_words) -> bool
{
    if (locked() || !is_open()) {
        return false;
    }

    spi_ioc_transfer spi{};

    memset(&spi, 0, sizeof(spi));

    spi.tx_buf        = reinterpret_cast<__u64>( buffer );
    spi.rx_buf        = reinterpret_cast<__u64>( nullptr );
    spi.len           = n_words;
    spi.speed_hz      = m_config.clk_rate;
    spi.delay_usecs   = 0;
    spi.bits_per_word = m_config.bits_per_word;
    spi.cs_change     = 0;

    return( (ioctl(m_handle, SPI_IOC_MESSAGE(1), &spi) > 0) && (m_transferred_bytes += n_words*2) );
}

auto spi_device::transfer(std::uint8_t* tx_buffer, std::uint8_t* rx_buffer, std::size_t n_bytes) -> bool
{
    if (locked() || !is_open()) {
        return false;
    }

    spi_ioc_transfer spi{};

    memset(&spi, 0, sizeof(spi));

    spi.tx_buf        = reinterpret_cast<__u64>( tx_buffer );
    spi.rx_buf        = reinterpret_cast<__u64>( rx_buffer );
    spi.len           = n_bytes;
    spi.speed_hz      = m_config.clk_rate;
    spi.delay_usecs   = 0;
    spi.bits_per_word = 8;
    spi.cs_change     = 0;

    return( (ioctl(m_handle, SPI_IOC_MESSAGE(1), &spi) > 0) && (m_transferred_bytes += n_bytes) );
}

auto spi_device::transfer(std::uint16_t* tx_buffer, std::uint16_t* rx_buffer, std::size_t n_words) -> bool
{
    if (locked() || !is_open()) {
        return false;
    }

    spi_ioc_transfer spi{};

    memset(&spi, 0, sizeof(spi));

    spi.tx_buf        = reinterpret_cast<__u64>( tx_buffer );
    spi.rx_buf        = reinterpret_cast<__u64>( rx_buffer );
    spi.len           = n_words;
    spi.speed_hz      = m_config.clk_rate;
    spi.delay_usecs   = 0;
    spi.bits_per_word = m_config.bits_per_word;
    spi.cs_change     = 0;

    return( (ioctl(m_handle, SPI_IOC_MESSAGE(1), &spi) > 0) && (m_transferred_bytes += n_words*2) );
}

void spi_device::start_timer()
{
    m_start = std::chrono::system_clock::now();
}

void spi_device::stop_timer()
{
    m_last_duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - m_start);
}

} // namespace PiRaTe
