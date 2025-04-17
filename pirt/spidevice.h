#pragma once

#include <chrono>
#include <cinttypes> // std::uint8_t, etc
#include <iostream>
#include <string>

//namespace PiRaTe {

/**
* @brief The spi_device class
* This class defines an access interface to the hardware spi bus (master mode) using the linux sysfs interface.
* Basic data exchange is implemented, such as read, write and transfer of byte-aligned data blocks.
* @note The sysfs device files /dev/spidevX.Y must be existing and access rights for the user
* executing this code must be granted.
* @note Setting the word size is possible but the RPi hardware spi interface does not support
* word sizes other than 8 bit.
*/
class spi_device {
public:
    enum class Flags : std::uint8_t {
        None = 0,
        Normal = 0x01,
        Failed = 0x02,
        Locked = 0x04
    };

    using mode_t =  std::uint16_t;

    enum MODE : mode_t {
        CPHA=           0x01,
        CPOL=           0x02,
        MODE0=          0,
        MODE1=          CPHA,
        MODE2=          CPOL,
        MODE3=          CPHA | CPOL,
        CS_HIGH=        0x04,
        LSB_FIRST=      0x08,
        THREE_WIRE=     0x10,
        LOOP=           0x20,
        NO_CS=          0x40,
        READY=          0x80,
        TX_DUAL=        0x100,
        TX_QUAD=        0x200,
        RX_DUAL=        0x400,
        RX_QUAD=        0x800,
        CS_WORD=        0x1000,
        TX_OCTAL=       0x2000,
        RX_OCTAL=       0x4000,
        THREE_WIRE_HIZ= 0x8000
    };

    /**
     * @brief The config_t struct
     * This member struct of spi_device contains settings for the interface configuration.
     */
    struct config_t {
        mode_t mode; ///<! the mode bit pattern in which the interface shall be operated
        std::uint8_t bits_per_word; ///<! the number of bits per data word for one transfer
        std::uint32_t clk_rate; ///<! the bit rate of the spi clock (SCLK frequency) in Hertz
        std::uint16_t delay; ///<! the delay between the end of transfer until the CS deselect in us
    };

    /**
    * @brief constructor with specific device address path
    * @param device_path the system device address path of the spi bus.
    * @note On *ix systems this is usually /dev/spidevX.Y with X being the hardware interface and y the channel.
    */
    explicit spi_device(const std::string& device_path);

    virtual ~spi_device();

    /**
    * @brief check whether a device was opened for access
    * @return true, if the device was opened for access
    * @note A positive return value does not imply that an actual device is physically present.
    * The device was merely instantiated and opened for access.
    * Yet, the result of bus transactions is not reflected by this query.
    * Use @link spi_device#present (if implemented) to check for the physical presence of a device.
    */
    [[nodiscard]] auto is_open() const -> bool;
    /**
    * @brief close a device which was previously opened for access
    * @return true, if the spi master could successfully be released
    */
    void close() const;

    /**
    * @brief apply a specific configuration to the spi interface
    * @param config the configuration struct to be applied
    * @return true, if the configuration could be successfully written to the interface registers
    */
    [[nodiscard]] auto set_config( config_t config ) -> bool;

    /**
    * @brief get the currently set configuration of the spi interface
    * @return the config struct of the current spi interface configuration
    */
    [[nodiscard]] auto config() const -> config_t;

    /**
    * @brief check for the presence of a device on the bus
    * @return true, if a device is found
    * @note: this method may be overriden in derived classes, if there is a possibility to tell
    * the presence of a device from the bit battern read. If it is not overridden, this method returns false
    * by default.
    */
    [[nodiscard]] virtual auto present() -> bool;
    /**
    * @brief check for the presence of a specific device on the bus
    * @return true, if the device is found
    * @note: this method may be overriden in derived classes, if there is a possibility to tell
    * the presence of a specific device from the bit battern read. If it is not overridden,
    * this method returns false by default.
    */
    [[nodiscard]] virtual auto identify() -> bool;

    [[nodiscard]] auto io_errors() const -> std::size_t;

    /**
    * @brief the total number of bytes transferred through the spi interface
    * @return total number of bytes transferred
    */
    [[nodiscard]] auto transferred_bytes() const -> std::size_t;

    [[nodiscard]] auto flag_set(Flags flag) const -> bool;

    void lock(bool locked = true);
    [[nodiscard]] auto locked() const -> bool;

    /**
    * @brief get the transfer duration of the last measured access
    * @return the duration of last transfer
    */
    [[nodiscard]] auto last_access_duration() const -> std::chrono::microseconds;

    /**
    * @brief set the name of the spi device object
    * @param name the name string
    */
    void set_name(std::string name);

    /**
    * @brief get the name of the spi device object
    * @return the name string
    */
    [[nodiscard]] auto name() const -> std::string;

    /**
    * @brief apply a specific clock rate setting to the spi interface
    * @param clk_rate the bit rate to be set in Hertz
    */
    void set_clock_rate(unsigned int clk_rate);

    /**
    * @brief get the clock rate setting of the spi interface currently set
    * @return the bit rate in Hertz
    */
    [[nodiscard]] auto clock_rate() const -> unsigned int;

    /**
    * @brief read an array of bytes from the spi device
    * @param buffer pointer to the buffer in which the data shall be placed
    * @return true, if the read operation was successfull
    */
    [[nodiscard]] auto read(std::uint8_t* buffer, std::size_t n_bytes = 1) -> bool;

    /**
    * @brief read an array of data words from the spi device
    * @param buffer pointer to the buffer in which the data shall be placed
    * @return true, if the read operation was successfull
    * @note the bit width of the values placed in buffer is determined by the bits_per_word configuration setting
    */
    [[nodiscard]] auto read(std::uint16_t* buffer, std::size_t n_words = 1) -> bool;

    /**
    * @brief write an array of bytes to the spi device
    * @param buffer pointer to the buffer with the data to be written
    * @return true, if the write operation was successfull
    */
    [[nodiscard]] auto write(const std::uint8_t* buffer, std::size_t n_bytes = 1) -> bool;

    /**
    * @brief write an array of data words to the spi device
    * @param buffer pointer to the buffer with the data to be written
    * @return true, if the write operation was successfull
    * @note the bit width of the data words actually written is determined by the bits_per_word configuration setting
    */
    [[nodiscard]] auto write(const std::uint16_t* buffer, std::size_t n_words = 1) -> bool;

    /**
    * @brief transfer bytes to/from the spi device
    * @param tx_buffer pointer to the buffer with the data to be written
    * @param rx_buffer pointer to the buffer in which the data shall be placed
    * @return true, if the transfer operation was successfull
    */
    [[nodiscard]] auto transfer(std::uint8_t* tx_buffer, std::uint8_t* rx_buffer, std::size_t n_bytes = 1) -> bool;

    /**
    * @brief transfer data words to/from the spi device
    * @param tx_buffer pointer to the buffer with the data to be written
    * @param rx_buffer pointer to the buffer in which the data shall be placed
    * @return true, if the transfer operation was successfull
    * @note the bit width of the data words actually written and read is determined by the bits_per_word configuration setting
    */
    [[nodiscard]] auto transfer(std::uint16_t* tx_buffer, std::uint16_t* rx_buffer, std::size_t n_words = 1) -> bool;

protected:
    void set_flag(Flags flag);
    void unset_flag(Flags flag);

    void start_timer();
    void stop_timer();

private:

    static constexpr const char* s_default_device_path { "/dev/spidev0.0" };

    std::string m_device_path { s_default_device_path };

    int m_handle {};
    bool m_locked { false };

    std::size_t m_transferred_bytes {};

    std::size_t m_io_errors {};
    std::chrono::microseconds m_last_duration {}; // the last time measurement's result is stored here

    std::string m_name { "SPI device" };
    std::uint8_t m_flags { };
    config_t m_config { static_cast<mode_t>(MODE::MODE0), 8u, 1'000'000UL, 0 };

    std::chrono::system_clock::time_point m_start {};
};

//} // namespace PiRaTe
