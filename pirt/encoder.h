#pragma once

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

#include "spidevice.h"

namespace PiRaTe {

constexpr unsigned int SPI_BAUD_DEFAULT { 500'000U };
constexpr unsigned int MAX_CONN_ERRORS { 10U };

/**
 * @brief Interface class for reading out SSI-interface based positional encoders.
 * This class manages the read-out of absolute position encoders connected to the SPI interface.
 * The interface to be utilized is set in the constructor call together with baud rate and SPI mode settings. 
 * The absolute position is obtained with {@link SsiPosEncoder::absolutePosition} with the return value in evolutions.
* @note In order to use two SPI channnels on RPi4 for the encoder connections of both axes
* following steps are required in preparation:
* * configure the boot overlay with:
*  * dtparam=spi=off
*  * dtoverlay=spi0-cs,cs0_pin=46,cs1_pin=47
*  * dtoverlay=spi6-1cs,cs0_pin=48
* * Instantiate the two encoders with the spidev paths /dev/spidev0.0 and /dev/spidev6.0
* * The encoders i tested work in SPI mode 3 only (CPHA=1 and CPOL=1), 
*   so make sure the spi_mode param is set accordingly
 * @note The class launches a separate thread loop upon successfull construction which reads the encoder's data word every 10 ms.
 * @author HG Zaunick
 */
class SsiPosEncoder {
public:
    SsiPosEncoder() = delete;
    /**
    * @brief The main constructor.
    * Initializes an object with the given gpio object pointer and SPI parameters.
    * @param spidev_path path to spidev device, e.g. "/dev/spidev0.0
    * @param baudrate the bitrate which the SPI interface should be initialized for
    * @param spi_mode the SPI mode to use (spi_device::MODE::MODE0 etc.)
    * @throws std::exception if the initialization of the SPI channel fails
    */
    SsiPosEncoder(const std::string& spidev_path,
        unsigned int baudrate = SPI_BAUD_DEFAULT,
        spi_device::mode_t spi_mode = spi_device::MODE::MODE3);
    ~SsiPosEncoder();

    [[nodiscard]] auto isInitialized() const -> bool { return (fSpi && fSpi->is_open()); }

    [[nodiscard]] auto position() -> unsigned int
    {
        fUpdated = false;
        return fPos;
    }
    [[nodiscard]] auto nrTurns() -> int
    {
        fUpdated = false;
        return fTurns;
    }
    [[nodiscard]] auto absolutePosition() -> double;

    [[nodiscard]] auto isUpdated() const -> bool { return fUpdated; }
    void setStBitWidth(std::uint8_t st_bits) { fStBits = st_bits; }
    void setMtBitWidth(std::uint8_t mt_bits) { fMtBits = mt_bits; }
    [[nodiscard]] auto bitErrorCount() const -> unsigned long { return fBitErrors; }
    [[nodiscard]] auto currentSpeed() const -> double { return fCurrentSpeed; }
    [[nodiscard]] auto lastReadOutDuration() const -> std::chrono::duration<int, std::micro> { return fReadOutDuration; }
    [[nodiscard]] auto statusOk() const -> bool;

private:
    void readLoop();
    auto readDataWord(std::uint32_t& data) -> bool;
    [[nodiscard]] auto gray_decode(std::uint32_t g) -> std::uint32_t;
    [[nodiscard]] auto intToBinaryString(unsigned long number) -> std::string;

    std::uint8_t fStBits { 12 };
    std::uint8_t fMtBits { 12 };
    unsigned int fPos { 0 };
    int fTurns { 0 };
    unsigned int fLastPos { 0 };
    unsigned int fLastTurns { 0 };
    unsigned long fBitErrors { 0 };
    double fCurrentSpeed { 0. };
    std::chrono::duration<int, std::micro> fReadOutDuration {};

    bool fUpdated { false };
    bool fActiveLoop { false };
    unsigned int fConErrorCountdown { MAX_CONN_ERRORS };

    static unsigned int fNrInstances;
    std::unique_ptr<std::thread> fThread { nullptr };
    std::unique_ptr<spi_device> fSpi { nullptr };

    std::mutex fMutex;
};

} // namespace PiRaTe
