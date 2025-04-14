#ifndef ENCODERSRT_H
#define ENCODERSRT_H

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

namespace PiRaTe {

constexpr unsigned int SPI_BAUD_DEFAULT { 500000U };
constexpr unsigned int MAX_CONN_ERRORS { 10U };

/**
 * @brief Interface class for reading out SSI-interface based positional encoders.
 * This class manages the read-out of absolute position encoders connected to the SPI interface.
 * The interface to be utilized is set in the constructor call together with baud rate and SPI mode settings. 
 * The absolute position is obtained with {@link SsiPosEncoder::absolutePosition} with the return value in evolutions.
 * @note The class launches a separate thread loop upon successfull construction which reads the encoder's data word every 10 ms.
 * @author HG Zaunick
 */
class SsiPosEncoder {
public:
    SsiPosEncoder() = delete;
    /**
	 * @brief The main constructor.
	 * Initializes an object with the given gpio object pointer and SPI parameters.
	 * @param gpio shared pointer to an initialized GPIO object
	 * @param GPIO::SPI_INTERFACE the SPI interface to use (GPIO::SPI_INTERFACE::Main or GPIO::SPI_INTERFACE:Aux)
	 * @param baudrate the bitrate which the SPI interface should be initialized for
	 * @param spi_channel the SPI channel to use (which CEx pin is assigned to this SPI device)
	 * @param GPIO::SPI_MODE the SPI mode to use (GPIO::SPI_MODE::POL0PHA0, GPIO::SPI_MODE::POL0PHA1, GPIO::SPI_MODE::POL1PHA0 or GPIO::SPI_MODE::POL1PHA1)
	 * @throws std::exception if the supplied gpio object is not initialized or the initialization of the SPI channel fails
	 */
    SsiPosEncoder(std::shared_ptr<GPIO> gpio,
        GPIO::SPI_INTERFACE spi_interface,
        unsigned int baudrate = SPI_BAUD_DEFAULT,
        std::uint8_t spi_channel = 0,
        GPIO::SPI_MODE spi_mode = GPIO::SPI_MODE::POL1PHA1);
    ~SsiPosEncoder();

    [[nodiscard]] auto isInitialized() const -> bool { return (fSpiHandle >= 0); }

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

    int fSpiHandle { -1 };
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
    std::shared_ptr<GPIO> fGpio { nullptr };

    std::mutex fMutex;
};

} // namespace PiRaTe

#endif
