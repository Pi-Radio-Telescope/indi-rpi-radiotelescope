#include <chrono>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <string>
#include <unistd.h>

#include "encoder.h"
#include "utility.h"
#include "spidevice.h"

#define DEFAULT_VERBOSITY 1

namespace PiRaTe {

unsigned int SsiPosEncoder::fNrInstances = 0;
constexpr std::chrono::milliseconds loop_delay { 50 };
constexpr double MAX_TURNS_PER_SECOND { 10. };

auto SsiPosEncoder::intToBinaryString(unsigned long number) -> std::string
{
    std::string numStr {};
    for (int i = 31; i >= 0; i--) {
        numStr += (number & (1 << i)) ? "1" : "0";
    }
    return numStr;
}

auto SsiPosEncoder::gray_decode(std::uint32_t g) -> std::uint32_t
{
    for (std::uint32_t bit = 1U << 31; bit > 1; bit >>= 1) {
        if (g & bit)
            g ^= bit >> 1;
    }
    return g;
}

SsiPosEncoder::SsiPosEncoder(const std::string& spidev_path, unsigned int baudrate, spi_device::mode_t spi_mode)
    : fSpi { std::make_unique<spi_device>(spidev_path) }
{
    if (fSpi == nullptr) {
        std::cerr << "Error: no valid SPI instance.\n";
        throw std::exception();
    }

    struct spi_device::config_t spi_config { spi_mode | spi_device::MODE::NO_CS, 8, baudrate, 0 };
    if (!fSpi->set_config(spi_config)) {
        std::cerr << "Error: no valid SPI configuration.\n";
        throw std::exception();
    }

    fActiveLoop = true;
    // since C++14 using std::make_unique
    // fThread = std::make_unique<std::thread>( [this]() { this->readLoop(); } );
    // C++11 is unfortunately more unconvenient with move from a locally generated pointer
    //std::unique_ptr<std::thread> thread( new std::thread( [this]() { this->readLoop(); } ));
    //fThread = std::move(thread);
    // or with the reset method of smart pointers
    fThread.reset(new std::thread([this]() { this->readLoop(); }));
    fNrInstances++;
}

SsiPosEncoder::~SsiPosEncoder()
{
    fActiveLoop = false;
    if (fThread != nullptr)
        fThread->join();
    fNrInstances--;
    // close SPI device
    if (fSpi != nullptr)
        fSpi->close();
}

// this is the background thread loop
void SsiPosEncoder::readLoop()
{
    bool errorFlag { true };
    auto lastReadOutTime = std::chrono::system_clock::now();
    while (fActiveLoop) {
        std::uint32_t data { 0 };
        auto currentReadOutTime = std::chrono::system_clock::now();
        bool ok = readDataWord(data);
        if (!ok) {
            errorFlag = true;
            if (fConErrorCountdown)
                fConErrorCountdown--;
        } else {
            auto readOutDuration { std::chrono::system_clock::now() - std::chrono::system_clock::time_point { currentReadOutTime } };
            fConErrorCountdown++;
            // check if MSB is 1
            // this should always be the case
            // comment out, if your encoder behaves differently
            if (!(data & (1 << 31))) {
                fBitErrors++;
                errorFlag = true;
                lastReadOutTime = currentReadOutTime;
                std::this_thread::sleep_for(loop_delay);
                continue;
            }
            //			std::cout<<" raw: "<<intToBinaryString(data)<<"\n";
            std::uint32_t temp = data >> (32 - fStBits - fMtBits - 1);
            temp &= (1 << (fStBits + fMtBits - 1)) - 1;
            temp = gray_decode(temp);
            std::uint32_t st = temp & ((1 << (fStBits)) - 1);
            //			std::cout<<" st: "<<intToBinaryString(st)<<"\n";

            std::int32_t mt = (temp >> fStBits) & ((1 << (fMtBits)) - 1);
            //			std::cout<<" mt: "<<intToBinaryString(mt)<<"\n";

            // add sign bit to MT value
            // negative counts have to be offset by -1. Otherwise one had to
            // distinguish between -0 and +0 rotations
            if (data & (1 << 30))
                mt = -mt - 1;
            //std::cout<<" MT="<<mt;

            if (errorFlag) {
                fLastPos = st;
                fLastTurns = mt;
                lastReadOutTime = currentReadOutTime;
                std::this_thread::sleep_for(loop_delay);
                errorFlag = false;
                continue;
            }

            int turnDiff = mt - fLastTurns;
            if (std::abs(turnDiff) > 1) {
                //std::cout<<" st diff: "<<posDiff<<"\n";
                fBitErrors++;
                errorFlag = true;
                lastReadOutTime = currentReadOutTime;
                std::this_thread::sleep_for(loop_delay);
                continue;
            }

            int posDiff = st - fLastPos;

            if (std::abs(posDiff) > (1 << (fStBits - 1))) {
                posDiff -= sgn(posDiff) * (1 << (fStBits));
            }
            double speed = static_cast<double>(posDiff) / (1 << fStBits);
            auto diffTime { currentReadOutTime - lastReadOutTime };

            speed *= 1000. / std::chrono::duration_cast<std::chrono::milliseconds>(diffTime).count();
            if (std::abs(speed) > MAX_TURNS_PER_SECOND) {
                fBitErrors++;
                errorFlag = true;
                lastReadOutTime = currentReadOutTime;
                std::this_thread::sleep_for(loop_delay);
                continue;
            }

            speed *= 360.;

            fLastPos = st;
            fLastTurns = mt;

            fMutex.lock();
            fPos = st;
            fTurns = mt;
            fCurrentSpeed = speed;
            fUpdated = true;
            fReadOutDuration = std::chrono::duration_cast<std::chrono::microseconds>(readOutDuration);
            fMutex.unlock();
            lastReadOutTime = currentReadOutTime;
        }
        if (fConErrorCountdown > MAX_CONN_ERRORS)
            fConErrorCountdown = MAX_CONN_ERRORS;
        std::this_thread::sleep_for(loop_delay);
    }
}

auto SsiPosEncoder::readDataWord(std::uint32_t& data) -> bool
{
    if (fSpi == nullptr)
        return false;
    constexpr unsigned int nBytes = 4;
    std::array<std::uint8_t, nBytes> buffer { 0, 0, 0, 0 };
    if (fSpi->is_open()) {
        [[maybe_unused]] auto opresult = fSpi->read(buffer.data(), nBytes);
    } else return false;
    data = static_cast<std::uint32_t>(buffer[3]) 
        | (static_cast<std::uint32_t>(buffer[2]) << 8) 
        | (static_cast<std::uint32_t>(buffer[1]) << 16) 
        | (static_cast<std::uint32_t>(buffer[0]) << 24);
    return true;
}

auto SsiPosEncoder::absolutePosition() -> double
{
    fUpdated = false;
    double pos = static_cast<double>(fPos) / (1 << fStBits);
    if (fTurns < 0) {
        pos = 1. - pos;
    }
    pos += static_cast<double>(fTurns);
    return pos;
}

auto SsiPosEncoder::statusOk() const -> bool
{
    if (!fActiveLoop)
        return false;
    return (fConErrorCountdown > 0);
}

} // namespace PiRaTe
