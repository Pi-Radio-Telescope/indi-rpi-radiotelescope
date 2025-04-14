#include "linux/i2c-dev.h" // I2C bus definitions for linux like systems
#include <fcntl.h> // open
#include <inttypes.h> // uint8_t, etc
#include <iostream>
#include <stdio.h>
#include <string>
#include <sys/ioctl.h> // ioctl
#include <sys/time.h> // for gettimeofday()
#include <unistd.h>
#include <vector>

#ifndef _I2CDEVICE_H_
#define _I2CDEVICE_H_

#define DEFAULT_DEBUG_LEVEL 0

/**
 * @brief I2C interface unix linux kernel character device access.
 * This class provides general access to the linux kernel i2c device with all basic
 * access functions.
 * For device specific implementations, classes can inherit this base class. 
 * Virtual methods should be reimplemented in the child class to make sense there,
 * such as devicePresent().
*/
class i2cDevice {

public:
    enum MODE { MODE_NONE = 0,
        MODE_NORMAL = 0x01,
        MODE_FORCE = 0x02,
        MODE_UNREACHABLE = 0x04,
        MODE_FAILED = 0x08,
        MODE_LOCKED = 0x10 };

    //using I2C_DEVICE_MODE;

    i2cDevice();
    i2cDevice(const char* busAddress);
    i2cDevice(uint8_t slaveAddress);
    i2cDevice(const char* busAddress, uint8_t slaveAddress);
    virtual ~i2cDevice();

    void setAddress(uint8_t address);
    uint8_t getAddress() const { return fAddress; }
    static unsigned int getNrDevices() { return fNrDevices; }
    unsigned int getNrBytesRead() const { return fNrBytesRead; }
    unsigned int getNrBytesWritten() const { return fNrBytesWritten; }
    unsigned int getNrIOErrors() const { return fIOErrors; }
    static unsigned int getGlobalNrBytesRead() { return fGlobalNrBytesRead; }
    static unsigned int getGlobalNrBytesWritten() { return fGlobalNrBytesWritten; }
    static std::vector<i2cDevice*>& getGlobalDeviceList() { return fGlobalDeviceList; }
    static i2cDevice* findDevice(uint8_t slaveAddress);
    virtual bool devicePresent();
    uint8_t getStatus() const { return fMode; }
    void lock(bool locked = true);

    double getLastTimeInterval() const { return fLastTimeInterval; }

    void setDebugLevel(int level) { fDebugLevel = level; }
    int getDebugLevel() const { return fDebugLevel; }

    void setTitle(const std::string& a_title) { fTitle = a_title; }
    const std::string& getTitle() const { return fTitle; }

    /** @brief Read multiple bytes from device into buffer.
     * @param buf Buffer to store the data read from the device
     * @param nBytes Number of bytes to be read
     * @return Number of bytes actually read during the operation (<=0 on failure)
     */
    int read(uint8_t* buf, int nBytes);

    /** @brief Write multiple bytes from buffer to device.
     * @param buf Buffer to store the data to write to the device
     * @param nBytes Number of bytes to be written
     * @return Number of bytes written during the operation (<=0 on failure)
     */
    int write(uint8_t* buf, int nBytes);

    /** @brief Write multiple bytes from buffer to device at register address.
     * @param reg Register address to read from
     * @param buf Buffer to store the data to write to the device
     * @param nBytes Number of bytes to be written
     * @return Number of bytes written during the operation (<=0 on failure)
     * @note First byte of the write sequence is the register address, 
     * the following bytes from buf are then written in a sequence
     */
    int writeReg(uint8_t reg, uint8_t* buf, int nBytes);

    /** @brief Read multiple bytes into buffer from register address.
     * First writes reg address and after a repeated start reads in a sequence of bytes.
     * @param reg Register address to read from
     * @param buf Buffer to store the data read from the device
     * @param nBytes Number of bytes to be read during the operation
     * @return Number of bytes actually read (<=0 on failure)
     * @note Not all devices support this procedure. Refer to the device's datasheet.
     */
    int readReg(uint8_t reg, uint8_t* buf, int nBytes);

    /** @brief Read a single bit from an 8-bit device register.
	* @param regAddr Register regAddr to read from
	* @param bitNum Bit position to read (0-7)
	* @param data Container for single bit value
	* @return Status of read operation (true = success)
	*/
    int8_t readBit(uint8_t regAddr, uint8_t bitNum, uint8_t* data);

    /** Read multiple bits from an 8-bit device register.
	* @param regAddr Register regAddr to read from
	* @param bitStart First bit position to read (0-7)
	* @param length Number of bits to read (not more than 8)
	* @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
	* @return Status of read operation (true = success)
	*/
    int8_t readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data);

    /** Read single byte from an 8-bit device register.
	* @param regAddr Register regAddr to read from
	* @param data Container for byte value read from device
	* @return Status of read operation (true = success)
	*/
    bool readByte(uint8_t regAddr, uint8_t* data);

    /** Read multiple bytes from an 8-bit device register.
	* @param regAddr First register regAddr to read from
	* @param length Number of bytes to read
	* @param data Buffer to store read data in
	* @return Number of bytes read (-1 indicates failure)
	*/
    int16_t readBytes(uint8_t regAddr, uint16_t length, uint8_t* data);

    /** write a single bit in an 8-bit device register.
	* @param regAddr Register regAddr to write to
	* @param bitNum Bit position to write (0-7)
	* @param data New bit value to write
	* @return Status of operation (true = success)
	*/
    bool writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);

    /** Write multiple bits in an 8-bit device register.
	* @param regAddr Register regAddr to write to
	* @param bitStart First bit position to write (0-7)
	* @param length Number of bits to write (not more than 8)
	* @param data Right-aligned value to write
	* @return Status of operation (true = success)
	*/
    bool writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);

    /** Write single byte to an 8-bit device register.
	* @param regAddr Register address to write to
	* @param data New byte value to write
	* @return Status of operation (true = success)
	*/
    bool writeByte(uint8_t regAddr, uint8_t data);

    /** Write multiple bytes to an 8-bit device register.
	* @param regAddr First register address to write to
	* @param length Number of bytes to write
	* @param data Buffer to copy new data from
	* @return Status of operation (true = success)
	*/
    bool writeBytes(uint8_t regAddr, uint16_t length, uint8_t* data);

    /** Write multiple words to a 16-bit device register.
	* @param regAddr First register address to write to
	* @param length Number of words to write
	* @param data Buffer to copy new data from
	* @return Status of operation (true = success)
	*/
    bool writeWords(uint8_t regAddr, uint16_t length, uint16_t* data);

    /** Write single word to a 16-bit device register.
	* @param regAddr Register address to write to
	* @param data New word value to write
	* @return Status of operation (true = success)
	*/
    bool writeWord(uint8_t regAddr, uint16_t data);

    void getCapabilities();

protected:
    int fHandle;
    uint8_t fAddress;
    static unsigned int fNrDevices;
    unsigned long int fNrBytesWritten;
    unsigned long int fNrBytesRead;
    static unsigned long int fGlobalNrBytesRead;
    static unsigned long int fGlobalNrBytesWritten;
    double fLastTimeInterval; ///< the last time measurement's result is stored here
    struct timeval fT1, fT2;
    int fDebugLevel;
    static std::vector<i2cDevice*> fGlobalDeviceList;
    std::string fTitle = "I2C device";
    uint8_t fMode = MODE_NONE;
    unsigned int fIOErrors = 0;

    // functions for measuring time intervals
    void startTimer();
    void stopTimer();
};

#endif // _I2CDEVICE_H_
