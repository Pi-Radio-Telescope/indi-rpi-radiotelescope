/*
   INDI Raspberry Pi based mount driver.
   The driver itself acts as a telescope mount reading the positions from SSI-based absolute encoders from
   the on-board SPI interfaces and driving DC motors via PWM over GPIO pins
   "Pi Radiotelescope Driver"

*/

/** \file pirt.cpp
    \brief A customized INDI telescope device with full functionality of a telescope mount.
    \author Hans-Georg Zaunick
*/

#include "pirt.h"
#include <connectionplugins/connectioninterface.h>

#include "indicom.h"

#include <cmath>
#include <map>
#include <memory>

#include <libnova.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ads1115.h"
#include "encoder.h"
#include "gpioif.h"
#include "motordriver.h"
#include "config.h"

#include "sysfspwm.hpp"

namespace Connection {
    class Interface;
    class Serial;
    class TCP;
}

constexpr char GPIO_CHIP_PATH[] {"/dev/gpiochip0"};

constexpr unsigned int SSI_BAUD_RATE { 1'000'000 }; //< SPI baud rate for encoder read-out
constexpr char AZ_SPIDEV_PATH[] {"/dev/spidev0.0"};
constexpr char ALT_SPIDEV_PATH[] {"/dev/spidev6.0"};

constexpr unsigned int POLL_INTERVAL_MS { 200 }; //< polling interval of this driver
constexpr double DEFAULT_AZ_AXIS_TURNS_RATIO { 152. / 9. }; //< ratio between Az encoder revolutions and Az axis revolutions
constexpr double DEFAULT_EL_AXIS_TURNS_RATIO { 1. }; //< ratio between Alt encoder revolutions and Alt axis revolutions
constexpr double MAX_AZ_OVERTURN { 0.5 }; //< maximum overturn in Az in revolutions at both ends
constexpr double ALT_LIMIT_LOW { 0.25 / 360. }; //< lower position limit Alt in revolutions
constexpr double ALT_LIMIT_HI { 100. / 360. }; //< upper position limit in Alt in revolutions
constexpr bool AZ_POS_DIR_INVERT { false }; //< invert helicity of Az axis
constexpr bool ALT_POS_DIR_INVERT { true }; //< invert helicity of Alt axis
constexpr double DEFAULT_AZ_AXIS_OFFSET { -181.25 }; //< offset between Az encoder-axis zero and real world Az-axis zero
constexpr double DEFAULT_ALT_AXIS_OFFSET { 0.64 }; //< offset between Alt encoder-axis zero and real world Alt-axis zero

constexpr double POS_ACCURACY_COARSE { 4.0 }; //< coarse positioning accuracy threshold in degrees
constexpr double POS_ACCURACY_FINE { 0.2 }; //< fine positioning accuracy threshold in degrees
constexpr double TRACK_ACCURACY_AZ { 0.06 }; //< tracking accuracy for Az axis threshold in degrees
constexpr double TRACK_ACCURACY_ALT { 0.04 }; //< tracking accuracy for Alt axis threshold in degrees

constexpr unsigned int NR_SLEW_RATES { 5 }; //< number of slew speeds available for this scope

constexpr double MIN_AZ_MOTOR_THROTTLE_DEFAULT { 0.04 }; //< minimum applicable motor throttle, Az motor
constexpr double MIN_ALT_MOTOR_THROTTLE_DEFAULT { 0.10 }; //< minimum applicable motor throttle, Alt motor
constexpr double AZ_MOTOR_CURRENT_LIMIT_DEFAULT { 4.1 }; //< absolute motor current limit for Az motor in Ampere
constexpr double ALT_MOTOR_CURRENT_LIMIT_DEFAULT { 3.0 }; //< absolute motor current limit for Alt motor in Ampere
//constexpr double MOTOR_CURRENT_FACTOR { 1./0.14 }; //< conversion factor for motor current sense in A/V
constexpr bool AZ_MOTOR_DIR_INVERT { true }; //< invert default (positive) direction of Az motor
constexpr bool ALT_MOTOR_DIR_INVERT { true }; //< invert default (positive) direction of Alt motor

constexpr char PWM_UDEV_PATH[] {"/sys/class/pwm/pwmchip0"};
constexpr int AZ_PWM_CHANNEL { 0 };
constexpr int ALT_PWM_CHANNEL { 1 };
constexpr auto PWM_EXPORT_DELAY { std::chrono::microseconds(200'000) };

constexpr PiRaTe::MotorDriver::Pins AZ_MOTOR_PINS {
    .Dir = -1,
    .DirA = 23,
    .DirB = 24,
    .Enable = 25,
    .Fault = -1
}; //< GPIO pin mapping to functions provided by motor driver

constexpr PiRaTe::MotorDriver::Pins ALT_MOTOR_PINS {
    .Dir = -1,
    .DirA = 5,
    .DirB = 6,
    .Enable = 26,
    .Fault = -1
}; //< GPIO pin mapping to functions provided by motor driver

constexpr char I2C_DEV_PATH[] { "/dev/i2c-1" };

constexpr std::uint8_t MOTOR_ADC_ADDR { 0x48 }; //< I2C address of ADS1115 ADC for motor current read-out
constexpr std::uint8_t VOLTAGE_MONITOR_ADC_ADDR { 0x49 }; //< I2C address of ADS1115 ADC for voltage monitoring

constexpr std::chrono::milliseconds DEFAULT_INT_TIME { 1000 };

constexpr unsigned int MAX_TARGET_POINTING_IMPROVEMENT_TIME_MS { 250 };

struct GpioPin {
    std::string name;
    unsigned int gpio_pin;
    bool inverted;
};

const std::vector<GpioPin> GpioOutputVector { { "Relay1", 17, true },
    { "Relay2", 27, true },
    { "Relay3", 22, true },
    { "Relay4", 16, true },
    { "RefOut (BCM0)", 0, false }
};

const std::vector<GpioPin> GpioInputVector { { "In1 (BCM8)", 8, false },
    { "In2 (BCM7)", 7, false },
    { "In3 (BCM20)", 20, false },
    { "In4 (BCM1)", 1, false } };

struct I2cVoltageDef {
    std::string name;
    double nominal;
    double divider_ratio;
    std::uint8_t adc_address;
    std::uint8_t adc_channel;
    std::string unit;
};

const std::vector<I2cVoltageDef> supply_voltage_defs { { "+3.3V", 3.3, 2., VOLTAGE_MONITOR_ADC_ADDR, 0, "" },
    { "+5V", 5., 2., VOLTAGE_MONITOR_ADC_ADDR, 1, "" },
    { "+13.8V", 13.8, 11., VOLTAGE_MONITOR_ADC_ADDR, 2, "" },
    { "+24V", 24., 11., VOLTAGE_MONITOR_ADC_ADDR, 3, "" } };

const std::vector<I2cVoltageDef> measurement_voltage_defs { { "Analog1", 0., 55.5556, MOTOR_ADC_ADDR, 2, "dB" },
    { "Analog2", 0., 55.5556, MOTOR_ADC_ADDR, 3, "dB" } };

const HorCoords DefaultParkPosition { 180., 89.7 };

// VSTW Radebeul, Google-Maps: 51°06'58.1"N 13°37'17.3"E
const std::map<INDI::Telescope::TelescopeLocation, double> DefaultLocation = { { INDI::Telescope::LOCATION_LATITUDE, 51.116139 },
    { INDI::Telescope::LOCATION_LONGITUDE, 13.621472 },
    { INDI::Telescope::LOCATION_ELEVATION, 200. } };

// the server will handle one unique instance of the driver
static std::unique_ptr<PiRT> pirt(new PiRT());

/**************************************************************************************
** Return properties of device.
***************************************************************************************/
void ISGetProperties(const char* dev)
{
    pirt->ISGetProperties(dev);
}

/**************************************************************************************
** Process new switch from client
***************************************************************************************/
void ISNewSwitch(const char* dev, const char* name, ISState* states, char* names[], int n)
{
    pirt->ISNewSwitch(dev, name, states, names, n);
}

/**************************************************************************************
** Process new text from client
***************************************************************************************/
void ISNewText(const char* dev, const char* name, char* texts[], char* names[], int n)
{
    pirt->ISNewText(dev, name, texts, names, n);
}

/**************************************************************************************
** Process new number from client
***************************************************************************************/
void ISNewNumber(const char* dev, const char* name, double values[], char* names[], int n)
{
    pirt->ISNewNumber(dev, name, values, names, n);
}

/**************************************************************************************
** Process new blob from client
***************************************************************************************/
void ISNewBLOB(const char* dev, const char* name, int sizes[], int blobsizes[], char* blobs[], char* formats[],
    char* names[], int n)
{
    pirt->ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

/**************************************************************************************
** Process snooped property from another driver
***************************************************************************************/
void ISSnoopDevice(XMLEle* root)
{
    pirt->ISSnoopDevice(root);
}

/*
 * class Pi RT
 * 
 */
PiRT::PiRT()
{
    DBG_SCOPE = INDI::Logger::DBG_SESSION;
    // We add an additional debug level so we can log verbose scope status
//     DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");

    setVersion(CDRIVER_VERSION_MAJOR, CDRIVER_VERSION_MINOR);
    //    setTelescopeConnection(CONNECTION_SERIAL);
    setTelescopeConnection(CONNECTION_TCP);
    //    setTelescopeConnection(CONNECTION_NONE);
    //    setTelescopeConnection(Connection::Interface::CONNECTION_CUSTOM);
    //    setTelescopeConnection(1<<15);

    SetTelescopeCapability(
        TELESCOPE_CAN_GOTO | TELESCOPE_CAN_PARK | TELESCOPE_CAN_ABORT | TELESCOPE_HAS_TIME | TELESCOPE_HAS_LOCATION | TELESCOPE_HAS_TRACK_MODE | TELESCOPE_CAN_CONTROL_TRACK /*|
			TELESCOPE_HAS_TRACK_RATE */
        ,
        NR_SLEW_RATES);
}

/**************************************************************************************
** We init our properties here. The only thing we want to init are the Debug controls
***************************************************************************************/
bool PiRT::initProperties()
{
    // ALWAYS call initProperties() of parent first
    INDI::Telescope::initProperties();

    setDefaultPollingPeriod(POLL_INTERVAL_MS);

    IUFillLight(&ScopeStatusL[0], "SCOPE_IDLE", "Idle", IPS_OK);
    IUFillLight(&ScopeStatusL[1], "SCOPE_SLEWING", "Slew", IPS_IDLE);
    IUFillLight(&ScopeStatusL[2], "SCOPE_TRACKING", "Tracking", IPS_IDLE);
    IUFillLight(&ScopeStatusL[3], "SCOPE_PARKING", "Parking", IPS_IDLE);
    IUFillLight(&ScopeStatusL[4], "SCOPE_PARKED", "Parked", IPS_IDLE);

    IUFillLightVector(&ScopeStatusLP, ScopeStatusL, 5, getDeviceName(), "SCOPE_STATUS", "Scope Status", MAIN_CONTROL_TAB, IPS_IDLE);

    IUFillTextVector(&TimeTP, TimeT, 2, getDeviceName(), TimeTP.name, TimeTP.label, SITE_TAB, IP_RO, 60, IPS_IDLE);
    IUFillNumberVector(&LocationNP, LocationN, 3, getDeviceName(), LocationNP.name, LocationNP.label, SITE_TAB, IP_RO, 60, IPS_IDLE);

    IUFillNumber(&JDN, "JD", "", "%10.5f", 0, 1e8, 0, 0);
    IUFillNumberVector(&JDNP, &JDN, 1, getDeviceName(), "JD", "Julian Date", SITE_TAB,
        IP_RO, 60, IPS_IDLE);

    IUFillNumber(&HorN[AXIS_AZ], "AZ", "Azimuth (deg:mm:ss)", "%010.6m", 0, 360, 0, 0);
    IUFillNumber(&HorN[AXIS_ALT], "ALT", "Elevation (dd:mm:ss)", "%010.6m", -90, 90, 0, 90);
    IUFillNumberVector(&HorNP, HorN, 2, getDeviceName(), "HORIZONTAL_EOD_COORD", "Hor. Coordinates", MAIN_CONTROL_TAB,
        IP_RW, 60, IPS_IDLE);
    lastHorState = IPS_IDLE;

    LocationN[LOCATION_LATITUDE].value = DefaultLocation.at(LOCATION_LATITUDE);
    LocationN[LOCATION_LONGITUDE].value = DefaultLocation.at(LOCATION_LONGITUDE);
    LocationN[LOCATION_ELEVATION].value = DefaultLocation.at(LOCATION_ELEVATION);
    LocationNP.s = IPS_OK;
    IDSetNumber(&LocationNP, NULL);

    IUFillNumber(&EncoderBitRateN, "SSI_BITRATE", "SSI Bit Rate", "%5.0f Hz", 0, 5000000, 0, SSI_BAUD_RATE);
    IUFillNumberVector(&EncoderBitRateNP, &EncoderBitRateN, 1, getDeviceName(), "ENC_SPI_SETTINGS", "SPI Interface", "Encoders",
        IP_RW, 60, IPS_IDLE);
    defineProperty(&EncoderBitRateNP);
    IDSetNumber(&EncoderBitRateNP, nullptr);

    IUFillNumber(&AzEncoderN[0], "AZ_ENC_POS", "Position", "%5.4f rev", -32767, 32767, 0, 0);
    IUFillNumber(&AzEncoderN[1], "AZ_ENC_ST", "ST", "%5.0f", 0, 65535, 0, 0);
    IUFillNumber(&AzEncoderN[2], "AZ_ENC_MT", "MT", "%5.0f", -32767, 32767, 0, 0);
    IUFillNumber(&AzEncoderN[3], "AZ_ENC_ERR", "Bit Errors", "%5.0f", 0, 0, 0, 0);
    IUFillNumber(&AzEncoderN[4], "AZ_ENC_ROTIME", "R/O Time", "%5.0f us", 0, 0, 0, 0);
    IUFillNumberVector(&AzEncoderNP, AzEncoderN, 5, getDeviceName(), "AZ_ENC", "Azimuth", "Encoders",
        IP_RO, 60, IPS_IDLE);
    IUFillNumber(&ElEncoderN[0], "EL_ENC_POS", "Position", "%5.4f rev", -32767, 32767, 0, 0);
    IUFillNumber(&ElEncoderN[1], "EL_ENC_ST", "ST", "%5.0f", 0, 65535, 0, 3);
    IUFillNumber(&ElEncoderN[2], "EL_ENC_MT", "MT", "%5.0f", -32767, 32767, 0, 4);
    IUFillNumber(&ElEncoderN[3], "EL_ENC_ERR", "Bit Errors", "%5.0f", 0, 0, 0, 0);
    IUFillNumber(&ElEncoderN[4], "EL_ENC_ROTIME", "R/O Time", "%5.0f us", 0, 0, 0, 0);
    IUFillNumberVector(&ElEncoderNP, ElEncoderN, 5, getDeviceName(), "EL_ENC", "Elevation", "Encoders",
        IP_RO, 60, IPS_IDLE);
    IUFillNumber(&AzEncSettingN[0], "AZ_ENC_ST_BITS", "ST bits", "%5.0f", 0, 24, 0, 12);
    IUFillNumber(&AzEncSettingN[1], "AZ_ENC_MT_BITS", "MT bits", "%5.0f", 0, 24, 0, 12);
    IUFillNumberVector(&AzEncSettingNP, AzEncSettingN, 2, getDeviceName(), "AZ_ENC_SETTING", "Az Encoder Settings", "Encoders",
        IP_RW, 60, IPS_IDLE);
    defineProperty(&AzEncSettingNP);
    IDSetNumber(&AzEncSettingNP, NULL);

    IUFillNumber(&ElEncSettingN[0], "EL_ENC_ST_BITS", "ST bits", "%5.0f", 0, 24, 0, 13);
    IUFillNumber(&ElEncSettingN[1], "EL_ENC_MT_BITS", "MT bits", "%5.0f", 0, 24, 0, 12);
    IUFillNumberVector(&ElEncSettingNP, ElEncSettingN, 2, getDeviceName(), "EL_ENC_SETTING", "El Encoder Settings", "Encoders",
        IP_RW, 60, IPS_IDLE);
    defineProperty(&ElEncSettingNP);
    IDSetNumber(&ElEncSettingNP, NULL);

    double initval = DEFAULT_AZ_AXIS_TURNS_RATIO;
    if (IUGetConfigNumber(getDeviceName(), "AZ_AXIS_SETTING", "AZ_AXIS_RATIO", &initval)==0) {
        DEBUGF(DBG_SCOPE, "Found config for AZ_AXIS_RATIO: %5.4f", initval);
    }
    IUFillNumber(&AzAxisSettingN[0], "AZ_AXIS_RATIO", "Enc-to-Axis turns ratio", "%5.4f", 0.0001, 100000, 0, initval);
    axisRatio[0] = initval;

    initval = DEFAULT_AZ_AXIS_OFFSET;
    if (IUGetConfigNumber(getDeviceName(), "AZ_AXIS_SETTING", "AZ_AXIS_OFFSET", &initval)==0) {
        DEBUGF(DBG_SCOPE, "Found config for AZ_AXIS_OFFSET: %5.4f", initval);
    }
    IUFillNumber(&AzAxisSettingN[1], "AZ_AXIS_OFFSET", "Axis Offset", "%5.4f deg", -180., 180., 0, initval);
    axisOffset[0] = initval;

    IUFillNumberVector(&AzAxisSettingNP, AzAxisSettingN, 2, getDeviceName(), "AZ_AXIS_SETTING", "Az Axis Settings", "Axes",
        IP_RW, 60, IPS_IDLE);
    defineProperty(&AzAxisSettingNP);
    IDSetNumber(&AzAxisSettingNP, NULL);

    initval = DEFAULT_EL_AXIS_TURNS_RATIO;
    if (IUGetConfigNumber(getDeviceName(), "EL_AXIS_SETTING", "EL_AXIS_RATIO", &initval)==0) {
        DEBUGF(DBG_SCOPE, "Found config for EL_AXIS_RATIO: %5.4f", initval);
    }
    IUFillNumber(&ElAxisSettingN[0], "EL_AXIS_RATIO", "Enc-to-Axis turns ratio", "%5.4f", 0.0001, 100000, 0, initval);
    axisRatio[1] = initval;

    initval = DEFAULT_ALT_AXIS_OFFSET;
    if (IUGetConfigNumber(getDeviceName(), "EL_AXIS_SETTING", "EL_AXIS_OFFSET", &initval)==0) {
        DEBUGF(DBG_SCOPE, "Found config for EL_AXIS_OFFSET: %5.4f", initval);
    }
    IUFillNumber(&ElAxisSettingN[1], "EL_AXIS_OFFSET", "Axis Offset", "%5.4f deg", -90., 90., 0, initval);
    axisOffset[1] = initval;
    IUFillNumberVector(&ElAxisSettingNP, ElAxisSettingN, 2, getDeviceName(), "EL_AXIS_SETTING", "El Axis Settings", "Axes",
        IP_RW, 60, IPS_IDLE);
    defineProperty(&ElAxisSettingNP);
    IDSetNumber(&ElAxisSettingNP, NULL);

    IUFillNumber(&AxisAbsTurnsN[0], "AZ_AXIS_TURNS", "Az", "%5.4f rev", 0, 0, 0, 0);
    IUFillNumber(&AxisAbsTurnsN[1], "ALT_AXIS_TURNS", "Alt", "%5.4f rev", 0, 0, 0, 0);
    IUFillNumberVector(&AxisAbsTurnsNP, AxisAbsTurnsN, 2, getDeviceName(), "AXIS_ABSOLUTE_TURNS", "Absolute Axis Turns", "Axes",
        IP_RO, 60, IPS_IDLE);

    IUFillNumber(&MotorStatusN[0], "AZ_MOTOR_SPEED", "Az", "%4.0f %%", -100, 100, 0, 0);
    IUFillNumber(&MotorStatusN[1], "ALT_MOTOR_SPEED", "Alt", "%4.0f %%", -100, 100, 0, 0);
    IUFillNumberVector(&MotorStatusNP, MotorStatusN, 2, getDeviceName(), "MOTOR_STATUS", "Motor Status", "Motors",
        IP_RO, 60, IPS_IDLE);

    IUFillNumber(&MotorCurrentN[0], "AZ_MOTOR_CURRENT", "Az", "%4.2f A", 0, 0, 0, 0);
    IUFillNumber(&MotorCurrentN[1], "ALT_MOTOR_CURRENT", "Alt", "%4.2f A", 0, 0, 0, 0);
    IUFillNumberVector(&MotorCurrentNP, MotorCurrentN, 2, getDeviceName(), "MOTOR_CURRENT", "Motor Currents", "Motors",
        IP_RO, 60, IPS_IDLE);

    initval = MIN_AZ_MOTOR_THROTTLE_DEFAULT * 100;
    if (IUGetConfigNumber(getDeviceName(), "MOTOR_THRESHOLD", "AZ_MOTOR_THRESHOLD", &initval)==0) {
        DEBUGF(DBG_SCOPE, "Found config for AZ_MOTOR_THRESHOLD: %4.0f", initval);
    }
    IUFillNumber(&MotorThresholdN[0], "AZ_MOTOR_THRESHOLD", "Az", "%4.0f %%", 0, 100, 0, initval);

    initval = MIN_ALT_MOTOR_THROTTLE_DEFAULT * 100;
    if (IUGetConfigNumber(getDeviceName(), "MOTOR_THRESHOLD", "ALT_MOTOR_THRESHOLD", &initval)==0) {
        DEBUGF(DBG_SCOPE, "Found config for ALT_MOTOR_THRESHOLD: %4.0f", initval);
    }
    IUFillNumber(&MotorThresholdN[1], "ALT_MOTOR_THRESHOLD", "Alt", "%4.0f %%", 0, 100, 0, initval);
    IUFillNumberVector(&MotorThresholdNP, MotorThresholdN, 2, getDeviceName(), "MOTOR_THRESHOLD", "Motor Thresholds", "Motors",
        IP_RW, 60, IPS_IDLE);

    initval = AZ_MOTOR_CURRENT_LIMIT_DEFAULT;
    if (IUGetConfigNumber(getDeviceName(), "MOTOR_CURRENT_LIMITS", "AZ_MOTOR_CURRENT_LIMIT", &initval)==0) {
        DEBUGF(DBG_SCOPE, "Found config for AZ_MOTOR_CURRENT_LIMIT: %4.2f", initval);
    }
    IUFillNumber(&MotorCurrentLimitN[0], "AZ_MOTOR_CURRENT_LIMIT", "Az", "%4.2f A", 0, 0, 0, initval);

    initval = ALT_MOTOR_CURRENT_LIMIT_DEFAULT;
    if (IUGetConfigNumber(getDeviceName(), "MOTOR_CURRENT_LIMITS", "ALT_MOTOR_CURRENT_LIMIT", &initval)==0) {
        DEBUGF(DBG_SCOPE, "Found config for ALT_MOTOR_CURRENT_LIMIT: %4.2f", initval);
    }
    IUFillNumber(&MotorCurrentLimitN[1], "ALT_MOTOR_CURRENT_LIMIT", "Alt", "%4.2f A", 0, 0, 0, initval);
    IUFillNumberVector(&MotorCurrentLimitNP, MotorCurrentLimitN, 2, getDeviceName(), "MOTOR_CURRENT_LIMITS", "Motor Current Limits", "Motors",
        IP_RW, 60, IPS_IDLE);

    IUFillNumber(&VoltageMonitorN[0], "VOLTAGE", "+0V", "%4.2f V", 0, 0, 0, 0);
    IUFillNumberVector(&VoltageMonitorNP, VoltageMonitorN, 0, getDeviceName(), "VOLTAGE_MONITOR", "Voltages", "Monitoring",
        IP_RO, 60, IPS_IDLE);

    IUFillNumber(&VoltageMeasurementN[0], "MEASUREMENT0", "+0V", "%4.2f V", 0, 0, 0, 0);
    IUFillNumberVector(&VoltageMeasurementNP, VoltageMeasurementN, 0, getDeviceName(), "MEASUREMENTS", "Measurements", "Monitoring",
        IP_RO, 60, IPS_IDLE);
    
    initval = DEFAULT_INT_TIME.count() / 1000.;
    if (IUGetConfigNumber(getDeviceName(), "INT_TIME", "TIME", &initval)==0) {
        DEBUGF(DBG_SCOPE, "Found config for INT_TIME: %5.2f", initval);
    }
    IUFillNumber(&MeasurementIntTimeN, "TIME", "Time", "%5.2f s", 0.01, 1000., 0.1, initval);
    IUFillNumberVector(&MeasurementIntTimeNP, &MeasurementIntTimeN, 1, getDeviceName(), "INT_TIME", "Integration Time", "Monitoring",
        IP_RW, 60, IPS_IDLE);

    IUFillNumber(&TempMonitorN[0], "TEMP_SYSTEM", "CPU", "%4.2f °C", 0, 0, 0, 0);
    IUFillNumberVector(&TempMonitorNP, TempMonitorN, 0, getDeviceName(), "TEMPERATURE_MONITOR", "Temperatures", "Monitoring",
        IP_RO, 60, IPS_IDLE);

    IUFillNumber(&DriverUpTimeN, "UPTIME", "Uptime", "%5.2f h", 0, 0, 0, 0);
    IUFillNumberVector(&DriverUpTimeNP, &DriverUpTimeN, 1, getDeviceName(), "DRIVER_UPTIME", "Driver Uptime", "Monitoring",
        IP_RO, 60, IPS_IDLE);

    for (std::size_t relay_index = 0; relay_index < GpioOutputVector.size(); relay_index++) {
        IUFillSwitch(&OutputSwitchS[relay_index], std::string("GPIO_OUT" + std::to_string(relay_index)).c_str(), GpioOutputVector[relay_index].name.c_str(), ISS_OFF);
    }
    IUFillSwitchVector(&OutputSwitchSP, OutputSwitchS, GpioOutputVector.size(), getDeviceName(), "GPIO_OUTPUTS", "Outputs", "Switches",
        IP_RW, ISR_NOFMANY, 60, IPS_IDLE);

    for (std::size_t input_index = 0; input_index < GpioInputVector.size(); input_index++) {
        //		IUFillSwitch(&GpioInputS[input_index], std::string("GPIO_IN"+std::to_string(input_index)).c_str(), GpioInputVector[input_index].name.c_str(), ISS_OFF);
        IUFillLight(&GpioInputL[input_index], std::string("GPIO_IN" + std::to_string(input_index)).c_str(), GpioInputVector[input_index].name.c_str(), IPS_IDLE);
    }
    //	IUFillSwitchVector(&GpioInputSP, GpioInputS, GpioInputVector.size(), getDeviceName(), "GPIO_INPUTS", "Inputs", "Switches",
    //		IP_RO, ISR_NOFMANY, 60, IPS_IDLE);
    IUFillLightVector(&GpioInputLP, GpioInputL, GpioInputVector.size(), getDeviceName(), "GPIO_INPUTS", "Inputs", "Switches",
        IPS_IDLE);

    IUFillLight(&WeatherStatusN, "WEATHER_WIND_SPEED", "wind speed", IPS_IDLE);
    IUFillLightVector(&WeatherStatusNP, &WeatherStatusN, 1, getDeviceName(), "WEATHER_STATUS", "Status", "Monitoring",
        IPS_IDLE);

    addDebugControl();
    return true;
}

bool PiRT::updateProperties()
{
    // ALWAYS call initProperties() of parent first
    INDI::Telescope::updateProperties();
    // delete the paramters defined for optical scopes - don't need 'em
    deleteProperty(ScopeParametersNP.name);
    //	deleteProperty("ConnectSP");

    if (isConnected()) {
        defineProperty(&ScopeStatusLP);
        LocationNP.s = IPS_OK;
        IDSetNumber(&LocationNP, NULL);
        defineProperty(&HorNP);
        defineProperty(&JDNP);

        deleteProperty(EncoderBitRateNP.name);
//         IUFillNumberVector(&EncoderBitRateNP, &EncoderBitRateN, 1, getDeviceName(), "ENC_SPI_SETTINGS", "SPI Interface", "Encoders",
//             IP_RO, 60, IPS_IDLE);
        EncoderBitRateNP.p = IP_RO;
        defineProperty(&EncoderBitRateNP);
//         IDSetNumber(&EncoderBitRateNP, nullptr);

        defineProperty(&AzEncoderNP);
        defineProperty(&ElEncoderNP);
        defineProperty(&AxisAbsTurnsNP);
        defineProperty(&MotorStatusNP);
        defineProperty(&MotorCurrentNP);
        defineProperty(&MotorThresholdNP);
        defineProperty(&MotorCurrentLimitNP);
        defineProperty(&VoltageMonitorNP);
        defineProperty(&VoltageMeasurementNP);
        defineProperty(&MeasurementIntTimeNP);
        defineProperty(&TempMonitorNP);
        defineProperty(&DriverUpTimeNP);

        defineProperty(&OutputSwitchSP);
        defineProperty(&GpioInputLP);

        IDSnoopDevice("Weather Watcher", "WEATHER_STATUS");
    } else {
        deleteProperty(ScopeStatusLP.name);
        deleteProperty(HorNP.name);
        deleteProperty(JDNP.name);

        deleteProperty(EncoderBitRateNP.name);
//         IUFillNumberVector(&EncoderBitRateNP, &EncoderBitRateN, 1, getDeviceName(), "ENC_SPI_SETTINGS", "SPI Interface", "Encoders",
//             IP_RW, 60, IPS_IDLE);
        EncoderBitRateNP.p = IP_RW;
        defineProperty(&EncoderBitRateNP);
//         IDSetNumber(&EncoderBitRateNP, nullptr);

        deleteProperty(AzEncoderNP.name);
        deleteProperty(ElEncoderNP.name);
        deleteProperty(AxisAbsTurnsNP.name);
        deleteProperty(MotorStatusNP.name);
        deleteProperty(MotorCurrentNP.name);
        deleteProperty(MotorThresholdNP.name);
        deleteProperty(MotorCurrentLimitNP.name);
        deleteProperty(VoltageMonitorNP.name);
        deleteProperty(VoltageMeasurementNP.name);
        deleteProperty(MeasurementIntTimeNP.name);
        deleteProperty(TempMonitorNP.name);
        deleteProperty(DriverUpTimeNP.name);

        deleteProperty(OutputSwitchSP.name);
        deleteProperty(GpioInputLP.name);
    }

    return true;
}

/**************************************************************************************
**
***************************************************************************************/
bool PiRT::ISNewSwitch(const char* dev, const char* name, ISState* states, char* names[], int n)
{
    if (strcmp(dev, getDeviceName()) == 0) {
        //  Set output switch (relay switch)
        if (!strcmp(name, OutputSwitchSP.name)) {
            std::string tempstr { "Relay" };
            if (n < 0)
                return false;
            for (int index = 0; index < n; index++) {
                ISwitch* sw = IUFindSwitch(&OutputSwitchSP, names[index]);
                if (sw != nullptr) {
                    std::size_t sw_pos = std::distance(OutputSwitchS, sw);
                    if (sw->s != states[index]) {
                        tempstr += std::to_string(index + 1);
                        if (states[index] == ISS_ON) {
                            //OutputSwitchSP[relayIndex].s = IPS_OK;
                            gpio->set_gpio_state(GpioOutputVector[sw_pos].gpio_pin, !GpioOutputVector[sw_pos].inverted);
                            tempstr += " switch on";
                        } else {
                            //OutputSwitchSP[relayIndex].s = IPS_IDLE;
                            gpio->set_gpio_state(GpioOutputVector[sw_pos].gpio_pin, GpioOutputVector[sw_pos].inverted);
                            tempstr += " switch off";
                        }
                    }
                }
            }
            OutputSwitchSP.s = IPS_IDLE;
            IUUpdateSwitch(&OutputSwitchSP, states, names, n);
            IDSetSwitch(&OutputSwitchSP, tempstr.c_str());
            return true;
        }
    }
    //  Nobody has claimed this, so forward it to the base class's method
    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

bool PiRT::ISNewNumber(const char* dev, const char* name, double values[], char* names[], int n)
{
    if (strcmp(dev, getDeviceName()) == 0) {
        //  This one is for us
        if (!strcmp(name, HorNP.name)) {
            if (n != 2)
                return false;
            double az = values[0];
            double alt = values[1];
            if ((az < 0.) || (az > 360.) || (alt < -90.) || (alt > 90.))
                return false;
            // Remember Track State
            RememberTrackState = TrackState;
            // Issue GOTO
            bool rc = GotoHor(az, alt);
            if (rc) {
                HorNP.s = lastHorState = IPS_BUSY;
                //  Now fill in target co-ords, so domes can start turning
                double ra { 0. }, dec { 0. };
                Hor2Equ(az, alt, &ra, &dec);
                TargetN[AXIS_RA].value = ra;
                TargetN[AXIS_DE].value = dec;
                IDSetNumber(&TargetNP, nullptr);
            } else {
                HorNP.s = lastHorState = IPS_ALERT;
            }
            IDSetNumber(&HorNP, nullptr);
            return rc;
        } else if (!strcmp(name, EncoderBitRateNP.name)) {
            // set Encoder bit rate
            EncoderBitRateNP.s = IPS_OK;
            EncoderBitRateN.value = values[0];
            IDSetNumber(&EncoderBitRateNP, nullptr);
            unsigned int rate = EncoderBitRateNP.np[0].value;
            DEBUGF(DBG_SCOPE, "Setting SSI bit rate to: %u Hz. Please reconnect client!", rate);
            return true;
        } else if (!strcmp(name, AzEncSettingNP.name)) {
            // set Az encoder bit widths
            unsigned int stBits = values[0];
            unsigned int mtBits = values[1];
            if (stBits > 1 && stBits < 20 && mtBits > 1 && mtBits < 20) {
                AzEncSettingNP.s = IPS_OK;
                AzEncSettingN[0].value = stBits;
                AzEncSettingN[1].value = mtBits;
                if (isConnected()) {
                    az_encoder->setStBitWidth(stBits);
                    az_encoder->setMtBitWidth(mtBits);
                }
                IDSetNumber(&AzEncSettingNP, nullptr);
                return true;
            } else {
                AzEncSettingNP.s = IPS_ALERT;
                return false;
            }
        } else if (!strcmp(name, ElEncSettingNP.name)) {
            // set El encoder bit widths
            unsigned int stBits = values[0];
            unsigned int mtBits = values[1];
            if (stBits > 1 && stBits < 20 && mtBits > 1 && mtBits < 20) {
                ElEncSettingNP.s = IPS_OK;
                ElEncSettingN[0].value = stBits;
                ElEncSettingN[1].value = mtBits;
                if (isConnected()) {
                    el_encoder->setStBitWidth(stBits);
                    el_encoder->setMtBitWidth(mtBits);
                }
                IDSetNumber(&ElEncSettingNP, nullptr);
                return true;
            } else {
                ElEncSettingNP.s = IPS_ALERT;
                return false;
            }
        } else if (!strcmp(name, AzAxisSettingNP.name)) {
            // Az axis settings: encoder-to-axis turns ratio and offset
            AzAxisSettingNP.s = IPS_OK;
            AzAxisSettingN[0].value = values[0];
            AzAxisSettingN[1].value = values[1];
            IDSetNumber(&AzAxisSettingNP, nullptr);
            axisRatio[0] = values[0];
            axisOffset[0] = values[1];
            DEBUGF(DBG_SCOPE, "Setting Az axis turns ratio to %5.4f rev.", axisRatio[0]);
            DEBUGF(DBG_SCOPE, "Setting Az axis offset %5.4f rev.", axisOffset[0]);
            return true;
        } else if (!strcmp(name, ElAxisSettingNP.name)) {
            // El axis settings: encoder-to-axis turns ratio and offset
            ElAxisSettingNP.s = IPS_OK;
            ElAxisSettingN[0].value = values[0];
            ElAxisSettingN[1].value = values[1];
            IDSetNumber(&ElAxisSettingNP, nullptr);
            axisRatio[1] = values[0];
            axisOffset[1] = values[1];
            DEBUGF(DBG_SCOPE, "Setting El axis turns ratio to %5.4f rev.", axisRatio[1]);
            DEBUGF(DBG_SCOPE, "Setting El axis offset %5.4f rev.", axisOffset[1]);
        } else if (!strcmp(name, MotorCurrentLimitNP.name)) {
            // set motor current limit
            bool success { true };
            for (int index{0}; index < n; ++index) {
                INumber* nr = IUFindNumber(&MotorCurrentLimitNP, names[index]);
                if (nr != nullptr) {
                    std::size_t nr_pos = std::distance(MotorCurrentLimitN, nr);
                    MotorCurrentLimitN[nr_pos].value = values[index];
                    DEBUGF(DBG_SCOPE, "Setting motor current limit of axis %d to %5.3f A", nr_pos, MotorCurrentLimitN[nr_pos].value);
                } else {
                    success = false;
                }
            }
            if (!success) {
                MotorCurrentLimitNP.s = IPS_ALERT;
            } else MotorCurrentLimitNP.s = IPS_OK;
            IDSetNumber(&MotorCurrentLimitNP, nullptr);
        } else if (!strcmp(name, MotorThresholdNP.name)) {
            // set motor thresholds
            bool success { true };
            for (int index{0}; index < n; ++index) {
                INumber* nr = IUFindNumber(&MotorThresholdNP, names[index]);
                if (nr != nullptr) {
                    std::size_t nr_pos = std::distance(MotorThresholdN, nr);
                    MotorThresholdN[nr_pos].value = values[index];
                    DEBUGF(DBG_SCOPE, "Setting motor threshold of axis %d to %4.0f%%", nr_pos, MotorThresholdN[nr_pos].value);
                } else {
                    success = false;
                }
            }
            if (!success) {
                MotorThresholdNP.s = IPS_ALERT;
            } else MotorThresholdNP.s = IPS_OK;
            IDSetNumber(&MotorThresholdNP, nullptr);
        } else if (!strcmp(name, MeasurementIntTimeNP.name)) {
            if (!voltageMeasurements.empty() && values[0] > 0. && values[0] < 1000.) {
                for (auto meas : voltageMeasurements) {
                    meas->setIntTime(std::chrono::milliseconds(static_cast<long int>(values[0] * 1000)));
                }
                MeasurementIntTimeN.value = values[0];
                IDSetNumber(&MeasurementIntTimeNP, nullptr);
                MeasurementIntTimeNP.s = IPS_OK;
                return true;
            } else {
                MeasurementIntTimeNP.s = IPS_ALERT;
                return false;
            }
        }
    }
    //  Nobody has claimed this, so forward it to the base class method
    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

bool PiRT::ISSnoopDevice(XMLEle* root)
{
    char *dev, *name;

    /* check and crack type, device, name and state */
    if (crackDN(root, &dev, &name, NULL) < 0)
        return false;
    if (!strcmp(name, WeatherStatusNP.name)) {
        (void)crackIPState(findXMLAttValu(root, "state"), &WeatherStatusNP.s);
        if (WeatherStatusNP.s == IPS_OK)
            DEBUG(INDI::Logger::DBG_SESSION, "Weather status is ok");
        else if (WeatherStatusNP.s == IPS_BUSY)
            DEBUG(INDI::Logger::DBG_SESSION, "Weather status is in warning range");
        else if (WeatherStatusNP.s == IPS_ALERT)
            DEBUG(INDI::Logger::DBG_WARNING, "Weather status is critical!");
        return true;
    }
    return INDI::Telescope::ISSnoopDevice(root);
}

bool PiRT::saveConfigItems(FILE *fp) {
    // Save custom setting
    IUSaveConfigNumber(fp, &MotorCurrentLimitNP);
    IUSaveConfigNumber(fp, &MotorThresholdNP);
    IUSaveConfigNumber(fp, &EncoderBitRateNP);
    IUSaveConfigNumber(fp, &AzAxisSettingNP);
    IUSaveConfigNumber(fp, &ElAxisSettingNP);
    IUSaveConfigNumber(fp, &MeasurementIntTimeNP);
    // Save base telescope config
    return INDI::Telescope::saveConfigItems(fp);
}

bool PiRT::SetTrackMode(uint8_t /* mode */)
{
    return false;
}

bool PiRT::SetTrackEnabled(bool enabled)
{
    if (enabled && TrackState == SCOPE_PARKED) {
        DEBUG(INDI::Logger::DBG_ERROR, "Scope in park position - tracking is prohibited.");
        return false;
    }
    if (enabled) {
        targetEquatorialCoords = Hor2Equ(currentHorizontalCoords);
    } else {
        Abort();
    }
    fIsTracking = enabled;
    return true;
}

bool PiRT::Park()
{
    if (TrackState == SCOPE_PARKED) {
        DEBUG(INDI::Logger::DBG_ERROR, "Scope already parked.");
        return false;
    }

    targetHorizontalCoords = DefaultParkPosition;

    char AzStr[64] = { 0 }, AltStr[64] = { 0 };

    // Parse the Az/Alt into strings
    fs_sexa(AzStr, targetHorizontalCoords.Az.value(), 2, 3600);
    fs_sexa(AltStr, targetHorizontalCoords.Alt.value(), 2, 3600);

    // Mark state as parking
    TrackState = SCOPE_PARKING;
    TargetCoordSystem = SYSTEM_HOR;

    // Inform client we are slewing to a new position
    DEBUGF(INDI::Logger::DBG_SESSION, "Slewing to Park Pos ( Az: %s - Alt: %s )", AzStr, AltStr);

    return true;
}

bool PiRT::UnPark()
{
    if (TrackState != SCOPE_PARKED) {
        DEBUG(INDI::Logger::DBG_ERROR, "Scope already unparked.");
        return false;
    }
    SetParked(false);

    TrackState = SCOPE_IDLE;

    return true;
}

bool PiRT::Connect()
{
    fStartTime = std::chrono::system_clock::now();

//     ITextVectorProperty* tvp = getText("DEVICE_ADDRESS");
//     if (tvp == nullptr) {
//         DEBUG(INDI::Logger::DBG_ERROR, "No address property found.");
//         //return false;
//     }

    //DEBUGF(INDI::Logger::DBG_SESSION, "host: %s:%s", tvp->tp[0].text, tvp->tp[1].text);
    /*
	const std::string host { tvp->tp[0].text };
	const std::string port { tvp->tp[1].text };
*/

    // before instantiating a new GPIO interface, all objects which carry a reference
    // to the old gpio object must be invalidated, to make sure
    // that noone else uses the shared_ptr<GPIO> when it is newly created
    az_encoder.reset();
    el_encoder.reset();
    az_motor.reset();
    el_motor.reset();
    tempMonitor.reset();
    i2cDeviceMap.clear();
    voltageMonitors.clear();
    voltageMeasurements.clear();

    gpio = std::make_shared<PiRaTe::Gpio>(GPIO_CHIP_PATH);
    if (!gpio || !gpio->is_initialised()) {
        DEBUGF(INDI::Logger::DBG_ERROR, "Could not initialize GPIO interface. Is gpiod installed and %s available?", GPIO_CHIP_PATH);
        return false;
    }

    // set the baud rate on the SPI interface for communication with the pos encoders
    unsigned int bitrate = static_cast<unsigned int>(EncoderBitRateNP.np[0].value);
    if (bitrate < 80'000 || bitrate > 5'000'000) {
        DEBUG(INDI::Logger::DBG_ERROR, "SSI bitrate out of range (80k...5M)");
        return false;
    }

    // initialize Az pos encoder connected to the main SPI interface
    az_encoder = std::make_unique<PiRaTe::SsiPosEncoder>(std::string(AZ_SPIDEV_PATH), bitrate, PiRaTe::spi_device::MODE::MODE3);
    if (!az_encoder || !az_encoder->isInitialized()) {
        DEBUGF(INDI::Logger::DBG_ERROR, "Failed to connect to Az position encoder at %s", AZ_SPIDEV_PATH);
        return false;
    }
    DEBUG(INDI::Logger::DBG_SESSION, "Az position encoder ok.");

    az_encoder->setStBitWidth(AzEncSettingN[0].value);
    az_encoder->setMtBitWidth(AzEncSettingN[1].value);

    // initialize Alt pos encoder connected to the aux SPI interface
    el_encoder = std::make_unique<PiRaTe::SsiPosEncoder>(std::string(ALT_SPIDEV_PATH), bitrate, PiRaTe::spi_device::MODE::MODE3);
    if (!el_encoder || !el_encoder->isInitialized()) {
        DEBUGF(INDI::Logger::DBG_ERROR, "Failed to connect to Alt position encoder at %s", ALT_SPIDEV_PATH);
        return false;
    }
    DEBUG(INDI::Logger::DBG_SESSION, "Alt position encoder ok.");
    el_encoder->setStBitWidth(ElEncSettingN[0].value);
    el_encoder->setMtBitWidth(ElEncSettingN[1].value);

    // search for the ADS1115 ADCs at the specified addresses and initialize them
    // instantiate the first ADS1115 foreseen to read back the motor currents
    auto adc1 = std::make_shared<PiRaTe::ADS1115>(I2C_DEV_PATH, MOTOR_ADC_ADDR);
    if (adc1 != nullptr && adc1->devicePresent()) {
        adc1->setPga(PiRaTe::ADS1115::PGA4V);
        adc1->setRate(PiRaTe::ADS1115::RATE860);
        adc1->setAGC(true);
        DEBUGF(INDI::Logger::DBG_SESSION, "ADC1 values ch0: %f V ch1: %f ch3: %f V ch4: %f", 
               adc1->readVoltage(0),
               adc1->readVoltage(1),
               adc1->readVoltage(2),
               adc1->readVoltage(3));
        i2cDeviceMap.emplace(std::make_pair(MOTOR_ADC_ADDR, std::move(adc1)));
    } else {
        DEBUGF(INDI::Logger::DBG_ERROR, "ADS1115 at address 0x%02x not found.", MOTOR_ADC_ADDR);
        deleteProperty(MotorCurrentNP.name);
        deleteProperty(MotorCurrentLimitNP.name);
    }
    // instantiate second ADS1115 for voltage monitoring
    auto adc2 = std::make_shared<PiRaTe::ADS1115>(I2C_DEV_PATH, VOLTAGE_MONITOR_ADC_ADDR);
    if (adc2 != nullptr && adc2->devicePresent()) {
        adc2->setPga(PiRaTe::ADS1115::PGA4V);
        adc2->setRate(PiRaTe::ADS1115::RATE860);
        adc2->setAGC(true);
        DEBUGF(INDI::Logger::DBG_SESSION, "ADC2 values ch0: %f V ch1: %f ch3: %f V ch4: %f",
               adc2->readVoltage(0),
               adc2->readVoltage(1),
               adc2->readVoltage(2),
               adc2->readVoltage(3));
        i2cDeviceMap.emplace(std::make_pair(VOLTAGE_MONITOR_ADC_ADDR, std::move(adc2)));
    } else {
        DEBUGF(INDI::Logger::DBG_ERROR, "ADS1115 at address 0x%02x not found.", VOLTAGE_MONITOR_ADC_ADDR);
    }

    // initialize pwm chip (through udev system)
    sysfspwm::PWMChip pwmchip(std::string{PWM_UDEV_PATH});
    if (pwmchip.get_npwm() < 2) {
        // the system doesn't show at least two usable PWM channels, so we abort here
        DEBUGF(INDI::Logger::DBG_ERROR, "PWM device %s does not have two usable channels (pwmchip.get_npwm()=%i).", pwmchip.get_name().c_str(), pwmchip.get_npwm());
        return false;
    }
    // export both pwm channels from pwm chip
    auto pwm0 = std::make_shared<sysfspwm::PWM>(pwmchip.export_pwm(AZ_PWM_CHANNEL));
    auto pwm1 = std::make_shared<sysfspwm::PWM>(pwmchip.export_pwm(ALT_PWM_CHANNEL));
    std::this_thread::sleep_for(PWM_EXPORT_DELAY);
    
    // initialize Az motor driver
    az_motor = std::make_unique<PiRaTe::MotorDriver>(gpio, std::move(pwm0), AZ_MOTOR_PINS, AZ_MOTOR_DIR_INVERT, std::dynamic_pointer_cast<PiRaTe::ADS1115>(i2cDeviceMap[MOTOR_ADC_ADDR]), 0);
    if (!az_motor || !az_motor->isInitialized()) {
        DEBUG(INDI::Logger::DBG_ERROR, "Failed to initialize Az motor driver.");
        return false;
    }
    // initialize Alt motor driver
    el_motor = std::make_unique<PiRaTe::MotorDriver>(gpio, std::move(pwm1), ALT_MOTOR_PINS, ALT_MOTOR_DIR_INVERT, std::dynamic_pointer_cast<PiRaTe::ADS1115>(i2cDeviceMap[MOTOR_ADC_ADDR]), 1);
    if (!el_motor && !el_motor->isInitialized()) {
        DEBUG(INDI::Logger::DBG_ERROR, "Failed to initialize El motor driver.");
        return false;
    }

    // initialize the temperature monitor
    TempMonitorNP.nnp = 0;
    IDSetNumber(&TempMonitorNP, nullptr);
    tempMonitor = std::make_shared<PiRaTe::RpiTemperatureMonitor>();
    if (tempMonitor != nullptr) {
        tempMonitor->registerTempReadyCallback([this](PiRaTe::RpiTemperatureMonitor::TemperatureItem item) { this->updateTemperatures(item); });
    }

    // set up the supply voltages to be monitored
    voltageMonitors.clear();
    int voltage_index = 0;
    for (auto item : supply_voltage_defs) {
        auto it = i2cDeviceMap.find(item.adc_address);
        if (it == i2cDeviceMap.end())
            continue;
        std::shared_ptr<PiRaTe::ADS1115> adc(std::dynamic_pointer_cast<PiRaTe::ADS1115>(it->second));
        auto mon = std::make_shared<PiRaTe::Ads1115VoltageMonitor>(item.name, std::move(adc), item.adc_channel, item.nominal, item.divider_ratio, item.nominal / 10.);
        voltageMonitors.emplace_back(std::move(mon));
//         deleteProperty(VoltageMonitorNP.name);
        IUFillNumber(&VoltageMonitorN[voltage_index], ("VOLTAGE" + std::to_string(voltage_index)).c_str(), (item.name).c_str(), "%4.2f V", item.nominal * 0.9, item.nominal * 1.1, 0, 0.);

        voltage_index++;
    }
    IUFillNumberVector(&VoltageMonitorNP, VoltageMonitorN, voltage_index, getDeviceName(), "VOLTAGE_MONITOR", "System Voltages", "Monitoring",
        IP_RO, 60, IPS_IDLE);
//     defineProperty(&VoltageMonitorNP);

    // set up the measurement voltages to be monitored
    voltageMeasurements.clear();
    voltage_index = 0;
    for (auto item : measurement_voltage_defs) {
        auto it = i2cDeviceMap.find(item.adc_address);
        if (it == i2cDeviceMap.end())
            continue;
        std::shared_ptr<PiRaTe::ADS1115> adc(std::dynamic_pointer_cast<PiRaTe::ADS1115>(it->second));
        auto meas = std::make_shared<PiRaTe::Ads1115Measurement>(item.name, std::move(adc), item.adc_channel, item.divider_ratio, DEFAULT_INT_TIME);
        voltageMeasurements.emplace_back(std::move(meas));
//         deleteProperty(VoltageMeasurementNP.name);
//         deleteProperty(MeasurementIntTimeNP.name);
        IUFillNumber(&VoltageMeasurementN[voltage_index], ("MEASUREMENT" + std::to_string(voltage_index)).c_str(), (item.name).c_str(), ("%4.3f " + item.unit).c_str(), 0, 0, 0, 0.);

        voltage_index++;
    }
    IUFillNumberVector(&VoltageMeasurementNP, VoltageMeasurementN, voltage_index, getDeviceName(), "MEASUREMENTS", "Measurements", "Monitoring",
        IP_RO, 60, IPS_IDLE);
//     defineProperty(&VoltageMeasurementNP);
//     defineProperty(&MeasurementIntTimeNP);

    // set up the gpio pins for the relay switches
    IUResetSwitch(&OutputSwitchSP);
    for (unsigned int i = 0; i < GpioOutputVector.size(); i++) {
        gpio->set_gpio_direction(GpioOutputVector[i].gpio_pin, PiRaTe::Gpio::Direction::DIRECTION_OUTPUT);
        gpio->set_gpio_state(GpioOutputVector[i].gpio_pin, GpioOutputVector[i].inverted);
    }

    // set up the gpio pins for the digital inputs
    for (unsigned int i = 0; i < GpioInputVector.size(); i++) {
        gpio->set_gpio_direction(GpioInputVector[i].gpio_pin, PiRaTe::Gpio::Direction::DIRECTION_INPUT);
    }

    INDI::Telescope::Connect();

    updatePosition();
    if (std::fabs(currentHorizontalCoords.Az.degrees() - DefaultParkPosition.Az.degrees()) < 0.5
        && std::fabs(currentHorizontalCoords.Alt.degrees() - DefaultParkPosition.Alt.degrees()) < 0.5) {
        SetParked(true);
    }

    return true;
}

bool PiRT::Disconnect()
{
    az_encoder.reset();
    el_encoder.reset();
    az_motor.reset();
    el_motor.reset();
    tempMonitor.reset();
    i2cDeviceMap.clear();
    voltageMonitors.clear();
    voltageMeasurements.clear();
    gpio.reset();
    return true;
}

void PiRT::TimerHit()
{
    if (isConnected()) {
        bool rc;
        rc = ReadScopeStatus();
        if (rc == false) {
            //  read was not good
            EqNP.s = IPS_ALERT;
            IDSetNumber(&EqNP, NULL);
        }
        //DEBUG(INDI::Logger::DBG_SESSION, "Timer hit");
        SetTimer(getCurrentPollingPeriod());
    }
}

/**************************************************************************************
** INDI is asking us to check communication with the device via a handshake
***************************************************************************************/
bool PiRT::Handshake()
{
    // When communicating with a real mount, we check here if commands are receieved
    // and acknolowedged by the mount.
    if (isConnected()) {
        if (!gpio->is_initialised())
            return false;
        if (!az_encoder->statusOk())
            return false;
        if (!el_encoder->statusOk())
            return false;
        if (!az_motor->isInitialized())
            return false;
        if (!el_motor->isInitialized())
            return false;
    }
    return true;
}

/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char* PiRT::getDefaultName()
{
    return "Pi Radiotelescope";
}

/**************************************************************************************
** is telescope in tracking mode?
***************************************************************************************/
auto PiRT::isTracking() const -> bool
{
    //return true;
    //return (TrackStateS[TRACK_ON].s == ISS_ON);
    return fIsTracking;
}

/**************************************************************************************
** Client is asking us to slew to a new position
***************************************************************************************/
bool PiRT::Goto(double ra, double dec)
{
    targetEquatorialCoords = EquCoords { ra, dec };
    targetHorizontalCoords = Equ2Hor(targetEquatorialCoords);

    if (targetHorizontalCoords.Alt.value() < 0.) {
        DEBUG(INDI::Logger::DBG_WARNING, "Error: Target below horizon");
        return false;
    }

    char RAStr[64] = { 0 }, DecStr[64] = { 0 };

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, ra, 2, 3600);
    fs_sexa(DecStr, dec, 2, 3600);

    // Mark state as slewing
    TrackState = SCOPE_SLEWING;
    TargetCoordSystem = SYSTEM_EQ;
    // Inform client we are slewing to a new position
    DEBUGF(INDI::Logger::DBG_SESSION, "Slewing to RA: %s - DEC: %s", RAStr, DecStr);

    targetPointingCycles = 0;

    // Success!
    return true;
}

/**************************************************************************************
** Client is asking us to slew to a new position (horizontal coordinates)
***************************************************************************************/
bool PiRT::GotoHor(double az, double alt)
{
    if (isParked()) {
        DEBUG(INDI::Logger::DBG_WARNING, "Please unpark the mount before issuing any motion/sync commands.");
        return false;
    }

    if (alt < 0.) {
        DEBUG(INDI::Logger::DBG_WARNING, "Error: Target below horizon");
        return false;
    }

    targetHorizontalCoords = HorCoords { az, alt };

    char AzStr[64] = { 0 }, AltStr[64] = { 0 };

    // Parse the Az/Alt into strings
    fs_sexa(AzStr, az, 2, 3600);
    fs_sexa(AltStr, alt, 2, 3600);

    // Mark state as slewing
    TrackState = SCOPE_SLEWING;
    TargetCoordSystem = SYSTEM_HOR;

    // Inform client we are slewing to a new position
    DEBUGF(INDI::Logger::DBG_SESSION, "Slewing to Az: %s - Alt: %s", AzStr, AltStr);

    targetPointingCycles = 0;

    // Success!
    return true;
}

/**************************************************************************************
** Client is asking us to abort our motion
***************************************************************************************/
bool PiRT::Abort()
{
    az_motor->stop();
    el_motor->stop();
    targetPointingCycles = 0;
    if (TrackState == SCOPE_IDLE || TrackState == SCOPE_TRACKING || TrackState == SCOPE_PARKED)
        return true;
    else
        TrackState = (isTracking() ? SCOPE_TRACKING : SCOPE_IDLE);

    targetEquatorialCoords = Hor2Equ(currentHorizontalCoords);
    return true;
}

bool PiRT::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    if (command != MOTION_START) {
        el_motor->stop();
        return true;
    }
    int speedIndex = IUFindOnSwitchIndex(&SlewRateSP);

    float speed = 0.;
    if (NR_SLEW_RATES < 2) {
        speed = 1.;
    } else {
        speed = 0.01 * (MotorThresholdN[1].value + (100. - MotorThresholdN[1].value) * speedIndex / (NR_SLEW_RATES - 1));
    }

    switch (dir) {
    case DIRECTION_SOUTH:
        el_motor->move(-speed);
        break;
    case DIRECTION_NORTH:
        el_motor->move(speed);
        break;
    default:
        el_motor->stop();
        break;
    }
    return true;
}

bool PiRT::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    if (command != MOTION_START) {
        az_motor->stop();
        return true;
    }

    int speedIndex = IUFindOnSwitchIndex(&SlewRateSP);
    float speed = 0.;
    if (NR_SLEW_RATES < 2) {
        speed = 1.;
    } else {
        speed = 0.01 * (MotorThresholdN[0].value + (100. - MotorThresholdN[0].value) * speedIndex / (NR_SLEW_RATES - 1));
    }

    switch (dir) {
    case DIRECTION_WEST:
        az_motor->move(speed);
        break;
    case DIRECTION_EAST:
        az_motor->move(-speed);
        break;
    default:
        az_motor->stop();
        break;
    }
    return true;
}

void PiRT::Hor2Equ(const HorCoords& hor_coords, double* ra, double* dec)
{
    Hor2Equ(hor_coords.Az.value(), hor_coords.Alt.value(), ra, dec);
}

EquCoords PiRT::Hor2Equ(const HorCoords& hor_coords)
{
    double ra {}, dec {};
    Hor2Equ(hor_coords.Az.value(), hor_coords.Alt.value(), &ra, &dec);
    return EquCoords(ra, dec);
}

void PiRT::Hor2Equ(double az, double alt, double* ra, double* dec)
{
    struct ln_date date;
    struct tm* utc;

    time_t raw_time;
    time(&raw_time);

    utc = gmtime(&raw_time);

    /* UT date and time */
    date.years = utc->tm_year + 1900;
    date.months = utc->tm_mon + 1;
    date.days = utc->tm_mday;
    date.hours = utc->tm_hour;
    date.minutes = utc->tm_min;
    date.seconds = utc->tm_sec;

    double JD = ln_get_julian_day(&date);

    struct ln_hrz_posn horcoords;
    // 0 deg Az should be S, in libnova it is N
    horcoords.az = ln_range_degrees(az + 180.);
    horcoords.alt = alt;

    struct ln_lnlat_posn geocoords;
    double x = LocationN[LOCATION_LONGITUDE].value;
    if (x > 180.)
        x -= 360.;
    geocoords.lng = x;
    geocoords.lat = LocationN[LOCATION_LATITUDE].value;

    struct ln_equ_posn equcoords;

    ln_get_equ_from_hrz(&horcoords, &geocoords, JD, &equcoords);
    *ra = equcoords.ra * 24. / 360.;
    *dec = equcoords.dec;
}

HorCoords PiRT::Equ2Hor(const EquCoords& equ_coords)
{
    double az {}, alt {};
    Equ2Hor(equ_coords.Ra.value(), equ_coords.Dec.value(), &az, &alt);
    return HorCoords(az, alt);
}

void PiRT::Equ2Hor(double ra, double dec, double* az, double* alt)
{
    struct ln_date date;
    struct tm* utc;

    time_t raw_time;
    time(&raw_time);

    utc = gmtime(&raw_time);

    /* UT date and time */
    date.years = utc->tm_year + 1900;
    date.months = utc->tm_mon + 1;
    date.days = utc->tm_mday;
    date.hours = utc->tm_hour;
    date.minutes = utc->tm_min;
    date.seconds = utc->tm_sec;

    double JD = ln_get_julian_day(&date);

    struct ln_equ_posn equcoords;
    equcoords.ra = 360. * ra / 24.;
    equcoords.dec = dec;

    struct ln_lnlat_posn geocoords;
    double x = LocationN[LOCATION_LONGITUDE].value;
    if (x > 180.)
        x -= 360.;
    geocoords.lng = x;
    geocoords.lat = LocationN[LOCATION_LATITUDE].value;

    struct ln_hrz_posn horcoords;

    ln_get_hrz_from_equ(&equcoords, &geocoords, JD, &horcoords);
    // 0 deg Az should be S, in libnova it is N
    horcoords.az = ln_range_degrees(horcoords.az - 180.);
    *az = horcoords.az;
    *alt = horcoords.alt;
}

bool PiRT::isInAbsoluteTurnRangeAz(double absRev)
{

    if ((absRev > -0.5 - MAX_AZ_OVERTURN)
        && (absRev < 0.5 + MAX_AZ_OVERTURN))
        return true;
    return false;
}

bool PiRT::isInAbsoluteTurnRangeAlt(double absRev)
{

    if ((absRev > ALT_LIMIT_HI)
        && (absRev < ALT_LIMIT_HI))
        return true;
    return false;
}

void PiRT::updateMotorStatus()
{
    if ((az_motor->hasFaultSense() && az_motor->isFault())
        || (el_motor->hasFaultSense() && el_motor->isFault())) {
        MotorStatusNP.s = IPS_ALERT;
    } else {
        MotorStatusNP.s = IPS_OK;
    }
    MotorStatusN[0].value = 100. * az_motor->currentSpeed();
    MotorStatusN[1].value = 100. * el_motor->currentSpeed();
    IDSetNumber(&MotorStatusNP, nullptr);

    if (az_motor->hasAdc() || el_motor->hasAdc()) {
        MotorCurrentNP.s = IPS_OK;
        if (az_motor->hasAdc()) {
            MotorCurrentN[0].value = az_motor->readCurrent() + 0.005;
        } else {
            MotorCurrentNP.s = IPS_BUSY;
        }
        if (el_motor->hasAdc()) {
            MotorCurrentN[1].value = el_motor->readCurrent() + 0.005;
        } else {
            MotorCurrentNP.s = IPS_BUSY;
        }
        if (MotorCurrentN[0].value > MotorCurrentLimitN[0].value) {
            // Motor current limit exceeded. Stop immediately
            az_motor->stop();
            MotorCurrentNP.s = IPS_ALERT;
        }
        if (MotorCurrentN[1].value > MotorCurrentLimitN[1].value) {
            // Motor current limit exceeded. Stop immediately
            el_motor->stop();
            MotorCurrentNP.s = IPS_ALERT;
        }
        //DEBUGF(INDI::Logger::DBG_SESSION, "ADC value ch0: %f V ch1: %f ch3: %f V ch4: %f", v1,v2,v3,v4);
        IDSetNumber(&MotorCurrentNP, nullptr);
    }
}

void PiRT::updateMonitoring()
{
    // update uptime
    DriverUpTimeN.value = upTime().count() / 3600.;
    IDSetNumber(&DriverUpTimeNP, nullptr);

    // update inputs
    bool change_detected { false };
    for (std::size_t index = 0; index < GpioInputVector.size(); index++) {
        const bool state = gpio->get_gpio_state(GpioInputVector[index].gpio_pin);
        if ((GpioInputL[index].s == IPS_OK && !state) || (GpioInputL[index].s == IPS_IDLE && state)) {
            // the state of the pin changed
            change_detected = true;
            GpioInputL[index].s = (state) ? IPS_OK : IPS_IDLE;
            GpioInputLP.s = IPS_OK;
            IDSetLight(&GpioInputLP, nullptr);
        }
    }
    if (!change_detected && GpioInputLP.s == IPS_OK) {
        GpioInputLP.s = IPS_IDLE;
        IDSetLight(&GpioInputLP, nullptr);
    }

    int voltage_index = 0;
    if (!voltageMonitors.empty()) {
        bool outsideRange { false };
        VoltageMonitorNP.s = IPS_IDLE;
        for (auto monitor : voltageMonitors) {
            if (!monitor->isInitialized()) {
                VoltageMonitorN[voltage_index].value = 0.;
                VoltageMonitorNP.s = IPS_ALERT;
            } else {
                double meanVoltage = monitor->meanVoltage();
                if (meanVoltage < VoltageMonitorN[voltage_index].min || meanVoltage > VoltageMonitorN[voltage_index].max) {
                    outsideRange = true;
                }
                VoltageMonitorN[voltage_index].value = meanVoltage;
            }
            voltage_index++;
        }
        if (VoltageMonitorNP.s != IPS_ALERT) {
            if (outsideRange)
                VoltageMonitorNP.s = IPS_BUSY;
            else
                VoltageMonitorNP.s = IPS_OK;
        }
        IDSetNumber(&VoltageMonitorNP, nullptr);
    }

    voltage_index = 0;
    if (!voltageMeasurements.empty()) {
        VoltageMeasurementNP.s = IPS_IDLE;
        for (auto meas : voltageMeasurements) {
            if (!meas->isInitialized()) {
                VoltageMeasurementN[voltage_index].value = 0.;
                VoltageMeasurementNP.s = IPS_ALERT;
            } else {
                double meanVoltage = meas->meanValue();
                VoltageMeasurementN[voltage_index].value = meanVoltage;
            }
            voltage_index++;
        }
        if (VoltageMeasurementNP.s != IPS_ALERT) {
            VoltageMeasurementNP.s = IPS_OK;
        }
        IDSetNumber(&VoltageMeasurementNP, nullptr);
    }
}

void PiRT::updateTemperatures(PiRaTe::RpiTemperatureMonitor::TemperatureItem item)
{
    if (!isConnected())
        return;
    //	DEBUGF( INDI::Logger::DBG_SESSION, " temp measurement from source %s: %f", item.id.c_str(), item.temperature );
    int source = item.sourceIndex;
    if (source < TempMonitorNP.nnp) {
        TempMonitorN[source].value = item.temperature;
        IDSetNumber(&TempMonitorNP, nullptr);
        return;
    }
    deleteProperty(TempMonitorNP.name);

    IUFillNumber(&TempMonitorN[source], ("TEMPERATURE" + std::to_string(source)).c_str(), (item.name + ":" + item.id).c_str(), "%4.2f °C", 0, 0, 0, item.temperature);
    IUFillNumberVector(&TempMonitorNP, TempMonitorN, source + 1, getDeviceName(), "TEMPERATURE_MONITOR", "Temperatures", "Monitoring",
        IP_RO, 60, IPS_IDLE);

    defineProperty(&TempMonitorNP);
    IDSetNumber(&TempMonitorNP, nullptr);
}

void PiRT::updatePosition()
{
    double azAbsTurns { 0. };
    double altAbsTurns { 0. };

    if (az_encoder == nullptr || el_encoder == nullptr) {
        if (az_encoder == nullptr)
            AzEncoderNP.s = IPS_ALERT;
        if (el_encoder == nullptr)
            ElEncoderNP.s = IPS_ALERT;
        return;
    }
    // read pos encoders
    if (az_encoder->isUpdated() || el_encoder->isUpdated()) {
        AzEncoderN[0].value = az_encoder->absolutePosition();
        AzEncoderN[1].value = static_cast<double>(az_encoder->position());
        AzEncoderN[2].value = static_cast<double>(az_encoder->nrTurns());
        AzEncoderN[3].value = az_encoder->bitErrorCount();
        AzEncoderN[4].value = az_encoder->lastReadOutDuration().count();
        //DEBUGF(INDI::Logger::DBG_SESSION, "Az Encoder values: st=%d mt=%u t_ro=%u us", st, mt, us);
        AzEncoderNP.s = (az_encoder->statusOk()) ? IPS_OK : IPS_ALERT;
        IDSetNumber(&AzEncoderNP, nullptr);
        ElEncoderN[0].value = el_encoder->absolutePosition();
        ElEncoderN[1].value = static_cast<double>(el_encoder->position());
        ElEncoderN[2].value = static_cast<double>(el_encoder->nrTurns());
        ElEncoderN[3].value = el_encoder->bitErrorCount();
        ElEncoderN[4].value = el_encoder->lastReadOutDuration().count();
        ElEncoderNP.s = (el_encoder->statusOk()) ? IPS_OK : IPS_ALERT;
        IDSetNumber(&ElEncoderNP, nullptr);

        const double az_revolutions { az_encoder->absolutePosition() };
        const double el_revolutions { el_encoder->absolutePosition() };

        azAbsTurns = (az_revolutions / axisRatio[0]) + axisOffset[0] / 360.;
        altAbsTurns = (el_revolutions / axisRatio[1]) + axisOffset[1] / 360.;
        if (AZ_POS_DIR_INVERT)
            azAbsTurns *= -1.;
        if (ALT_POS_DIR_INVERT)
            altAbsTurns *= -1.;

        AxisAbsTurnsN[0].value = azAbsTurns;
        AxisAbsTurnsN[1].value = altAbsTurns;
        if (std::abs(azAbsTurns) > 0.5 + MAX_AZ_OVERTURN || altAbsTurns < ALT_LIMIT_LOW || altAbsTurns > ALT_LIMIT_HI) {
            AxisAbsTurnsNP.s = IPS_ALERT;
        } else {
            AxisAbsTurnsNP.s = IPS_OK;
        }
        IDSetNumber(&AxisAbsTurnsNP, nullptr);

        currentHorizontalCoords.Az.setValue(360. * azAbsTurns);
        currentHorizontalCoords.Alt.setValue(360. * altAbsTurns);
        //DEBUG(INDI::Logger::DBG_SESSION, "encoders updated");
    }
}

void PiRT::updateTime()
{

    static struct timeval ltv {
        0, 0
    };
    struct timeval tv {
        0, 0
    };
    double dt = 0;
    static double dt_time_update = 0.;

    static char ts[32] = { 0 };
    struct tm *utc, *local;

    /* update elapsed time since last poll, don't presume exactly POLLMS */
    gettimeofday(&tv, nullptr);

    // this is the first poll after connect
    if (ltv.tv_sec == 0 && ltv.tv_usec == 0) {
        ltv = tv;
        LocationNP.s = IPS_OK;
        IDSetNumber(&LocationNP, NULL);
    }

    // time since last update
    dt = tv.tv_sec - ltv.tv_sec + 1e-6 * (tv.tv_usec - ltv.tv_usec);
    ltv = tv;

    //       LocationNP.s = IPS_OK;
    //       IDSetNumber(&LocationNP, NULL);

    // update telescope time display only about once per second to save bandwidth
    dt_time_update += dt;
    if (dt_time_update > 0.9) {
        dt_time_update = 0.;
        time_t raw_time;
        time(&raw_time);

        utc = gmtime(&raw_time);
        strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", utc);
        IUSaveText(&TimeT[0], ts);

        local = localtime(&raw_time);
        snprintf(ts, sizeof(ts), "%4.2f", (local->tm_gmtoff / 3600.0));
        IUSaveText(&TimeT[1], ts);

        TimeTP.s = IPS_OK;
        IDSetText(&TimeTP, NULL);

        struct ln_date date;
        //struct tm *utc;

        /* UT date and time */
        date.years = utc->tm_year + 1900;
        date.months = utc->tm_mon + 1;
        date.days = utc->tm_mday;
        date.hours = utc->tm_hour;
        date.minutes = utc->tm_min;
        date.seconds = utc->tm_sec;

        double JD = ln_get_julian_day(&date);
        JDN.value = JD;
        JDNP.s = IPS_OK;
        IDSetNumber(&JDNP, NULL);
        //       LocationNP.s = IPS_OK;
        //       IDSetNumber(&LocationNP, NULL);
    }
}

/**************************************************************************************
** Client is asking us to report telescope status
***************************************************************************************/
bool PiRT::ReadScopeStatus()
{
    double dx = 0, dy = 0;

    updateTime();

    //DEBUG(INDI::Logger::DBG_SESSION, "before encoder readout");
    double azAbsTurns { 0. };
    double altAbsTurns { 0. };

    // read pos encoders and update the horizontal coordinates, absolute turn values and properties
    updatePosition();
    azAbsTurns = AxisAbsTurnsN[0].value;
    altAbsTurns = AxisAbsTurnsN[1].value;

    // update motor status
    updateMotorStatus();

    // update monitoring variables
    updateMonitoring();

    const unsigned int MAX_TARGET_POINTING_CYCLES { 1 + MAX_TARGET_POINTING_IMPROVEMENT_TIME_MS / std::max(getCurrentPollingPeriod(), 10U) };

    // the state machine to handle all operation conditions:
    // SCOPE_IDLE, SCOPE_TRACKING, SCOPE_PARKING, SCOPE_PARKED and SCOPE_SLEWING
    switch (TrackState) {
    case SCOPE_TRACKING:
        TargetCoordSystem = SYSTEM_HOR;
        targetHorizontalCoords = Equ2Hor(targetEquatorialCoords);
        [[fallthrough]];
    case SCOPE_PARKING:
        [[fallthrough]];
    case SCOPE_SLEWING:
        if (TargetCoordSystem == SYSTEM_EQ) {
            //Equ2Hor(targetRA, targetDEC, &targetAz, &targetAlt);
            targetHorizontalCoords = Equ2Hor(targetEquatorialCoords);
        } else if (TargetCoordSystem == SYSTEM_HOR) {
        } else if (TargetCoordSystem == SYSTEM_GAL) {
        } else {
            // unknown coordinate system - abort
            Abort();
        }

        //PiRaTe::RotAxis diffAz { -180, 180, 360};

        // calculate the movement vector
        dx = targetHorizontalCoords.Az.value() - currentHorizontalCoords.Az.value();
        dy = targetHorizontalCoords.Alt.value() - currentHorizontalCoords.Alt.value();

        // correct angles to valid range
        if (dx > 180.) {
            dx -= 360.;
        } else if (dx < -180.) {
            dx += 360.;
        }

        // this should never happen
        if (dy > 180.) {
            dy -= 360.;
        } else if (dy < -180.) {
            dy += 360.;
        }

        /*
			DEBUGF(INDI::Logger::DBG_SESSION, "trg  Az=%f Alt=%f", targetHorizontalCoords.Az.value(), targetHorizontalCoords.Alt.value());
			DEBUGF(INDI::Logger::DBG_SESSION, "curr Az=%f Alt=%f", currentHorizontalCoords.Az.value(), currentHorizontalCoords.Alt.value());
			DEBUGF(INDI::Logger::DBG_SESSION, "initial dx=%f dy=%f", dx, dy);
*/

        // check, if the absolute position of the target is beyond the allowable limit
        if (!isInAbsoluteTurnRangeAz(azAbsTurns + dx / 360.)) {
            //DEBUGF(INDI::Logger::DBG_SESSION, "initial dx=%f dy=%f", dx, dy);
            // if not, we still must be sure to turn into the right direction toward the allowable range
            // e.g. if we are currently far in the forbidden range, make sure to not go further in
            const double alt_dx = (dx > 0.) ? (dx - 360.) : (dx + 360.);
            if (std::abs(azAbsTurns + dx / 360.) > std::abs(azAbsTurns + alt_dx / 360.)) {
                dx = alt_dx;
            }
            //DEBUGF(INDI::Logger::DBG_SESSION, "allowed dx=%f dy=%f", dx, dy);
        }

        // do the actual movement
        // in Az
        if (std::abs(dx) > POS_ACCURACY_COARSE) {
            az_motor->move((dx >= 0) ? 1. : -1.);
        } else if (std::abs(dx) > POS_ACCURACY_FINE) {
            double mot = dx / POS_ACCURACY_COARSE;
            if (std::abs(mot) < MotorThresholdN[0].value / 100.) {
                mot = (dx > 0.) ? MotorThresholdN[0].value / 100. : -MotorThresholdN[0].value / 100.;
            }
            az_motor->move(mot);
        } else if (std::abs(dx) > TRACK_ACCURACY_AZ) {
            az_motor->move((dx > 0.) ? MotorThresholdN[0].value / 100. : -MotorThresholdN[0].value / 100.);
        } else
            az_motor->stop();

        // in Alt
        if (std::abs(dy) > POS_ACCURACY_COARSE) {
            el_motor->move((dy >= 0) ? 1. : -1.);
        } else if (std::abs(dy) > POS_ACCURACY_FINE) {
            double mot = dy / POS_ACCURACY_COARSE;
            if (std::abs(mot) < MotorThresholdN[1].value / 100.) {
                mot = (dy > 0.) ? MotorThresholdN[1].value / 100. : -MotorThresholdN[1].value / 100.;
            }
            el_motor->move(mot);
        } else if (std::abs(dy) > TRACK_ACCURACY_ALT) {
            el_motor->move((dy > 0.) ? MotorThresholdN[1].value / 100. : -MotorThresholdN[1].value / 100.);
        } else
            el_motor->stop();

        // Let's check if we reached target position for both axes
        if (std::abs(dx) < TRACK_ACCURACY_AZ
            && std::abs(dy) < TRACK_ACCURACY_ALT
            && (++targetPointingCycles > MAX_TARGET_POINTING_CYCLES)) {
            if (TargetCoordSystem == SYSTEM_EQ) {
                EqNP.s = IPS_OK;
                IDSetNumber(&EqNP, nullptr);
            } else if (TargetCoordSystem == SYSTEM_HOR) {
                HorNP.s = lastHorState = IPS_OK;
                IDSetNumber(&HorNP, nullptr);
            }
            if (TrackState == SCOPE_SLEWING) {
                DEBUG(INDI::Logger::DBG_SESSION, "Telescope slew is complete.");
            } else if (TrackState == SCOPE_PARKING) {
                fIsTracking = false;
                //TrackState = SCOPE_PARKED;
                SetParked(true);
            }
            Abort();
        } else {
            //targetPointingCycles = 0;
        }
        break;
    case SCOPE_PARKED:
    case SCOPE_IDLE:
    default:
        //Abort();
        break;
    }

    // check for axis limits and stop movement AND tracking, if motors are moving further into the forbidden range
    // on the other hand, allow movement into the opposite direction only
    // Az axis
    if (azAbsTurns < -0.6 - MAX_AZ_OVERTURN) {
        // no more movements towards negative direction allowed
        DEBUGF(INDI::Logger::DBG_SESSION, "neg. Az overturn: azAbsTurns=%f limit=%f", azAbsTurns, -0.6 - MAX_AZ_OVERTURN);
        if (az_motor->currentSpeed() < 0.) {
            Abort();
            fIsTracking = false;
        }
    } else if (azAbsTurns > 0.6 + MAX_AZ_OVERTURN) {
        // no more movements towards positive direction allowed
        DEBUGF(INDI::Logger::DBG_SESSION, "pos. Az overturn: azAbsTurns=%f limit=%f", azAbsTurns, 0.6 + MAX_AZ_OVERTURN);
        if (az_motor->currentSpeed() > 0.) {
            Abort();
            fIsTracking = false;
        }
    }
    // Alt axis
    if (altAbsTurns < ALT_LIMIT_LOW) {
        // no more movements towards negative direction allowed
        if (el_motor->currentSpeed() < 0.) {
            Abort();
            if (fIsTracking)
                TrackState = SCOPE_IDLE;
            fIsTracking = false;
        }
    } else if (altAbsTurns > ALT_LIMIT_HI) {
        // no more movements towards positive direction allowed
        if (el_motor->currentSpeed() > 0.) {
            Abort();
            if (fIsTracking)
                TrackState = SCOPE_IDLE;
            fIsTracking = false;
        }
    }

    /* update scope status */
    // update the telescope state lights
    for (int i = 0; i < 5; i++)
        ScopeStatusL[i].s = IPS_IDLE;
    ScopeStatusL[TrackState].s = IPS_OK;
    IDSetLight(&ScopeStatusLP, NULL);

    // update horizontal coordinates
    if (HorN[AXIS_AZ].value != currentHorizontalCoords.Az.value()
        || HorN[AXIS_ALT].value != currentHorizontalCoords.Alt.value()) {
        HorN[AXIS_AZ].value = currentHorizontalCoords.Az.value();
        HorN[AXIS_ALT].value = currentHorizontalCoords.Alt.value();
        HorNP.s = IPS_IDLE;
        //lastEqState = EqNP.s;
        IDSetNumber(&HorNP, NULL);
    }

    double currentRA { 0. }, currentDEC { 0. };
    Hor2Equ(currentHorizontalCoords, &currentRA, &currentDEC);

    char RAStr[64] = { 0 }, DecStr[64] = { 0 };

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, currentRA, 2, 3600);
    fs_sexa(DecStr, currentDEC, 2, 3600);

    //	DEBUGF(DBG_SCOPE, "Current RA: %s Current DEC: %s", RAStr, DecStr);

    NewRaDec(currentRA, currentDEC);

    return true;
}

auto PiRT::upTime() const -> std::chrono::duration<long, std::ratio<1>>
{
    auto now { std::chrono::system_clock::now() };
    auto difftime { now - fStartTime };
    return std::chrono::duration_cast<std::chrono::seconds>(difftime);
}
