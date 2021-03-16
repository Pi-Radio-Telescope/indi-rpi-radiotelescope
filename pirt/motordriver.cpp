#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <chrono>
#include <memory>
#include <cassert>

#include "motordriver.h"

#include <ads1115.h>

#define DEFAULT_VERBOSITY 1

namespace PiRaTe {
	
constexpr std::chrono::milliseconds loop_delay { 10 };
constexpr std::chrono::milliseconds ramp_time { 1000 };
constexpr std::size_t adc_measurement_rate_loop_cycles { 100 };
constexpr double ramp_increment { static_cast<double>(loop_delay.count())/ramp_time.count() };
constexpr unsigned int HW_PWM1_PIN { 12 };
constexpr unsigned int HW_PWM2_PIN { 13 };
constexpr double MOTOR_CURRENT_FACTOR { 1./0.14 }; //< conversion factor for motor current sense in A/V

// helper functions for compilation with c++11
// remove, when compiling with c++14 and add std:: to the lines where these functions are used
template<class T>
const T& clamp( const T& v, const T& lo, const T& hi )
{
    assert( !(hi < lo) );
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

template <typename T> constexpr int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

MotorDriver::MotorDriver(std::shared_ptr<GPIO> gpio, Pins pins, bool invertDirection, std::shared_ptr<ADS1115> adc, std::uint8_t adc_channel)
	: fGpio { gpio }, fPins { pins }, fAdc { adc }, fCurrentDir { false }, fInverted { invertDirection }, fAdcChannel { adc_channel }
{
	if (fGpio == nullptr) {
		std::cerr<<"Error: no valid GPIO instance.\n";
		return;
	}
	
	if (!fGpio->isInitialized()) {
		std::cerr<<"Error: gpio interface not initialized.\n";
		return;
	}

	if ( ( fPins.Dir < 0 && !hasDualDir() ) || ( fPins.Pwm < 0 ) ) {
		std::cerr<<"Error: mandatory gpio pins for motor control undefined.\n";
		return;
	}
	
	// set pin directions
	if ( fPins.Dir >= 0 ) { 
		fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Dir), true);
		fGpio->set_gpio_state(static_cast<unsigned int>(fPins.Dir), (fInverted) ? !fCurrentDir : fCurrentDir);
	}
	if ( hasDualDir() ) {
		fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.DirA), true);
		fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.DirB), true);
		fGpio->set_gpio_state(static_cast<unsigned int>(fPins.DirA), (fInverted) ? !fCurrentDir : fCurrentDir);
		fGpio->set_gpio_state(static_cast<unsigned int>(fPins.DirB), (fInverted) ? fCurrentDir : !fCurrentDir);
	}
	
	//fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Pwm), true);
	
	if ( fPins.Enable >= 0 ) {
		fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Enable), true);
		fGpio->set_gpio_state(static_cast<unsigned int>(fPins.Enable), true);
	}
	if ( fPins.Fault >= 0 ) {
		fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Fault), false);
		fGpio->set_gpio_pullup(static_cast<unsigned int>(fPins.Fault));
	}	
	
	// initialize ADC if one was supplied in the argument list
	if ( fAdc != nullptr && fAdc->devicePresent() ) {
		fAdc->setPga(ADS1115::PGA4V);
		fAdc->setRate(ADS1115::RATE860);
		fAdc->setAGC(true);
		const std::lock_guard<std::mutex> lock(fMutex);
		measureVoltageOffset();
	}
	
	fActiveLoop=true;
// since C++14 using std::make_unique
	// fThread = std::make_unique<std::thread>( [this]() { this->readLoop(); } );
// C++11 is unfortunately more unconvenient with move from a locally generated pointer
	std::unique_ptr<std::thread> thread( new std::thread( [this]() { this->threadLoop(); } ));
	fThread = std::move(thread);
}

MotorDriver::~MotorDriver()
{
	if (!fActiveLoop) return;
	fActiveLoop = false;
	if (fThread!=nullptr) fThread->join();
	if (fGpio != nullptr && fGpio->isInitialized()) {
		if (fPins.Dir >= 0) fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Dir), false);
		if (fPins.DirA >= 0) fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.DirA), false);
		if (fPins.DirB >= 0) fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.DirB), false);
		fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Pwm), false);
		if ( fPins.Enable >= 0 ) {
			fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Enable), false);
		}
		if ( fPins.Fault >= 0 ) {
			fGpio->set_gpio_pullup(static_cast<unsigned int>(fPins.Fault), false);
		}
	}
}


// this is the background thread loop
void MotorDriver::threadLoop()
{
	std::size_t cycle_counter { adc_measurement_rate_loop_cycles };
	auto lastReadOutTime = std::chrono::system_clock::now();
	bool errorFlag = true;
	while (fActiveLoop) {
		auto currentTime = std::chrono::system_clock::now();
		
		if (hasFaultSense() && isFault()) {
			// fault condition, switch off and deactivate everything
			emergencyStop();
		} else {
			const std::lock_guard<std::mutex> lock(fMutex);
			//fMutex.lock();
			if (fTargetDutyCycle != fCurrentDutyCycle) {
				fCurrentDutyCycle += ramp_increment * sgn( fTargetDutyCycle - fCurrentDutyCycle );
				if ( std::abs(fTargetDutyCycle-fCurrentDutyCycle) < ramp_increment ) {
					fCurrentDutyCycle = fTargetDutyCycle;
				}
				setSpeed(fCurrentDutyCycle);
			}
			//fMutex.unlock();
		}
		if ( hasAdc() && !cycle_counter-- ) {
			double voltage { 0. };
			if ( bool readout_guard = true ) {
				//std::lock_guard<std::mutex> lock(fMutex);
				// read current from adc
				fMutex.lock();
				voltage = fAdc->readVoltage(fAdcChannel);
				fMutex.unlock();
			}
			if ( std::abs(fCurrentDutyCycle) < ramp_increment ) fOffsetBuffer.add(voltage);
			double _current = ( voltage - fOffsetBuffer.mean() ) * MOTOR_CURRENT_FACTOR;
			fMutex.lock();
			fCurrent = _current;
			fUpdated = true;
			fMutex.unlock();
			cycle_counter = adc_measurement_rate_loop_cycles;
		}
		std::this_thread::sleep_for(loop_delay);
	}
}

void MotorDriver::measureVoltageOffset() {

	for (unsigned int i=0; i<10; i++) {
		double voltage = fAdc->readVoltage(fAdcChannel);
		fOffsetBuffer.add(voltage);
	}
/*	
	constexpr unsigned int Niter { 10 };
	double adc_voltage { 0. };
	for (unsigned int i = 0; i<Niter; i++) {
		adc_voltage += fAdc->readVoltage(fAdcChannel);
	}
	fVoltageOffset = adc_voltage/Niter;
*/
}

auto MotorDriver::readCurrent() -> double 
{ 
	std::lock_guard<std::mutex> lock(fMutex);
	fUpdated = false;
	return (fCurrent);
}

auto MotorDriver::isFault() -> bool {
	if (!isInitialized() || !fGpio->isInitialized()) return true;
	if ( !hasFaultSense() ) return false;
	return (fGpio->get_gpio_state(static_cast<unsigned int>(fPins.Fault), nullptr) == false);
}

void MotorDriver::setSpeed(float speed_ratio) {
	const bool dir { (speed_ratio < 0.) };
	// set pins
	if ( dir != fCurrentDir ) {
		if ( fPins.Dir>=0 ) fGpio->set_gpio_state(static_cast<unsigned int>(fPins.Dir), (fInverted) ? !dir : dir);
		if ( hasDualDir() ) {
			fGpio->set_gpio_state(static_cast<unsigned int>(fPins.DirA), (fInverted) ? !dir : dir);
			fGpio->set_gpio_state(static_cast<unsigned int>(fPins.DirB), (fInverted) ? dir : !dir);
		}
		fCurrentDir = dir;
	}
	float abs_speed_ratio = std::abs(clamp(speed_ratio, -1.f, 1.f));
	std::uint32_t duty_cycle { 0 };
	if ( (static_cast<unsigned int>(fPins.Pwm) == HW_PWM1_PIN) || (static_cast<unsigned int>(fPins.Pwm) == HW_PWM2_PIN) ) {
		// use hardware pwm
		duty_cycle = 1000000U * abs_speed_ratio;
		int res = fGpio->hw_pwm_set_value(static_cast<unsigned int>(fPins.Pwm), fPwmFreq, duty_cycle);
		return;
	}
	duty_cycle = abs_speed_ratio * fPwmRange;
	int res = fGpio->pwm_set_value(static_cast<unsigned int>(fPins.Pwm), duty_cycle);
}

void MotorDriver::move(float speed_ratio) {
	const std::lock_guard<std::mutex> lock(fMutex);
	fTargetDutyCycle = clamp(speed_ratio, -1.f, 1.f);
}

void MotorDriver::stop() {
	this->move(0.);
}

void MotorDriver::emergencyStop() {
	this->move(0.);
	if (hasEnable()) {
		fGpio->set_gpio_state(static_cast<unsigned int>(fPins.Enable), false);
	}
}

void MotorDriver::setPwmFrequency(unsigned int freq)
{ 
	if (freq == fPwmFreq) return;
	if ( !(fPins.Pwm == HW_PWM1_PIN || fPins.Pwm == HW_PWM2_PIN) ) {
		fMutex.lock();
		int res = fGpio->pwm_set_frequency(static_cast<unsigned int>(fPins.Pwm), freq);
		fMutex.unlock();
	}
	fPwmFreq = freq;
}

auto MotorDriver::currentSpeed() -> float {
	const std::lock_guard<std::mutex> lock(fMutex);
	const float speed = fCurrentDutyCycle;
	return speed;
}

} // namespace PiRaTe
