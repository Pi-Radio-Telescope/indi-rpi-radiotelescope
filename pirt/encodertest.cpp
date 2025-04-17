/* simple program to read out absolute position encoder via SSI/SPI interface
 * compile with:
 g++ -std=gnu++14 -Wall -pthread -c gpioif.cpp
 g++ -std=gnu++14 -Wall -pthread -c encoder.cpp
 g++ -std=gnu++14 -Wall -pthread -c test.cpp
 g++ -std=gnu++14 -Wall -pthread -o test gpioif.o encoder.o test.o -lpigpiod_if2 -lrt
 */

#include <iostream>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <atomic>
#include <iomanip>

#include "spidevice.h"
#include "encoder.h"

constexpr char az_spidev_path[] { "/dev/spidev0.0" };
constexpr char alt_spidev_path[] { "/dev/spidev6.0" };

constexpr unsigned int CE0 { 8 };
constexpr unsigned int CE1 { 18 };
constexpr unsigned int CLK1 { 11 };
constexpr unsigned int CLK2 { 21 };
constexpr unsigned int DATA1 { 9 };
constexpr unsigned int DATA2 { 19 };

constexpr unsigned int spi_clock_rate { 1'000'000 };

std::string intToBinaryString(unsigned long number) {
   std::string numStr { };
   for (int i=31; i>=0; i--) {
      numStr += (number & (1<<i))?"1":"0";
   }
  return numStr;
}

std::atomic<bool> stop { false };

int main(void) {

    PiRaTe::SsiPosEncoder az_encoder(std::string(az_spidev_path), spi_clock_rate, PiRaTe::spi_device::MODE::MODE3);
	PiRaTe::SsiPosEncoder el_encoder(std::string(alt_spidev_path), spi_clock_rate, PiRaTe::spi_device::MODE::MODE3);
	el_encoder.setStBitWidth(13);
	
	std::thread thr( [&]() {    
		while (!stop) {
			if (az_encoder.isUpdated() || el_encoder.isUpdated() ) {
				unsigned int pos = az_encoder.position();
				int turns = az_encoder.nrTurns();
				std::cout<<"Az: st="<<std::setfill('0')<<std::setw(4)<<pos;
				std::cout<<" mt="<<turns<<" err="<<az_encoder.bitErrorCount();
				std::cout<<" deg/s="<<az_encoder.currentSpeed();
				std::cout<<" r/o="<<az_encoder.lastReadOutDuration().count()<<"us";
				
				pos = el_encoder.position();
				turns = el_encoder.nrTurns();
				std::cout<<"  El: st="<<std::setfill('0')<<std::setw(4)<<pos;
				std::cout<<" mt="<<turns<<" err="<<el_encoder.bitErrorCount();
				std::cout<<" deg/s="<<el_encoder.currentSpeed();
				std::cout<<" r/o="<<el_encoder.lastReadOutDuration().count()<<"us";
				std::cout<<"           \r";
			}
			usleep(50000U);
		}	
	} );


	std::cin.get();
	
	stop = true;
	
	thr.join();
	
	return EXIT_SUCCESS;
}

