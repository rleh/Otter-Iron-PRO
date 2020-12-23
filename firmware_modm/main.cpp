/* Copyright (c) 2017, Raphael Lehmann
 * All Rights Reserved.
 *
 * The file is part of the reflow-oven-modm project and is released under
 * the GPLv3 license.
 * See the file `LICENSE` for the full license governing this code.
 * ------------------------------------------------------------------------- */

#include <modm/math/filter/pid.hpp>
#include <modm/io/iostream.hpp>
#include <modm/processing.hpp>
#include <modm/processing/timer.hpp>
#include <modm/processing/protothread.hpp>
#include <modm/debug/logger.hpp>

#include <array>
#include <chrono>

#include <tusb.h>

#include "hardware.hpp"


// modm::IODeviceWrapper<modm::platform::UsbUart0, modm::IOBuffer::DiscardIfFull> loggerDevice;
// modm::log::Logger modm::log::debug(loggerDevice);
// modm::log::Logger modm::log::info(loggerDevice);
// modm::log::Logger modm::log::warning(loggerDevice);
// modm::log::Logger modm::log::error(loggerDevice);

//modm::IODeviceWrapper< Usart1, modm::IOBuffer::BlockIfFull > loggerDevice;
// Set all four logger streams to use the UART
// modm::log::Logger modm::log::debug(loggerDevice);
// modm::log::Logger modm::log::info(loggerDevice);
// modm::log::Logger modm::log::warning(loggerDevice);
// modm::log::Logger modm::log::error(loggerDevice);

using namespace std::literals::chrono_literals;

#define UTILS_LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * ((value) - (sample)))

modm::Pid<float, 1> pid;

class AdcThread : public modm::pt::Protothread
{
public:
    AdcThread() : adcTimer(100us) {
    }

    bool update() {
        PT_BEGIN();
        while(1) {
            PT_WAIT_UNTIL(adcTimer.execute());
            r = Iron::AnalogReadings::readAll();
			r.tTip -= r.tRef;
			r.tTip += tRefLp;
			UTILS_LP_FAST(iInLp, r.iIn, 0.001);
			UTILS_LP_FAST(tTipLp, r.tTip, 0.001);
			UTILS_LP_FAST(tRefLp, r.tRef, 0.001);
            PT_YIELD();
        }
        PT_END();
    }
	
	float getIInLp() const { return iInLp; };
	float getUIn() const { return r.uIn; };
	float getTTipLp() const { return tTipLp; };
	float getTRef() const { return tRefLp; };

private:
    modm::ShortPrecisePeriodicTimer adcTimer;
	Iron::AnalogReadings::Readings r; 
	float tTipLp; // low-pass filtered tip temperature in [°C]
	float iInLp; // low-pass filtered input current [A]
	float tRefLp;  // low-pass filtered reference temperature in [°C]
};
AdcThread adcThread;

class PidThread : public modm::pt::Protothread
{
public:
	PidThread() : pidTimer(10ms), tSetpoint(100.0f)
	{

	}

	bool
	update()
	{
		PT_BEGIN();

		while (1)
		{
			if(true) {
				PT_WAIT_UNTIL(pidTimer.execute());
				tTipLp = adcThread.getTTipLp();
				if( 0.0f <= tTipLp && tTipLp <= 700.0f ) { // FIXME
					pid.update(tSetpoint - tTipLp );
					// 800/2000 since we use PID values from original otter iron firmware
					// duty = std::max<float>(0.0f, pid.getValue())*800.0f/2000.0f;
					duty = std::max<float>(0.0f, pid.getValue())/10.0f;
					duty = std::max<float>(0, std::min<float>(1.0f, duty));
					Iron::Pwm::setDuty(duty);
				}
				else {
                    // MODM_LOG_INFO << "Error: Invalid temperature measurement" << modm::endl;
                    // MODM_LOG_INFO << tTipLp << modm::endl;
				}
			}
			else {
				Iron::Pwm::disable();
			}
			PT_YIELD();
		}

		PT_END();
	}

	float tSetpoint;
	float duty;

private:
	modm::PeriodicTimer pidTimer;
	float tTipLp;

};
PidThread pidThread;




// UiThread updates the display and reads the
// button states (with debouncing).
class UiThread : public modm::pt::Protothread
{
public:
	UiThread() :
		inputTimer(100ms),
		displayTimer(200ms)
	{
	}

	bool update(){
		PT_BEGIN();
		while (1)
		{
			if(inputTimer.execute()) {
				if ( Iron::Ui::ButtonUp::read() ) {
					pidThread.tSetpoint += 5.f; // [°C]
				}
				if ( Iron::Ui::ButtonDown::read()) {
					pidThread.tSetpoint -= 5.f; // [°C]
				}
			}
			if( displayTimer.execute()) {
				// MODM_LOG_INFO << "Temperatures: tTipLp=";
				// MODM_LOG_INFO << printf("%03.2f", adcThread.getTTipLp());
				// MODM_LOG_INFO << "°C, tSetpoint=";
				// MODM_LOG_INFO << printf("%03.2f", pidThread.tSetpoint);
				// MODM_LOG_INFO << "°C\n";

				Iron::Display::display.clear();
				Iron::Display::display.setCursor(0,0);
				Iron::Display::display.printf("T=%03.2f", adcThread.getTTipLp());
				Iron::Display::display.setCursor(64,0);
				Iron::Display::display.printf("S=%03.2f", pidThread.tSetpoint);
				Iron::Display::display.setCursor(0,32);
				Iron::Display::display.printf("D=%03.2f", pidThread.duty);
				Iron::Display::display.setCursor(64,32);
				Iron::Display::display.printf("R=%03.2f", adcThread.getTRef());
				Iron::Display::display.setCursor(0,48);
				Iron::Display::display.printf("U=%03.2f", adcThread.getUIn());
				Iron::Display::display.setCursor(64,48);
				Iron::Display::display.printf("I=%03.2f", adcThread.getIInLp());
				Iron::Display::display.update();
			}
			
			PT_YIELD();
		}
		PT_END();
	}
private:
	modm::PeriodicTimer inputTimer;
	modm::PeriodicTimer displayTimer;
};
UiThread uiThread;


int main()
{
    modm::delay(500ms);
	Iron::SystemClock::enable();
	modm::platform::SysTickTimer::initialize<Iron::SystemClock>();
	// Iron::usb::initialize();
	// tusb_init();

    // Usart1::connect<GpioA9::Tx>();
	// Usart1::initialize<Board::systemClock,115200_Bd>();
	// MODM_LOG_INFO << "Otter-Iron-PRO-v2.1-Desktop:modm-firmware starting ..." << modm::endl;
	// MODM_LOG_INFO << "Docker build ... \n" <<
	// " Step 1/2 : RUN apt-get update && apt-get install -y otter-iron-firmware \n" <<
 	// " ---> Using cache \n" <<
	// " ---> 946ee47ee05e \n" << 
	// " Step 2/2 : RUN chmod +x /opt/otter-iron-firmware/firmware.elf \n" <<
 	// " ---> Using cache \n" <<
 	// " ---> 2f7d8ceee512 \n" <<
	// " Successfully built 2f7d8ceee512 " << modm::endl;
	// modm::delay(100ms);
	// MODM_LOG_INFO << "VXWorks Initialized ..." << modm::endl;
	// MODM_LOG_INFO << "License Test ... USB License-Key not found ... Falling back to factory default ... Expired" << modm::endl;
	// MODM_LOG_INFO << "Please contact Sales Representative at vx@schulz-krause.de" << modm::endl;
	// modm::delay(100ms);
	// MODM_LOG_INFO << "Waiting for 5G to become available..." << modm::endl;
	// modm::delay(100ms);
	// MODM_LOG_INFO << "Baseband not found..." << modm::endl;
	// MODM_LOG_INFO << "Falling back to Fiber..." << modm::endl;
	// modm::delay(100ms);
	// MODM_LOG_INFO << "Falling back to Fiber..." << modm::endl;
	// modm::delay(100ms);
	// MODM_LOG_INFO << "Connection Successfull" << modm::endl;

	Iron::Pwm::initialize();
	Iron::Display::initialize();
	Iron::AnalogReadings::initialize();
	Iron::Ui::initialize();


	Iron::Display::display.setCursor(0,32);
	Iron::Display::display.printf("ping");
	Iron::Display::display.update();


	/*
	* use PID with [K] temperatures
	*/
	pid.setParameter(modm::Pid<float, 1>::Parameter( 
		0.5f,	// Kp
		1.0f,	// Ki
		0.0f, 		// Kd
		10.0f,  	// err_I limit
		Iron::Pwm::Overflow)); // output limit

	// MODM_LOG_DEBUG << "PWM Timer Overflow: " << Iron::Pwm::Overflow << modm::endl;

	//uint8_t counter = 0; 

	while (1)
	{
        adcThread.update();
		pidThread.update();
		uiThread.update();

		// tud_task();

		// MODM_LOG_INFO << "Loop" << modm::endl;
	}
	return 0;
}
