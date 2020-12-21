/* Copyright (c) 2017, Raphael Lehmann
 * All Rights Reserved.
 *
 * The file is part of the reflow-oven-modm project and is released under
 * the GPLv3 license.
 * See the file `LICENSE` for the full license governing this code.
 * ------------------------------------------------------------------------- */
#ifndef HARDWARE_HPP
#define HARDWARE_HPP

#include <modm/driver/display/ssd1306.hpp>
#include <modm/platform.hpp>
#include <modm/architecture/interface/clock.hpp>
#define UTILS_LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * ((value) - (sample)))


namespace Iron
{

using namespace modm::literals;
using namespace modm::platform;

// COPIED FROM modm example for STM32F072_Discovery 
/// STM32F072 running at 48MHz generated from the internal 48MHz clock
// TODO: enable once clock driver is implemented
//using SystemClock = SystemClock<InternalClock<48_MHz>, 48_MHz>;

struct SystemClock
{
	static constexpr int Frequency = 48_MHz;
	static constexpr uint32_t Ahb = Frequency;
	static constexpr uint32_t Apb = Frequency;

	static constexpr uint32_t Adc   = Apb;
	static constexpr uint32_t Can   = Apb;

	static constexpr uint32_t Spi1   = Apb;
	static constexpr uint32_t Spi2   = Apb;

	static constexpr uint32_t Usart1 = Apb;
	static constexpr uint32_t Usart2 = Apb;
	static constexpr uint32_t Usart3 = Apb;
	static constexpr uint32_t Usart4 = Apb;

	static constexpr uint32_t I2c1   = Apb;
	static constexpr uint32_t I2c2   = Apb;

	static constexpr uint32_t Timer1  = Apb;
	static constexpr uint32_t Timer2  = Apb;
	static constexpr uint32_t Timer3  = Apb;
	static constexpr uint32_t Timer6  = Apb;
	static constexpr uint32_t Timer7  = Apb;
	static constexpr uint32_t Timer14 = Apb;
	static constexpr uint32_t Timer15 = Apb;
	static constexpr uint32_t Timer16 = Apb;
	static constexpr uint32_t Timer17 = Apb;

	static constexpr uint32_t Usb = Frequency;

	static bool inline
	enable()
	{
		// Enable the interal 48MHz clock
		Rcc::enableInternalClockMHz48();
		// set flash latency for 48MHz
		Rcc::setFlashLatency<Frequency>();
		// Switch to the 48MHz clock
		Rcc::enableSystemClock(Rcc::SystemClockSource::InternalClockMHz48);
		// update frequencies for busy-wait delay functions
		Rcc::updateCoreFrequency<Frequency>();

		return true;
	}
};

namespace usb
{
    using Dm = GpioA11;
    using Dp = GpioA12;

    using Device = UsbFs;

    inline void
    initialize()
    {
        Device::initialize<SystemClock>();
        Device::connect<Dm::Dm, Dp::Dp>();
    }
}


namespace Ui {
	using ButtonUp = GpioC13; // SW2 on silk, has external pulldown
	using ButtonDown = GpioA3; // SW1 on silk

	inline void
	initialize()
	{
		ButtonUp::setInput();
		ButtonDown::setInput(ButtonDown::InputType::PullDown);
	}
}


namespace Display {
	using Sda = GpioB7;
	using Scl = GpioB6;
	using MyI2cMaster = I2cMaster1;
	modm::Ssd1306<MyI2cMaster> display;

	inline void
	initialize()
	{
        MyI2cMaster::connect<Scl::Scl, Sda::Sda>();
        MyI2cMaster::initialize<Iron::SystemClock, 400_kHz>();

        display.initializeBlocking();
		display.setFont(modm::font::Assertion);
		display.setCursor(0,16);
		display << "Otter Iron v2.1";
		display.update();
	}
}

namespace Pwm {
	using Timer = Timer1;
	using Pin = GpioOutputA8;
	static uint16_t Overflow = 4800;

	inline void
	initialize()
	{
	    Pin::setOutput();
		Timer::connect<Pin::Ch1>();
		Timer::enable();
		Timer::setMode(Timer::Mode::UpCounter);
		// Pwm frequency: 10kHz
		Timer::setPrescaler(1); // 48 * 1000 * 1000 / 10000 / 4800 = 1
		Timer::setOverflow(Overflow);
		Timer::configureOutputChannel(1, Timer::OutputCompareMode::Pwm, 0);
		Timer::applyAndReset();
		Timer::start();
        Timer::enableOutput();
	}

	inline void
	set(uint16_t value)
	{
		Timer::setCompareValue(1, value);
	}

    // Use duty [0...1]
    inline void 
    setDuty(float duty) {
        duty = std::max<float>(0, std::min<float>(1.0f, duty));
        set(duty * Overflow);
    }

	inline void
	disable()
	{
		set(0);
	}
}

namespace AnalogReadings {
    using AdcIn0_iIn  = GpioA0;
    using AdcIn1_tTip = GpioA1;
    using AdcIn2_uIn  = GpioA2;
    using AdcIn5_tRef = GpioA5;

    inline void
    initialize() {
        Adc::connect<AdcIn0_iIn::In0, AdcIn1_tTip::In1, AdcIn2_uIn::In2, AdcIn5_tRef::In5>();
        Adc::initialize<Iron::SystemClock, Adc::ClockMode::Asynchronous, 14_MHz>();
        Adc::setRightAdjustResult();
    }

    struct Readings {
        float tRef;  // reference temperature in [°C]
        float tTip;  // tip temperature in [°C]
        float uIn;   // input voltage in [V]
        float iIn;   // input current in [A]
    };

    static constexpr float tipCalOffset  = 120.0f;
    static constexpr float tipCalCoeff   = 92.0f; 

    Readings readAll(){
        Readings r; 
        // copied from original otter iron firmware
        r.tRef = (((static_cast<float>(Adc::readChannel(Adc::Channel::In5))/4095.0f)*3.3f)-0.5f)/0.01f;
        r.tTip = ((static_cast<float>(Adc::readChannel(Adc::Channel::In1))-tipCalOffset)*tipCalCoeff)/1000.0f+r.tRef;
        r.uIn = (static_cast<float>(Adc::readChannel(Adc::Channel::In2))/4095.0f)*3.3f*6.6f;
        r.iIn = (static_cast<float>(Adc::readChannel(Adc::Channel::In0)/4095.0f)*3.3f*1.659f)/(0.01f*(2370.0f/33.0f));
        return r;
    }

}
} // namespace Iron


#endif	// HARDWARE_HPP
