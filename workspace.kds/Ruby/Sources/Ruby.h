/*
 * Hardware.h
 *
 *  Created on: 19 Mar 2016
 *      Author: podonoghue
 */

#ifndef PROJECT_HEADERS_RUBY_H_
#define PROJECT_HEADERS_RUBY_H_

#include "hardware.h"
#include "dac.h"

/** Motor/solenoid PWM period - define before Motor/Gripper includes */
static constexpr float PWM_PERIOD  = 100 * USBDM::us; // 100 us + 10kHz

#include "Motor.h"
#include "Gripper.h"

/*
 * Pin mapping
 */
//                          FTM      ChA ChB Fault       Encoder          Indent
using Motor1 = Motor<USBDM::Ftm0Info, 2,  3,   3, USBDM::Ftm1Info, USBDM::GpioA<5>>;
using Motor2 = Motor<USBDM::Ftm0Info, 4,  5,   0, USBDM::Ftm2Info, USBDM::GpioB<3>>;

//                              FTM     Channel    OpenSensor       CloseSensor
using Gripper1 = Gripper<USBDM::Ftm0Info, 7, USBDM::GpioE<0>, USBDM::GpioE<1>>;
using Gripper2 = Gripper<USBDM::Ftm0Info, 1, USBDM::GpioC<0>, USBDM::GpioC<1>>;

// Test points
using TpA = USBDM::Gpio_p54;
using TpB = USBDM::Gpio_p56;

class MotorSupplySensor : public USBDM::Adc1Channel<4> {

private:
   static constexpr float CALIB_FACTOR  = (11*3.3)/1024.0;  /* Ratio of external voltage to ADC reading 10K:1K divider - TBC */
   static constexpr float MINIMUM_LEVEL = 10.0;             /* Minimum supply voltage for motor */

public:
   /**
    * Read Motor voltage using ADC
    *
    * @return Voltage as float
    */
   static float motorVoltage() {
      return readAnalogue()*CALIB_FACTOR;
   }
   /**
    * Checks if motor voltage meets the minimum required
    *
    * @return true if OK
    */
   static bool isOK() {
      return (motorVoltage() > MINIMUM_LEVEL);
   }

   static void initialise() {
      configure(USBDM::AdcResolution_10bit_se, USBDM::AdcClockSource_Bus, USBDM::AdcClockDivider_8);
      USBDM::waitMS(50);
      calibrate();
      setAveraging(USBDM::AdcAveraging_32);
   }
};

class DacOut : public USBDM::Dac0 {
public:
   /**
    * Send position to DAC output\n
    * Note: Value is scaled
    *
    * @param value Value to send
    */
   static void setValue(float value) {
      USBDM::Dac0::setValue(((int)value+4200)/5);
   }
};

#endif /* PROJECT_HEADERS_RUBY_H_ */
