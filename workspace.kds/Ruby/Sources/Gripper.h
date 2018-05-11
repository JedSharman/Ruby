/*
 * Gripper.h
 *
 *  Created on: 18 Mar 2016
 *      Author: podonoghue
 */

#ifndef PROJECT_HEADERS_GRIPPER_H_
#define PROJECT_HEADERS_GRIPPER_H_

#include "hardware.h"
#include "console.h"

/**
 * @tparam DriverFTM    Describes FTM used to drive solenoid e.g. Ftm0Info
 * @tparam channel      Channel on FTM
 * @tparam OpenSensor   GPIO for hall effect sensor for open position
 * @tparam CloseSensor  GPIO for hall effect sensor for closed position
 *
 * Example:
 * @code
 * //                              FTM     Channel     OpenSensor       CloseSensor
 * using Gripper1 = Gripper<USBDM::Ftm0Info, 7, USBDM::GpioE<0>, USBDM::GpioE<1>>;
 * @endcode
 */
template<class DriverFTM, int channel, class OpenSensor, class CloseSensor>
class Gripper {

private:
   /** Duty cycle for solenoid on (percent) */
   static constexpr int SOLENOID_PWM_VALUE     = 99;
   /** Solenoid movement delay - closing (ms) */
   static constexpr int SOLENOID_OPERATE_DELAY = 100;
   /** Solenoid movement delay - Opening (ms) */
   static constexpr int SOLENOID_RELEASE_DELAY = 100;

   using Timer  = USBDM::FtmBase_T<DriverFTM>;
   using Driver = USBDM::FtmChannel_T<DriverFTM,channel>;

public:
   /**
    * Initialise the Gripper
    */
   static void initialise() {

      /* Grippers are run Open-drain to allow output voltage to rise above Vdd with external pull-ups */
      Driver::setPCR(PORT_PCR_DSE_MASK|PORT_PCR_ODE_MASK); // ~4V with external PUP

      if (!Timer::isEnabled()) {
         Timer::configure(USBDM::FtmMode_LeftAlign);
         Driver::setPeriod(PWM_PERIOD);
      }
      USBDM::checkError();

      Driver::configure(USBDM::FtmChMode_PwmHighTruePulses, USBDM::FtmChannelAction_None);

      open();
      OpenSensor::setInput();
      CloseSensor::setInput();
   }

   /**
    * Calibrate the Gripper
    * Just checks open and close operations
    */
   static bool calibrate() {
      if (!open()) {
         return false;
      }
      if (!close()) {
         return false;
      }
      open();
      return true;
   }

   /**
    * Close Gripper
    * Note: incorporates a delay waiting for solenoid to move
    *
    * @return true => OK, false => failed to close
    */
   static bool close() {
      // Power solenoid
      Driver::setDutyCycle(SOLENOID_PWM_VALUE);
      // Wait for confirmed movement
      if (USBDM::waitMS(SOLENOID_OPERATE_DELAY, CloseSensor::read)) {
         return true;
      }
      Driver::setDutyCycle(SOLENOID_PWM_VALUE);
      return false;
   }

   /**
    * Open Gripper
    * Note: incorporates a delay waiting for solenoid to move
    *
    * @return true => OK, false => failed to open
    */
   static bool open() {
      // Release solenoid
      Driver::setDutyCycle(0);
      // Wait for release confirmation
      return USBDM::waitMS(SOLENOID_RELEASE_DELAY, OpenSensor::read);
   }

};

#endif /* PROJECT_HEADERS_GRIPPER_H_ */