/*
 * Motor.h
 *
 *  Created on: 18 Mar 2016
 *      Author: podonoghue
 */

#ifndef PROJECT_HEADERS_MOTOR_H_
#define PROJECT_HEADERS_MOTOR_H_

#include "hardware.h"
#include <string.h>

/**
 *
 * @tparam DriverFTM       Info for FTM being used for Motor Driver (ChannelA, ChannelB, FaultInputNum)
 * @tparam ChannelA        FTM Channel A driving half of bridge
 * @tparam ChannelB        FTM Channel B driving half of bridge
 * @tparam FaultInputNum   FTM fault input connected to error flag from bridge driver indicating overload
 * @tparam EncoderFTM      Info for FTM used for quadrature encoder
 * @tparam EncoderIndex    GPIO connected to Quadrature Encoder index position detector
 *
 * Example:
 * @code
 * //                          FTM      ChA ChB Fault        Encoder          Indent
 * using Motor1 = Motor<USBDM::Ftm0Info, 2,  3,   3,  USBDM::Ftm1Info, USBDM::GpioA<5>>;
 * @endcode
 */
template <class DriverFTM, uint8_t ChannelA, uint8_t ChannelB, int FaultInputNum, class EncoderFTM, class EncoderIndex>
class Motor {

private:

   using Timer = USBDM::FtmBase_T<DriverFTM>;

   using Motor_A = USBDM::FtmChannel_T<DriverFTM, ChannelA>;
   using Motor_B = USBDM::FtmChannel_T<DriverFTM, ChannelB>;

public:
   using Encoder    = USBDM::QuadEncoder_T<EncoderFTM>;
   using EncoderFtm = USBDM::FtmBase_T<EncoderFTM>;

public:

   /**
    * Initialise motor driver
    */
   static void initialise() {
      using namespace USBDM;

      // Enable shared FTM
      if (!Timer::isEnabled()) {
         Timer::configure(FtmMode_LeftAlign, FtmClockSource_System);
         Timer::setPeriod(PWM_PERIOD);
      }
      USBDM::checkError();

      // Enable FTM outputs
      Motor_A::configure(FtmChMode_PwmHighTruePulses, FtmChannelAction_None);
      Motor_B::configure(FtmChMode_PwmHighTruePulses, FtmChannelAction_None);
      setSpeed(0);

      // Configure encoder
      EncoderIndex::setInput();

      Encoder::configure();
      Encoder::enableFilter(0);
      Encoder::resetPosition();
      Encoder::setTimerOverflowCallback(toiHandler);
      Encoder::enableTimerOverflowInterrupts();
   }

   // Handler for encoder overflow
   static void faultHandler() {
      // Fault
      setSpeed(0);
      USBDM::setAndCheckErrorCode(USBDM::E_ERROR);
   }

   // Handler for encoder overflow
   static void toiHandler() {
      if (EncoderFtm::tmr->QDCTRL&FTM_QDCTRL_TOFDIR_MASK) {
         // Overflowed top
      }
      else {
         // Overflowed bottom
      }
   }

   static bool fn() {
	  //! How large an arc the calibration can be safely applied
	  static constexpr int MAX_CALIBRATE_ARC_MOVE = 700;//absolute value
	  static constexpr int MIN_CALIBRATE_ARC_MOVE = -MAX_CALIBRATE_ARC_MOVE;

	  // Report status
      //USBDM::console.write("Pos=").write(Motor::getPosition()).write(", Index=").writeln(EncoderIndex::read());

	  //Safe guard against over turning
//	  if((getPosition() > MAX_CALIBRATE_ARC_MOVE) || (getPosition() < MIN_CALIBRATE_ARC_MOVE))//Check motor is within bounds
//	  {
//		  //TODO stop here, right now it turns off motor and waits for time out
//		  setSpeed(0);
//	  }

	  return EncoderIndex::read();
   };

   /**
    * Calibrate the motor position
    *
    * @return true => OK, false => Failed to find index location
    */
   static bool calibrate() {
      //! Speed to use when homing motor
      static constexpr int SLOW_SPEED         = 20;
      //! How long to wait for motor to reach home position
      static constexpr int MAX_CALIBRATE_WAIT = 1000;

      Encoder::resetPosition();//mark safety region zero
      setSpeed(SLOW_SPEED);

      //// Test motor operation at low speed without stop
//      for(;;) {
//         fn();
//      }

      // Enable fault inputs after 1ms so bridges have reset
      USBDM::waitMS(1);
      // Active-low fault inputs with filtering
//      Ftm::template enableFault<FaultInputNum>(false, true);
      Timer::setFaultCallback(faultHandler);
      Timer::enableFaultInterrupt();

      bool calibrated = USBDM::waitMS(MAX_CALIBRATE_WAIT, fn);
      Encoder::resetPosition();//mark index
      setSpeed(0);

      return calibrated;
   }

   /*
    * Set motor speed
    *
    * speed Speed to set motor -100.0...100.0
    */
   static void setSpeed(float speed) {
      if (speed<-100.0) {
         speed = -100.0;
      }
      else if (speed>100.0) {
         speed = 100.0;
      }
      if (speed > 0) {
         // Clockwise A=High, B=PWM
         Motor_A::setDutyCycle(100.0f-(float)speed);
         Motor_B::setDutyCycle(100.0f);
      }
      else if (speed < 0) {
         // Anti-clockwise A=PWM, B=High
         Motor_A::setDutyCycle(100.0f);
         Motor_B::setDutyCycle(100.0f+(float)speed);
      }
      else {
         // Braking A=B=High
         Motor_A::setDutyCycle(100.0f);
         Motor_B::setDutyCycle(100.0f);
      }
   }

   /*
    * Get motor position
    *
    * @return Position from shaft encoder
    */
   static int getPosition() {
      return Encoder::getPosition();
   }

   /*
    * Get motor position
    *
    * @return Position from shaft encoder
    */
   static float getPositionAsFloat() {
      return (float)Encoder::getPosition();
   }

};

#endif /* PROJECT_HEADERS_MOTOR_H_ */
