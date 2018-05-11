/*
 * pid.h
 *
 *  Created on: 10 Jul 2016
 *      Author: podonoghue
 */

#ifndef PROJECT_HEADERS_PID_H_
#define PROJECT_HEADERS_PID_H_

#include <time.h>

#define FULLROTATIONTICKS (8000)
#define QUARTERROTATIONTICKS (FULLROTATIONTICKS/4)
#define SAMPLES_FOR_AVERAGE (400)

class PID {
public:
   typedef float  InFunction();
   typedef void   OutFunction(float);
};

/**
 * These template parameters connect the PID controller to the control variables/operations
 *
 * @tparam inputFn     Input function  - used to obtain value of system state
 * @tparam outputFn    Output function - used to modify the control variable
 *
 */
template<PID::InFunction inputFn, PID::OutFunction outputFn>
class PID_T : PID {

private:
   const double sampleTime;
   const double outMin;
   const double outMax;

   double kp;                 // Proportional Tuning Parameter
   double ki;                 // Integral Tuning Parameter
   double kd;                 // Derivative Tuning Parameter

   bool   enabled;            // Enable for controller

   double integral;           // Integral accumulation term

   double lastInput;          // Last input sample
   double currentInput;       // Current input sample
   double currentOutput;      // Current output
   double setpoint;           // Setpoint for controller
   double currentError;



   bool averageErrorReady = false;
   int averageErrorRecord[SAMPLES_FOR_AVERAGE];
   int averageErrorRecordIndex;

public:

   /**
    * Constructor
    *
    * @param Kp          Initial proportional constant
    * @param Ki          Initial integral constant
    * @param Kd          Initial differential constant
    * @param sampleTime  Sample interval for controller. (update() should be called at this interval)
    * @param outMin      Minimum value of output variable
    * @param outMax      Maximum value of output variable
    */
   PID_T(double Kp, double Ki, double Kd, double sampleTime, double outMin, double outMax) :
      sampleTime(sampleTime), outMin(outMin), outMax(outMax)  {

      // Controller initially disabled
      enabled = false;

      setTunings(Kp, Ki, Kd);

      averageErrorRecordIndex = 0;
      averageErrorReady = false;
   }

   /**
    * Enable controller\n
    * Note: Controller is re-initialised when enabled
    *
    * @param enable True to enable
    */
   void enable(bool enable = true) {
      if(enable != enabled) {
         // Just enabled
         currentInput = inputFn();
         integral     = currentOutput;
      }
      enabled = enable;
   }

   /**
    * Indicates if the controller is enabled
    *
    * @return True => enabled
    */
   bool isEnabled() {
      return enabled;
   }

   /**
    * Main PID calculation
    *
    * Should be called \ref sampleTime interval.
    * This would usually be done by a timer call-back or similar.
    */
   void update() {
      if(!enabled) {
         return;
      }

      // Update input samples & error
      lastInput = currentInput;
      currentInput = inputFn();
      currentError = setpoint - currentInput;

      integral += (ki * currentError);
      if(integral > outMax) {
         integral = outMax;
      }
      else if(integral < outMin) {
         integral = outMin;
      }
      double dInput = (currentInput - lastInput);

//      if ((dInput>=-3) && (dInput<=3)) {
//         // Calculate PID Output
//         currentOutput = kp * currentError + integral - (kd/10) * dInput;
//      }
//      else {
         // Calculate PID Output
         currentOutput = kp * currentError + integral - kd * dInput;
//      }
      if(currentOutput > outMax) {
         currentOutput = outMax;
      }
      else if(currentOutput < outMin) {
         currentOutput = outMin;
      }
      // Update output
      outputFn(currentOutput);

      //Update average
      if(averageErrorRecordIndex >= SAMPLES_FOR_AVERAGE)
      {
    	  averageErrorRecordIndex = 0;

    	  averageErrorReady = true;//At least one full set of polls has occured since last reset
      }

      averageErrorRecord[averageErrorRecordIndex] = currentError;

      averageErrorRecordIndex ++;
   }

   /**
    * Change controller tuning
    *
    * @param Kp Proportional constant
    * @param Ki Integral constant
    * @param Kd Differential constant
    */
   void setTunings(double Kp, double Ki, double Kd) {
      if (Kp<0 || Ki<0 || Kd<0) {
         USBDM::setAndCheckErrorCode(USBDM::E_ILLEGAL_PARAM);
      }

      kp = Kp;
      ki = Ki * sampleTime;
      kd = Kd / sampleTime;
   }

   /**
    * Change setpoint of controller
    *
    * @param value Value to set
    */
   void setSetpoint(double value) {
      averageErrorReady = false;//Discontinuity breaks average

	   if (value > FULLROTATIONTICKS) {
         return;
      }
      if (value < -FULLROTATIONTICKS) {
         return;
      }
      setpoint = value;
   }

   /**
    * Get setpoint of controller
    *
    * @return Current setpoint
    */
   double getSetpoint() {
      return setpoint;
   }

   /**
    * Get input of controller
    *
    * @return Last input sample
    */
   double getInput() {
      return currentInput;
   }

   /**
    * Get setpoint of controller
    *
    * @return Last output sample
    */
   double getOutput() {
      return currentOutput;
   }

   /**
    * Get error of controller
    *
    * @return Last error calculation
    */
   double getError() {
      return currentError;
   }

   /**
    * Get proportional control factor
    *
    * @return factor as double
    */
   double getKp() {
      return  kp;
   }
   /**
    * Get integral control factor
    *
    * @return factor as double
    */
   double getKi() {
      return  ki/sampleTime;
   }
   /**
    * Get differential control factor
    *
    * @return factor as double
    */
   double getKd() {
      return  kd*sampleTime;
   }

   /*
    *Return a value indicating if a valid average is available
    *
    */
   bool getAverageReady()
   {
	   return averageErrorReady;
   }

   bool getIsSteadyState(int tolerance)
   {
	   bool result = false;

	   if(!averageErrorReady)
	   {
		   result = false;
	   }

	   else
	   {
		   int average = 0;

		   int i = 0;
		   while(i < SAMPLES_FOR_AVERAGE)
		   {
			   average += averageErrorRecord[i];

			   i += 1;
		   }

		   average /= SAMPLES_FOR_AVERAGE;

		   result = (abs(average) < abs(tolerance));
	   }

	   return result;
   }

};

#endif // PROJECT_HEADERS_PID_H_
