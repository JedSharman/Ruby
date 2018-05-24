/*
 ============================================================================
 * @file    main.c (derived from main-basic.cpp)
 * @brief   Basic C++ demo using GPIO class
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ============================================================================
 */




#ifndef PROJECT_MAIN_/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define PROJECT_MAIN_

#include <stdio.h>
#include <random>
#include <string.h>
#include <vector>


#include "system.h"
#include "derivative.h"
#include "delay.h"
#include "Hardware.h"
#include "Ruby.h"
#include "pdb.h"
#include "pit.h"
#include "pid.h"

#define STEADY_STATE_TOLERANCE (((FULLROTATIONTICKS)/(360)) * ((3)/(3)))

using namespace USBDM;

//Used to distribute interpreted commands
std::vector<int> interpretedActions;

//Holds a list of chars read from the pc
std::vector<char> readCommands;

//represents the offset from the index to the initial position of each motor (in encoder ticks)
constexpr int motor1InitialOffset = -401;
constexpr int motor2InitialOffset = -2581;


void stopHere() {
   for(;;) {
      console.writeln("Stopped");
#ifdef DEBUG_BUILD
      __BKPT();
#else
      __WFE();
#endif
   }
}

void shutDown(void) {
   // Motors off
   Motor1::setSpeed(0);
   Motor2::setSpeed(0);

   console.writeln("System Failure");

   uint32_t oldM1Position = 0;
   uint32_t oldM2Position = 0;
   bool stillMoving;

   do {
      // Let motors stop spinning
      waitMS(1000);

      // Check if still moving
      uint32_t m1Position = Motor1::getPosition();
      uint32_t m2Position = Motor2::getPosition();
      stillMoving = ((oldM1Position != m1Position) || (oldM2Position != m2Position));
      oldM1Position = m1Position;
      oldM2Position = m2Position;

   } while (stillMoving);

   // Grippers off
   Gripper1::open();
   Gripper2::open();

   stopHere();
}

#if 0
/*
 * Test the effect of ODE HSE etc on output levels
 */
__attribute__((unused))
static void testODE() {
   Ftm0Channel<1>::enable();
   Ftm0Channel<1>::setPeriod(PWM_PERIOD);
   Ftm0Channel<1>::setDutyCycle(50);

   Ftm0Channel<1>::setPCR(PORT_PCR_MUX(4)|PORT_PCR_PS(0)|PORT_PCR_PE(0)|PORT_PCR_ODE(1)|PORT_PCR_DSE(1)); // 4V

   bool ode = false;
   bool dse = false;
   bool pe  = false;
   bool ps  = false;

   for(;;) {
      console.
         write("PS=").write(ps).
         write(",PE=").write(pe).
         write(",ODE=").write(ode).
         write(",DSE=").writeln(dse);
      Ftm0Channel<1>::setPCR(PORT_PCR_MUX(4)|PORT_PCR_PS(ps)|PORT_PCR_PE(pe)|PORT_PCR_ODE(ode)|PORT_PCR_DSE(dse));
      waitMS(2000);
      if (pe && dse && ode) {
         ps = !ps;
      }
      if (dse && ode) {
         pe = !pe;
      }
      if (dse) {
         ode = !ode;
      }
      dse = !dse;
   }
}

/**
 * Test encoders
 *
 * Just reports encoder values
 */
__attribute__((unused))
static void testEncoders() {
   Motor1::initialise();
   Motor2::initialise();
   DacOut::enable();

   for (;;) {
      DacOut::setValue(Motor2::getPosition());
      console.
      write("E1 =").write( Motor1::getPosition().
      write(", E2 =").writeln( Motor2::getPosition());
   }
}
#endif

static constexpr float pidInterval  = 500 * us;
static constexpr float kp           = 0.01f;
static constexpr float ki           = 0.000f;
static constexpr float kd           = 00.1f*pidInterval;


PID_T<Motor1::getPositionAsFloat, Motor1::setSpeed> pid1(kp, ki, kd, pidInterval, -30, +30, false);
PID_T<Motor2::getPositionAsFloat, Motor2::setSpeed> pid2(kp, ki, kd, pidInterval, -30, +30, true);

/**
 * Debug PID call-back
 * Uses TpA to check timing.
 */
void controller() {
   TpA::set();
   pid2.update();
   pid1.update();
   TpA::clear();
}

using Timer = Pit;
using TimerChannel = PitChannel<0>;

/**
 * Configure PID timer call-back
 */
void initialisePids() {
   Timer::configure(PitDebugMode_Stop);
   TimerChannel::setCallback(controller);
   TimerChannel::configure(pidInterval, PitChannelIrq_Enable);
   TimerChannel::enableNvicInterrupts(true, NvicPriority_Normal);

   pid1.setSetpoint(0);
   pid1.enable(false);//initialise turned off

   pid2.setSetpoint(0);
   pid2.enable(false);//initialise turned off
}

int testStep = 2000;

void testPid() {
   int position = 1;
   for(;;) {
      auto fn = [](){
//         double out = pid2.getOutput();
         console.
            write((int)pid2.getSetpoint()).write(" ").
            write((int)pid2.getInput()).write(" ").
            write((int)pid2.getError()).write(" ").
            writeln(pid2.getOutput());
//            writeln((int)out).setWidth(3).setPadding(Padding_LeadingZeroes).writeln((int)(out*1000)%1000).reset();
         return false;
      };
      position = -position;
//      int oldPosition = position;
//      do {
//         position = (2000*(rand()%4))-3000;
//      } while (position == oldPosition);
      pid2.setSetpoint(position*testStep);
      waitMS(1000, fn);
   }
}

/**
 * Calibrate motors and grippers
 */
static void calibrate() {
   console.writeln("Calibrating gripper 1");
   if (!Gripper1::calibrate()) {
      console.writeln("Failed calibrate gripper 1");
      shutDown();
   }

   console.writeln("Calibrating gripper 2");
   if (!Gripper2::calibrate()) {
      console.writeln("Failed calibrate gripper 2");
      shutDown();
   }

   console.writeln("Calibrating");
   console.write("Position gripper 1: Press any key to continue!");
   console.readChar();

   Motor1::Encoder::resetPosition();//Zero motor 1

   console.write("\nPosition gripper 2: Press any key to continue!");
   console.readChar();

   Motor2::Encoder::resetPosition();//Zero motor 1

   console.write("\n\n");

   /*console.writeln("Calibrating motor 1");
      if (!Motor1::calibrate()) {
	  console.writeln(Motor1::getPosition());
      console.writeln("Failed calibrate motor 1");
      console.writeln("Check motor was turn to correct starting position");
      shutDown();
   }


   console.writeln("Calibrating motor 2");
   if (!Motor2::calibrate()) {
      console.writeln("Failed calibrate motor 2");
      shutDown();
   }*/
}

/**
 * Check motor supply voltage
 */
void checkMotorSupply() {

   int voltage = round(10*MotorSupplySensor::motorVoltage());
   console.write("Motor voltage = ").write(voltage/10).write(".").writeln(voltage%10);

   if (!MotorSupplySensor::isOK()) {
      console.writeln("Insufficient motor power supply");
      stopHere();
   }
}

/*
 * Test motors
 */
void testMotors() {
   static constexpr int MAX_SPEED = 60;

   checkMotorSupply();

   Gripper1::initialise();
   Gripper2::initialise();

   Motor1::initialise();
   Motor2::initialise();

   DacOut::enable();

   auto rpt = []() {
      DacOut::setValue(Motor2::getPosition());
      console.
         write("E1 =").write(Motor1::getPosition()).
         write(", E2 =").writeln(Motor2::getPosition());
      return false;
   };
   for (;;) {

      // Ramp motor speed
      for (int speed=0; speed<=MAX_SPEED; speed++) {
         Motor1::setSpeed(speed);
         Motor2::setSpeed(-speed);
         waitMS(50, rpt);
      }
      for (int speed=MAX_SPEED; speed>=0; speed--) {
         Motor1::setSpeed(speed);
         Motor2::setSpeed(-speed);
         waitMS(50, rpt);
      }
      waitMS(500, rpt);

      // Ramp motor speed other way
      for (int speed=0; speed>=-MAX_SPEED; speed--) {
         Motor1::setSpeed(speed);
         Motor2::setSpeed(-speed);
         waitMS(50, rpt);
      }
      for (int speed=-MAX_SPEED; speed<=0; speed++) {
         Motor1::setSpeed(speed);
         Motor2::setSpeed(-speed);
         waitMS(50, rpt);
      }
      waitMS(500, rpt);
   }

}

/**
 * Test Grippers
 */
void testGrippers() {
   for (;;) {
      Gripper1::initialise();
      Gripper2::initialise();

      // Toggle gripper1
      console.writeln("Gripper 1");
      Gripper1::close();
      waitMS(4000);
      Gripper1::open();
      waitMS(1000);

      // Toggle gripper2
      console.writeln("Gripper 2");
      Gripper2::close();
      waitMS(4000);
      Gripper2::open();
      waitMS(1000);

      // Toggle both
      console.writeln("Gripper 1+2");
      Gripper1::close();
      Gripper2::close();
      waitMS(4000);
      Gripper1::open();
      Gripper2::open();
      waitMS(1000);
   }
}

/**
 * Initialise system
 */
void initialise() {
   TpA::setOutput();
   TpB::setOutput();

   DacOut::enable();

   MotorSupplySensor::initialise();

   checkMotorSupply();

   Gripper1::initialise();
   Gripper2::initialise();

   Motor1::initialise();
   Motor2::initialise();

   Motor1::Encoder::resetPosition();

   console.writeln(Motor1::getPosition());

   initialisePids();

   calibrate();

   waitMS(10);
}

double input      = 0.0;
double output     = 0.0;
double setPoint   = 0.0;

float motor2Position() {
   float position = Motor2::getPositionAsFloat();
   DacOut::setValue(position);
   return position;
};

bool readFromPC()
{
	bool result = false;

	/*Not standard library,
	 *Add readCharNoBlock method by copying readChar and changing check for (ch < 0)
	 *Return NULL instead of looping
	 */
	int ch = console.readCharNoBlock();

	if(ch == NULL)
	{
		result = false;
	}

	else//Character read in
	{
		char readCharacter = (char)ch;

		//Interpret user commands
		if((readCharacter == 's') || (readCharacter == 'S'))
		{
			stopHere();
		}

		else//Set outputs if not recognised as a command
		{
			readCommands.push_back(readCharacter);

			console.writeln();

			/*int i = 0;
			while(i < readCommands.size())
			{
				console.write(readCommands[i]);

				i += 1;
			}

			console.writeln();//*/

			result = true;
		}
	}

	return result;
}

enum TrackerState {Turning1, Turning2, Gripping1, Gripping2, Free, Stopped};

TrackerState currentTrackedState = Free;

u_int PIDUpdaterIndex = 0;

/*
 * Updates the pids and grippers as well as checking for steady states before updating them.
 * Returns true if an update was made
 */

constexpr int MOTOR_ACTION_OFFSET = 0;
constexpr int GRIPPER_ACTION_OFFSET = 2;

bool ControlUpdate(int actionArgument)
{
/*
 * -4 open gripper2
 * -3 open gripper1
 * -2 rotate motor2 -90 degrees
 * -1 rotate motor1 -90 degrees
 * 0 no action
 * 1 rotate motor1 90 degrees
 * 2 rotate motor2 90 degrees
 * 3 close gripper1
 * 4 close gripper2
 */
	bool result = false;

	bool steadyStateFound = false;

	if(currentTrackedState == Stopped)
	{
		stopHere();
	}

	//Set steady state
	//Seperate cheks to save resources
	if(currentTrackedState == Turning1)
	{
		steadyStateFound = pid1.getIsSteadyState(STEADY_STATE_TOLERANCE);
	}

	if(currentTrackedState == Turning2)
	{
		steadyStateFound = pid2.getIsSteadyState(STEADY_STATE_TOLERANCE);
	}

	if(currentTrackedState == Gripping1)
	{
		steadyStateFound = true;//Current configuration grippers are steady after return from call to open/close
	}

	if(currentTrackedState == Gripping2)
	{
		steadyStateFound = true;//Current configuration grippers are steady after return from call to open/close
	}


	//When steady states are achieved update tracker
	if(steadyStateFound)
	{
		currentTrackedState = Free;
	}

	else
	{
		result = false;
	}

	if(currentTrackedState == Free)
	{
		int actionToComplete = 0;


		/*if(PIDUpdaterIndex < interpretedActions.size())
		{
			actionToComplete = interpretedActions[PIDUpdaterIndex];//read from interpreted commands

			PIDUpdaterIndex ++;

		}

		else
		{
			result = false;
		}//*/

		actionToComplete = actionArgument;

		console.write("Actioning ").writeln(actionToComplete);

		if(actionToComplete == 0)
		{
			result = true;
		}

		else if(actionToComplete == 1) //Rotate Motor 1 90 degrees
		{
			currentTrackedState = Turning1;

			pid1.setSetpoint(pid1.getSetpoint() + QUARTERROTATIONTICKS);

			result = true;
		}

		else if(actionToComplete == -1) //Rotate Motor 1 -90 degrees
		{
			currentTrackedState = Turning1;

			pid1.setSetpoint(pid1.getSetpoint() - QUARTERROTATIONTICKS);

			result = true;
		}

		else if(actionToComplete == 2) //Rotate Motor 2 90 degrees
		{
			currentTrackedState = Turning2;

			pid2.setSetpoint(pid2.getSetpoint() + QUARTERROTATIONTICKS);

			result = true;
		}

		else if(actionToComplete == -2) //Rotate Motor 2 -90 degrees
		{
			currentTrackedState = Turning2;

			pid2.setSetpoint(pid2.getSetpoint() - QUARTERROTATIONTICKS);

			result = true;
		}

		else if(actionToComplete == 3) //Close gripper1
		{
			currentTrackedState = Gripping1;

			Gripper1::close();

			result = true;
		}

		else if(actionToComplete == 4) //Close gripper2
		{
			currentTrackedState = Gripping2;

			Gripper2::close();

			result = true;
		}

		else if(actionToComplete == -3) //Open gripper1
		{
			currentTrackedState = Gripping1;

			Gripper1::open();

			result = true;
		}

		else if(actionToComplete == -4) //Open gripper2
		{
			currentTrackedState = Gripping2;
			Gripper2::open();

			result = true;
		}
	}

	return result;
}

//Cube tracking variables
enum Face {Front, Back, Left, Right, Up, Down, None};

Face grippedFace1 = Front; //---change when initial state known
Face grippedFace2 = Down;  //--- ^as above

//									 Front = 2      Back = 2       Left = 2       Right = 2      Up = 2         Down = 2
const int cubePositions[6][6][6] = {{{0,0,0,0,0,0}, {0,0,0,0,0,0}, {1,6,2,5,4,3}, {1,6,5,2,3,4}, {1,6,3,4,2,5}, {1,6,4,3,5,2}},  //Front = 1
									{{0,0,0,0,0,0}, {0,0,0,0,0,0}, {6,1,2,5,3,4}, {6,1,5,2,4,3}, {6,1,4,3,2,5}, {6,1,3,4,5,2}},  //Back = 1
									{{2,5,1,6,3,4}, {5,2,1,6,4,3}, {0,0,0,0,0,0}, {0,0,0,0,0,0}, {4,3,1,6,2,5}, {3,4,1,6,5,2}},  //Left = 1
									{{2,5,6,1,4,3}, {5,2,6,1,3,4}, {0,0,0,0,0,0}, {0,0,0,0,0,0}, {3,4,6,1,2,5}, {4,3,6,1,5,2}},  //Right = 1
									{{2,5,4,3,1,6}, {5,2,3,4,1,6}, {3,4,2,5,1,6}, {4,3,5,2,1,6}, {0,0,0,0,0,0}, {0,0,0,0,0,0}},  //Up = 1
									{{2,5,3,4,6,1}, {5,2,4,3,6,1}, {4,3,2,5,6,1}, {3,4,5,2,6,1}, {0,0,0,0,0,0}, {0,0,0,0,0,0}}}; //Down = 1

/*
 *Returns position of requested face, as given by Danny's scheme
 * 1   - Gripper 1
 * 2   - Gripper 2
 * 3/4 - Adjacent 1&2
 * 5   - Above 1
 * 6   - Above 2
 *
 * Returns 0 on failure.
 */
int LookupFace(Face f)
{
	int result = 0;
	
	if(f != None){
	//First, check trivial case of currently gripped face
	//if(grippedFace1 == f) {
	//	result = 1;
	//} else if(grippedFace2 == f) {
	//	result = 2;
	//} else {
		//24 permutations of possible gripped faces
		//Thus, a simple lookup array is used to minimise computation
		result = cubePositions[grippedFace1][grippedFace2][f];
	//}
	}
	
	return result;
}

//The offset of the motor in right turns at completion of the current interpreted commands
int plannedOffsetMotor1 = 0;
int plannedOffsetMotor2 = 0;

int constexpr MAXPLANNEDOFFSET = 3;
int constexpr MINPLANNEDOFFSET = -MAXPLANNEDOFFSET;

u_int commandInterpreterIndex = 0;

/*
 *Reads the next entry from commands if one exists and converts it to an interpreted set of commands
 *
 * Returns true if an entry was read
 *
 */
bool interpretCommand()
{
	int constexpr rewind1[4] = {-3, -1, -1, 3};//Sequence for putting the position of motor1 back two indexes
	int constexpr rewind2[4] = {-4, -2, -2, 4};//Sequence for putting the position of motor2 back two indexes
	int constexpr fastforward1[4] = {-3, 1, 1, 3};//Sequence for putting the position of motor1 forward two indexes
	int constexpr fastforward2[4] = {-4, 2, 2, 4};//Sequence for putting the position of motor2 forward two indexes

	bool result = false;

	if(commandInterpreterIndex < readCommands.size())
	{
		char toInterpret = readCommands[commandInterpreterIndex];
		commandInterpreterIndex ++;

		std::vector<int> commands;
		
		Face f = None;
		int position = 0;
		bool CCW; //true = -90 deg, false = +90 deg

		//Determine face and direction
		switch(toInterpret)
		{
			case 'f':
				CCW = true;
				f = Front;
				break;
			case 'F':
				CCW = false;
				f = Front;
				break;
			case 'b':
				CCW = true;
				f = Back;
				break;
			case 'B':
				CCW = false;
				f = Back
				break;
			case 'l':
				CCW = true;
				f = Left
				break;
			case 'L':
				CCW = false;
				f = Left
				break;
			case 'r':
				CCW = true;
				f = Right
				break;
			case 'R':
				CCW = false;
				f = Right
				break;
			case 'u':
				CCW = true;
				f = Up
				break;
			case 'U':
				CCW = false;
				f = Up
				break;
			case 'd':
				CCW = true;
				f = Down
				break;
			case 'D':
				CCW = false;
				f = Down
				break;
			case 'x':
				CCW = true;
				//todo
				break;
			case 'X':
				CCW = false;
				//todo
				break;
			case 'y':
				CCW = true;
				//todo
				break;
			case 'Y':
				CCW = false;
				//todo
				break;
			case 'z':
				CCW = true;
				//todo
				break;
			case 'Z':
				CCW = false;
				//todo
				break;
			default:
				console.write(toInterpret).writeln(" couldn't be interpreted");
				break;
		}
		
		//Determine necessary moves
		position = LookupFace(f);
		
		//note - add check for gripper clashes
		switch(position)
		{
			case 1:
				//No extra commands necessary
				commands.push_back(3);
				commands.push_back(4);
				commands.push_back(1*pow(-1, (int)CCW));
				break;
			case 2:
				//No extra commands necessary
				commands.push_back(4);
				commands.push_back(3);
				commands.push_back(2*pow(-1, (int)CCW));
				break;
			case 3:
				//One extra rotation required
				commands.push_back(4);
				commands.push_back(-3);
				commands.push_back(-2);
				
				commands.push_back(3);
				commands.push_back(1*pow(-1, (int)CCW));
				grippedFace1 = f;
				break;
			case 4:
				//One extra rotation required
				commands.push_back(3);
				commands.push_back(-4);
				commands.push_back(-1);
				
				commands.push_back(4);
				commands.push_back(2*pow(-1, (int)CCW));
				grippedFace2 = f;
				break;
			case 5:
				//Two extra rotations required
				commands.push_back(3);
				commands.push_back(-4);
				commands.push_back(1);
				commands.push_back(1);
				
				commands.push_back(4);
				commands.push_back(2*pow(-1, (int)CCW));
				grippedFace2 = f;
				break;
			case 6:
				//Two extra rotations required
				commands.push_back(4);
				commands.push_back(-3);
				commands.push_back(2);
				commands.push_back(2);
				
				commands.push_back(3);
				commands.push_back(1*pow(-1, (int)CCW));
				grippedFace1 = f;
				break;
			default:
				console.write(toInterpret).writeln(" couldn't be located");
				shutDown();
				break;
		}

		//if commands were established
		if(commands.size() != 0)
		{
			//Integrity check for out of bounds moves
			u_int length = commands.size();

			for(u_int i = 0; i < length; i++)
			{
				//Update world model for motor offsets
				switch(commands[i])
				{
					case -1:
						if(plannedOffsetMotor1 == MINPLANNEDOFFSET)
						{
							const int *precalc = fastforward1;//Pointer to a (const int) not a constant pointer to an int

							for(u_int j = 0; j < sizeof(precalc); j++)
								interpretedActions.push_back(precalc[j]);

							plannedOffsetMotor1 += 2;//two moves in fastforward
						}

						interpretedActions.push_back(commands[i]);

						plannedOffsetMotor1 -= 1;
						break;
					case 1:
						if(plannedOffsetMotor1 == MAXPLANNEDOFFSET)
						{
							const int *precalc = rewind1;//Pointer to a (const int) not a constant pointer to an int

							for(u_int j = 0; j < sizeof(precalc); j++)
								interpretedActions.push_back(precalc[j]);

							plannedOffsetMotor1 -= 2;//two moves in rewind
						}

						interpretedActions.push_back(commands[i]);

						plannedOffsetMotor1 += 1;
						break;
					case -2:
						if(plannedOffsetMotor2 == MINPLANNEDOFFSET)
						{
							const int *precalc = fastforward2;//Pointer to a (const int) not a constant pointer to an int

							for(u_int j = 0; j < sizeof(precalc); j++)
								interpretedActions.push_back(precalc[j]);

							plannedOffsetMotor2 += 2;//two moves in fastforward
						}

						interpretedActions.push_back(commands[i]);

						plannedOffsetMotor2 -= 1;
						break;
					case 2:
						if(plannedOffsetMotor2 == MAXPLANNEDOFFSET)
						{
							const int *precalc = rewind2;//Pointer to a (const int) not a constant pointer to an int

							for(u_int j = 0; j < sizeof(precalc); j++)
								interpretedActions.push_back(precalc[j]);

							plannedOffsetMotor2 -= 2;//two moves in rewind
						}

						interpretedActions.push_back(commands[i]);

						plannedOffsetMotor2 += 1;
						break;
					case -3:
						interpretedActions.push_back(commands[i]);
						break;
					case 3:
						interpretedActions.push_back(commands[i]);
						break;
					case -4:
						interpretedActions.push_back(commands[i]);
						break;
					case 4:
						interpretedActions.push_back(commands[i]);
						break;
				}
			}
		}
	}

	return result;
}

//
void thread1()
{
	//Reading from PC
	readFromPC();

	//Check PIDs
	ControlUpdate(0);

	//Interpret commands
	interpretCommand();

	//Check PIDs
	ControlUpdate(0);
}

int main() {
   console.writeln("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");//clear screen

   console.writeln("Starting");

   console.write("Core clock = ").writeln(::SystemCoreClock);
   console.write("Bus clock  = ").writeln(::SystemBusClock);

   initialise();

//   while(true)
//   {
//	   thread1();
//   }


pid1.setTunings(5.0f, 0.1f, 0.01f);
pid1.enable(true);
pid1.setSetpoint(0);

  pid2.setTunings(5.0f, 0.1f, 0.01f);
  pid2.enable(true);
  pid2.setSetpoint(0);

  //demo
  console.readChar();
  Gripper1::close();
  console.readChar();
  Gripper2::close();
  console.readChar();

  //Demo

  while(!ControlUpdate(1));

  while(!ControlUpdate(0));

  console.readChar();

  while(!ControlUpdate(1));

  while(!ControlUpdate(0));

  console.readChar();

  while(!ControlUpdate(-3));

  while(!ControlUpdate(0));

  console.readChar();

  while(!ControlUpdate(-1));

  while(!ControlUpdate(0));

  console.readChar();

  while(!ControlUpdate(-1));

  while(!ControlUpdate(0));

  console.readChar();

  while(!ControlUpdate(3));

  while(!ControlUpdate(0));

//while(true)
//{
////	console.write(Motor1::getPosition()).write(", ").write(pid1.getError()).write(", ").write(pid1.getOutput()).writeln();
////
////	if(pid1.getIsSteadyState(STEADY_STATE_TOLERANCE))
////	{
////		pid1.setSetpoint(-pid1.getSetpoint());
////	}
//
//	console.write(Motor2::getPosition()).write(", ").write(pid2.getError()).write(", ").write(pid2.getOutput()).writeln();
//
//	if(pid2.getIsSteadyState(STEADY_STATE_TOLERANCE))
//	{
//		pid2.setSetpoint(-pid2.getSetpoint());
//	}
//}

  console.writeln("Blocking");

  console.readChar();

   return 0;
}


#endif /* PROJECT_MAIN_ */
