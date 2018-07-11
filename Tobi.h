/*   Tobi library
Created by Cherag Bhagwagar
10/21/2016

/////////////////////////////////////
 $ Changes to code by Andrea Frank
	marked with $ before comment.
	7/21/17
/////////////////////////////////////

/////////////////////////////////////
$ Additional methods from libraries
	written by Andrea Frank (TobiPro,
	TobiCustom, and TobiFilterManager)
	added to this library by Alex
	Wysoczanski, marked by $$ before 
	function headers.
	7/2/18
/////////////////////////////////////

*/


#ifndef Tobi_h 
#define Tobi_h 
 
#include "Arduino.h"
#include <Wire.h>
#include "math.h"

#define MAX_NUM_MOTORS 6

#define NUM_MOTORS 6	// number of usable motors
#define NUM_AXES 2		// number of usable axes (right now 2, because third axis is connected to 
						// motor 5, which is wired weirdly to connect to reset sequence)

const float default3dbFreq = 0.25;
const float defaultWinLen = 50;

enum STATS_TYPE {
	INPUT_STATS,
	FILTER_STATS
};

class Tobi
{
	
	/***************************************************************************************/
	/***************************************************************************************/
	/************************************ $ PUBLIC *****************************************/
	/***************************************************************************************/
	/***************************************************************************************/

	public:	

		/////////////////////////// SETUP ////////////////////////////

		/*  $         CONSTRUCTOR
			Initialize all variables on this instance.
			INPUTS: 	- None
			OUTPUTS: 	- None
			UPDATED: 	__pwmPins, __encoderPins
		*/
		Tobi();

		/*  $         ENABLE
			Set up and enable hardware. Sets IO pins to proper direction, writes 0 to two pcf pins, sets default motor
		directions, turn son power for each axis, and writes pwm to 0 for each motor. It then cascades LEDs on
		and off and prints a complete statement to Serial (if enabled).
		NOTE: DOES NOT ENABLE AXIS 2 (M4-M5), AS THIS LINKS MOTOR 5 TO UPLOAD SEQUENCE (spins with flashing LEDS AND
		DOES NOT ACTIVATE MOTOR. Direction and pwm are still set for motors 4 and 5, but axis is not powered. If
		use of motor 4 and 5 is desired, it should be powered separately in code. This will make it spin during upload.
		INPUTS: 	- None
		OUTPUTS: 	- None
		UPDATED:	- No variables updated, but writes 0 to pins __pcf1 and __pcf2, and 0 to all pins in __pwmPins. 
								Also turns on all axis and sets motor directions to default.
		*/
		void enable(void);

		/*  $         DISABLE
			Disable TOBI. Set all legs to speed 0, clear __bit1 and __bit2.
			INPUTS: 	- None.
			OUTPUTS: 	- None.
			UPDATED:	- __bit1, __bit2.
		*/
		void disable();

			/////////////////////////// MOTION ////////////////////////////

		/*  $         SETMOTOR
			Set motor direction. Based on our test-TOBI, motors (0,1,3,5) should be set to -1,
				and motors (2,4) should be set to 1. Uses PCF io expander and bit shifts.
			INPUTS: 	- (int) motor, (int) direction.
			OUTPUTS: 	- None
			UPDATED:	- __bit1.
		*/
		void setMotor(int motor , int direction);

		/*  $         POWERAXIS
			Turn motor axis on or off. Can only control power to motors in pairs (axes),
				M0-M1 (axis 0), M2-M3 (axis 1), and M4-M5 (axis 2).
			INPUTS: 	- (int) axis, (int) state.
			OUTPUTS: 	- None
			UPDATED:	- __bit1, __bit2.
		*/
		void powerAxis(int axis, int state);

		/*  $         SETPWM
			Set PWM of motor from 0 - 255.
			INPUTS: 	- (int) motor, (int) pwm.
			OUTPUTS: 	- None.
			UPDATED:	- None.
		*/
		void setPwm(int motor, int pwm);

		/*  $         GETPWM
			Returns PWM of desired motor from 0 - 255.
			INPUTS: 	- (int) motor
			OUTPUTS: 	- None.
			UPDATED:	- None.
		*/
		int getPwm(int motor);
				
		///////////////////////// READ ENCODERS /////////////////////////

		/*  $         CALIBRATEENCODERS
			Finds max values of each encoder and saves them to an array passed
			in by reference. maxEncoderVals MUST be an array of size numEncoders.
			If encoders are not numbered 0-numEncoders, must pass in a third
			argument that is a pointer to an array that indexes encoder numbers.
			INPUTS: 	- int* maxEncoderVals, int numEncoders (optional int* encoderIndices)
			OUTPUTS: 	- None.
			UPDATED:	- maxEncoderVals
		*/
		void calibrateEncoders(int* maxEncoderVals, int numEncoders);
		void calibrateEncoders(int* maxEncoderVals, int numEncoders, int* encoderIndices);

		/*  $         READENCODER
			Call to read the value reported by the encoder for a specific leg. Method is necessary
				because __encoderVal is private. Calls _analogUpdate() before polling values. Encoder
				values usually range from 0-1023. However, it is good practice to use calibrateEncoders()
				to determine the true max value for each encoder, as occasionally an encoder will roll
				back to 0 at ~600 instead of 1023.
			INPUTS: 	- (int) leg 	-> 	number 0 to 5 to specify which leg to query.
			OUTPUTS: 	- (int) value 	-> 	number 0 to 1023 from encoder, corresponds to motor angle.
			UPDATED:	- encoderVal
		*/
		int readEncoder(int motor);

		/////////////////////////// LEDS ////////////////////////////

		/*  $         LED
			Turn onboard LED (0,1,2,3,4,5) on (1) or off (0). Note: for state, any nonzero number will be treated as on.
			INPUTS: 	- (int) led, (int) state.
			OUTPUTS: 	- None
			UPDATED:	- __bit1, __bit2. 
		*/
		void led(int led, int state);
		
		/*  $         NOSELED
			Turn nose LED off (state = 0) or on (state = 1).
			INPUTS: 	- (int) state.
			OUTPUTS: 	- None.
			UPDATED:	- __bit2.
		*/
		void noseLed(int state);
		
		///////////////// $ UNFINISHED OR MISSING METHODS ///////////////
		void print (void);
		void print_raw(void);

		/////////////////////////// $$ TOBI_PRO METHODS $$ ////////////////////////////

		/*          SERIALSETUP()
		Set up Serial communication. Begins Serial communication at 9600 baud, sets timeout at
		10 ms, waits 5 seconds for Serial comm to begin before returning and allowing program
		to continue.
		INPUTS:   - None.
		OUTPUTS:  - None.
		UPDATES:  - None.
		EFFECTS:  - Serial communication set up.
		*/
		void serialSetup(void);

		/*      SETMOTORINDICES()
		If active motors are not numbered simply 0 through NUM_MOTORS, this method can be used to
		set TobiPro::_motorIndices to the correct indices for the active motors.
		INPUTS:   - int* motInd (pointer to an integer array of size NUM_MOTORS corresponding to
		the motors that are active for this TobiPro).
		OUTPUTS:  - None.
		UPDATES:  - TobiPro::_motorIndices
		EFFECTS:  - None.
		*/
		void setMotorIndices(int* motInd);

		/*      UPDATE()
		Calculates speed and updates filters for each motor.
		INPUTS:   - None.
		OUTPUTS:  - None.
		UPDATES:  - TobiPro::_motorSpeed.
		EFFECTS:  - None.
		*/
		void update(void);
		
		/*      CALCSPEED()
		Get the speed of a particular motor. Returns filtered speed if filter is on, and raw speed if filter
		is off. The speed is returned in units of change in encoder values per second.
		INPUTS:   - int motor
		OUTPUTS:  - int rawSpeed
		UPDATES:  - None.
		EFFECTS:  - None.
		*/
		int calcSpeed(int motor);
		
		/*      GETSPEED()
		Returns the speed of the specified motor in encoder_vals/sec.
		INPUTS:   - int motor.
		OUTPUTS:  - int motorSpeed.
		UPDATES:  - None.
		EFFECTS:  - None.
		*/
		int getSpeed(int motor);
		
		/*      SETSPEED()
		Sets speed of a motor to a certain percentaghe of its maximum speed.
		INPUTS:   - int motor, float percent.
		OUTPUTS:  - None.
		UPDATES:  - TobiPro::_motorSpeed[motor].
		EFFECTS:  - Changes servo angular velocity.
		*/
		void setSpeed(int motor, float percent);
		
		/*      SETSAMPLERATE()
		Sets sample rate of TobiPro object.
		INPUTS:   - int Fs (sample rate, in Hz).
		OUTPUTS:  - None.
		UPDATES:  - TobiPro::_dt.
		EFFECTS:  - None.
		*/
		void setSampleRate(int dtInMs);
		
		/*      FILTERINPUTS()
		Sets TobiPro to filter or not filter encoder values with a two pole low pass filter, defined
		in TobiFilterManager.
		INPUTS:   - bool onOff (true for filtering on, false for filtering off).
		OUTPUTS:  - None.
		UPDATES:  - TobiPro::filter.
		EFFECTS:  - None.
		*/
		void filterInputs(bool onOff);
		
		/*      SRDELAY()
		Pauses code execution for the period corresponding to TobiPro::_dt, the inverse of your sample rate.
		INPUTS:   - None.
		OUTPUTS:  - None.
		UPDATES:  - None.
		EFFECTS:  - Suspends code execution for 1/Fs seconds (TobiPro::_dt).
		*/
		void srDelay(void);	

		/////////////////////////// $$ TOBI_FILTER_MANAGER METHODS $$ ////////////////////////////

		

		/////////////////////////// $$ TOBI_CUSTOM METHODS $$ ////////////////////////////

		/*  $         DRIVEFWD
		Sets pwm of wheel motors to drive forward at same speed, so TOBI bot moves 
		directly forward. 
		INPUTS: 	- (int) speed	-> 	number 0 to 100 in percentage of max speed.
		OUTPUTS: 	- None
		UPDATED:	- - _pwmPin[0] and _pwmPin[1]
		EFFECTS:	- Drives TOBI forward
		*/
		void driveFwd(int speed);

	/***************************************************************************************/
	/***************************************************************************************/
	/*********************************** $ PRIVATE *****************************************/
	/***************************************************************************************/
	/***************************************************************************************/

	private:
		/////////////////////////// $ TOBI FIELDS ////////////////////////////
		
		int __pwmPins[6] ; 		// $ addresses of pins for writing the pwm of each motor
		int __pwmVal[6];		// $ values of pwm for each motor, updated in setPwm()
		int __encoderPins[6];	// $ addresses of pins for reading the value of each encoder for each leg
		int __encoderVal[6] ; 	// $ values of encoders read from each leg, updated with analogUpdate() method

		/////////////////////////// $$ TOBI_PRO FIELDS $$ ////////////////////////////

		int _motorSpeed[MAX_NUM_MOTORS];	// stores the motor speed of each motor
		int _dt = 50;						// default sample rate is 50 ms
		int _lastEnc[NUM_MOTORS];			// save past encoder values (for calculating speed)
		int _lastTime[NUM_MOTORS];			// 
		int _motorIndices[NUM_MOTORS];		//

		/////////////////////////// $$ TOBI_FILTER_MANAGER FIELDS $$ ////////////////////////////
		
		////////////////////////// $ METHODS ////////////////////////////
		
		/*  $         ANALOGUPDATE
			Read encoder values into __encoderVal array for easy access. Should be called before 
				acting upon __encoderVal values.
			INPUTS: 	- None
			OUTPUTS: 	- None
			UPDATED:	- __encoderVal
		*/
		void _analogUpdate(void);

		/*  $         WRITE8
			Write one byte to chosen address. Used to write to PCF expander.
			INPUTS: 	- (int) address, (byte) value.
			OUTPUTS: 	- None.
			UPDATED:	- None.
		*/
		void __write8(int address, byte data);
		
};

#endif

