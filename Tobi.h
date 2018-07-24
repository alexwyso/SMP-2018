/*   Tobi library
Created by Cherag Bhagwagar
10/21/2016

/////////////////////////////////////
 $ Changes to code by Andrea Frank
	marked with $ before comment.
	7/21/17
/////////////////////////////////////

/////////////////////////////////////
$ Changes to code by Alex Wysoczanski
	marked with $$ before comment.
	7/21/17
/////////////////////////////////////

*/
*/

#ifndef Tobi_h 
#define Tobi_h 
 
#include "Arduino.h"
#include <Wire.h>
#include "math.h"

#define NUM_MOTORS 6	// Number of usable motors
#define NUM_WHEELS 2	// Number of wheels 
#define NUM_AXES 3		// Number of usable axes

class Tobi
{
	
	/***************************************************************************************/
	/***************************************************************************************/
	/************************************ $ PUBLIC *****************************************/
	/***************************************************************************************/
	/***************************************************************************************/

	public:	

		int maxEncoderVals[NUM_MOTORS];		// stores max encoder vals; originally set to 1023 by Tobi::enable(),
												// and updated during calibrateEncoders()

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
		UPDATED:	- maxEncoderVals initialized.
		EFFECTS:	- Writes 0 to pins __pcf1 and __pcf2, and 0 to all pins in __pwmPins. Turns on all axis and sets motor
					directions to default. Cascades LEDs.
		*/
		void enable(void);

		/*  $$         DISABLE
			Disable TOBI. Set all legs to speed 0, unpower all axes, and clear __byte1 and __byte2.
			INPUTS: 	- None.
			OUTPUTS: 	- None.
			UPDATED:	- __byte1, __byte2.
		*/
		void disable(void);

		/////////////////////////// MOTION ////////////////////////////

		/*  $$         SETMOTOR
			Set motor direction. Uses PCF io expander and bit shifts.
			INPUTS: 	- (int) motor, (int) direction.
			OUTPUTS: 	- None
			UPDATED:	- __byte1.
		*/
		void setMotor(int motor , int direction);

		/*  $$         FLIPMOTOR
			Flip motor direction. Uses PCF io expander and bit shifts.
			INPUTS: 	- (int) motor
			OUTPUTS: 	- None
			UPDATED:	- __byte1.
		*/
		void flipMotor(int motor);

		/*  $$         POWERAXIS
			Turn motor axis on or off. Can only control power to motors in pairs (axes),
				M0-M1 (axis 0), M2-M3 (axis 1), and M4-M5 (axis 2).
			INPUTS: 	- (int) axis, (int) state.
			OUTPUTS: 	- None
			UPDATED:	- __byte1, __byte2.
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
		void calibrateEncoders();
		void calibrateEncoders(int* encoderIndices);

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

		/*  $$         LED
			Turn onboard LED (0,1,2,3,4,5) on (1) or off (0). Note: for state, any nonzero number will be treated as on.
			INPUTS: 	- (int) led, (int) state.
			OUTPUTS: 	- None
			UPDATED:	- __byte1, __byte2. 
		*/
		void led(int led, int state);
		
		/*  $         NOSELED
			Turn nose LED off (state = 0) or on (state = 1).
			INPUTS: 	- (int) state.
			OUTPUTS: 	- None.
			UPDATED:	- __byte2.
		*/
		void noseLed(int state);
		
		///////////////// $ UNFINISHED OR MISSING METHODS ///////////////
		void print (void);
		void print_raw(void);

	/***************************************************************************************/
	/***************************************************************************************/
	/*********************************** $ PRIVATE *****************************************/
	/***************************************************************************************/
	/***************************************************************************************/

	private:
		/////////////////////////// $ FIELDS ////////////////////////////
		
		int __pwmPins[6] ; 		// $ Addresses of pins for writing the pwm of each motor
		int __pwmVal[6];		// $ Values of pwm for each motor, updated in setPwm()
		int __encoderPins[6];	// $ Addresses of pins for reading the value of each encoder for each leg
		int __encoderVal[6] ; 	// $ Values of encoders read from each leg, updated with analogUpdate() method
		
		////////////////////////// $ HELPER METHODS ////////////////////////////
		
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
