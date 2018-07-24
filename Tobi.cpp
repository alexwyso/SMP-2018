/* Tobi.cpp
Created by Cherag Bhagwagar 
10/21/2016

/////////////////////////////////////
 $ Changes to code by Andrea Frank
	marked with $ before comment.
	7/18/17
/////////////////////////////////////

/////////////////////////////////////
$ Changes to code by Alex Wysoczanski
	marked with $$ before comment.
	7/23/18
/////////////////////////////////////

*/

/***************************************************************************************/
/***************************************************************************************/
/****************************** $ GLOBAL VARIABLES *************************************/
/***************************************************************************************/
/***************************************************************************************/

// $$ Link the header file
#include "Tobi.h"

// $ Placeholder pointer
void *__ptr1; 		 

// $ Addresses for PCF IO expander
#define __pcf2 57	
#define __pcf1 56

// $$ Byte used to store direction of each motor at bit index
byte __byte1 = 0;	

// $$ Byte used to store LED state (Unconfirmed)
byte __byte2 = 0;

/***************************************************************************************/
/***************************************************************************************/
/************************************ $ METHODS ****************************************/
/***************************************************************************************/
/***************************************************************************************/

/////////////////////////// SETUP ////////////////////////////

/*  $         CONSTRUCTOR
	Initialize all variables on this instance.
	INPUTS: 	- None
	OUTPUTS: 	- None
	UPDATED: 	Tobi::__pwmPins, Tobi::__encoderPins
*/
Tobi::Tobi()
{
	// $$ Turn on nose LED
	digitalWrite(13,HIGH);		

	// $$ Assign pins values to each motor
	Tobi::__pwmPins[0] = 9;
	Tobi::__pwmPins[1] = 5; 		
	Tobi::__pwmPins[2] = 6;
	Tobi::__pwmPins[3] = 10;
	Tobi::__pwmPins[4] = 11;
	Tobi::__pwmPins[5] = 13;		

	// $$ Turn off nose LED
	digitalWrite(13,LOW);		

	// $$ Assign pin values to each encoder
	Tobi::__encoderPins[0] = 0;
	Tobi::__encoderPins[1] = 1;
	Tobi::__encoderPins[2] = 2;
	Tobi::__encoderPins[3] = 3;
	Tobi::__encoderPins[4] = 4;
	Tobi::__encoderPins[5] = 5;

	// $$ Turn on nose LED
	digitalWrite(13,HIGH); 		                                                                     
}

/*  $$         ENABLE
	Set up and enable hardware. Sets IO pins to proper direction, writes 0 to two pcf pins, sets default motor
		directions, turns on power for each axis, and writes pwm to 0 for each motor. It then cascades LEDs on
		and off and prints a complete statement to Serial if successfully enabled.
	NOTE: DOES NOT ENABLE AXIS 2 (M4-M5), AS THIS LINKS MOTOR 5 TO UPLOAD SEQUENCE (spins with flashing LEDS AND
		DOES NOT ACTIVATE MOTOR. Direction and pwm are still set for motors 4 and 5, but axis is not powered. If
		use of motor 4 and 5 is desired, it should be powered separately in code. This will make it spin during upload.
	INPUTS: 	- None
	OUTPUTS: 	- None
	UPDATED:	- maxEncoderVals initialized.
	EFFECTS:	- Writes 0 to pins __pcf1 and __pcf2, and 0 to all pins in Tobi::__pwmPins. Turns on all axis and sets motor
					directions to default. Cascades LEDs.
*/
void Tobi::enable(void){
	if(Serial) Serial.println("Enabling TOBI test...");

	// $ Set PWM pins to output 
	for (int i = 0 ; i < NUM_MOTORS; i ++){
		pinMode(Tobi::__pwmPins[i],OUTPUT);
	}

	// $ Set encoder pins to input
	for (int i = 0 ; i < NUM_MOTORS; i ++){
		pinMode(Tobi::__encoderPins[i],INPUT);
	}

	// $$ Initialize maxEncoderVals to default max
	for (int i =0; i < NUM_MOTORS; i++){
		maxEncoderVals[i] = 1023;		
	}

	// $ Write 0 to IO expanders
	Wire.begin();
	Tobi::__write8(__pcf1,0);
	Tobi::__write8(__pcf2,0);

	// $$ Set all the pins to 0 output
	for(int i = 0 ; i < NUM_MOTORS; i ++) {		
		analogWrite(Tobi::__pwmPins[i], 0);	
	}
	
	// $ Turn all motor axes off originally to avoid issues with setting direction and pwm
	for (int i = 0; i < 3; i++){
		Tobi::powerAxis(i,0);
	}

	// $ Set up motors initially in forward direction, pwm = 0
	for (int i = 0; i < NUM_MOTORS; i++){
		// Set motors in default direction
		Tobi::setMotor(i,1);	
		// Set duty cycle to 0
		Tobi::setPwm(i,0);			
	}

	// $$ Turn axis power on
	for (int i = 0; i < NUM_AXES; i++){
		Tobi::powerAxis(i,1);
	}

	// $$ Cascade LEDS on		
  	for (int i = 0 ; i < 6 ; i ++){
  		Tobi::led(i,1);
  		delay(100);
  	}

	// $$ Cascade LEDS on	
	for(int i = 6 ; i >=0 ; i --) {
		Tobi::led(i,0);
  		delay(100);	
	}

	// $$ Print a success message when enabled
	if(Serial) Serial.println("TOBI enabled.\n");
}


/*  $         DISABLE
	Disable TOBI. Set all legs to speed 0, unpower all axes, and clear __byte1 and __byte2.
	INPUTS: 	- None.
	OUTPUTS: 	- None.
	UPDATED:	- __byte1, __byte2.
*/
void Tobi::disable(){

	// $$ Turn off axis power
	for(int i = 0; i < 3; i++){
		Tobi::powerAxis(i,0);
	}

	// $$ Turn the wheel speeds to 0
	for(int i  = 0 ; i < NUM_MOTORS ; i ++){
		analogWrite(Tobi::__pwmPins[i], 0);
	}

	// $$ Clear the motor and LED state storage bytes
	__byte2 = 0 ;
	__byte1 = 0 ;

	// $$ Print a success message when disabled
	if(Serial) Serial.println("TOBI disabled.\n");
}

/////////////////////////// MOTION ////////////////////////////

/*  $         SETMOTOR
	Set motor direction. Uses PCF io expander and bit shifts.
	INPUTS: 	- (int) motor, (int) direction.
	OUTPUTS: 	- None
	UPDATED:	- __byte1.
*/
void Tobi::setMotor(int motor, int direction){

	// $ Check for valid motor number input
	if ((motor < 0) && (motor > 5)){
		Serial.println("Wrong Motor");
		return;
	}

	// $$ Check for disired direction
	if (direction == 1) {
		// $$ Assign the corresponding bit for the input motor to 1
		__byte1 |= (1 << motor);
	} else if (direction == -1) {
		// $$ Assign the corresponding bit for the input motor to 1
		__byte1 &= ~(1 << (motor));
	}
	else {
		// $$ Otherwise print an error message
		Serial.println("Invalid direction. Must choose 1 or -1.");
	}

	// $ Write __byte1 to __pcf1 through PCF io expander.
    Tobi::__write8(__pcf1,__byte1);	
}

/*  $$         FLIPMOTOR
Flips motor direction. Uses PCF io expander and bit shifts.
INPUTS: 	- (int) motor
OUTPUTS: 	- None
UPDATED:	- __byte1 (via setMotor).
*/
void Tobi::flipMotor(int motor) {

	// $$ Check for the current direction based on the bit shift
	if (((__byte1 >> motor) & 1) == 0) {
		// $$ Set the motor to the opposite direction
		Tobi::setMotor(motor,1);
	} else {
		Tobi::setMotor(motor,-1);
	}
}

/*  $         POWERAXIS
	Turn motor axis on or off. Can only control power to motors in pairs (axes),
		M0-M1 (axis 0), M2-M3 (axis 1), and M4-M5 (axis 2).
	INPUTS: 	- (int) axis, (int) state.
	OUTPUTS: 	- None
	UPDATED:	- __byte1, __byte2.
*/
void Tobi::powerAxis (int axis, int state){

	// $$ Check for axis input 0, 1, or 2
    switch (axis){
    	case 0 : 
			// $$ If you want to power on, set the corresponding motor bits to 1
	    	if (state == 1)			__byte1 |= (1<<6);
			// $$ If you want to power off, set the corresponding motor bits to 0
	    	else if (state == 0)	__byte1 &= ~(1<<6);
			// $$ Write the new byte value to PCF1
	    	Tobi::__write8(__pcf1,__byte1);
	    	break;
    	case 1 :
	    	if (state == 1)			__byte1 |= (1<<7);
	    	else if (state == 0)	__byte1 &= ~(1<<7);
	    	Tobi::__write8(__pcf1,__byte1);
	    	break;
    	case 2 :
	    	if (state == 1)			__byte2 |= (1<<0);
	    	else if (state == 0)	__byte2 &= ~(1<<0);
	    	Tobi::__write8(__pcf2,__byte2);
	    	break;
		// $$ Print an error for any other input
   	 	default:
   	 		Serial.println ("Wrong command") ;
   	}
}

/*  $         SETPWM
	Set PWM of motor from 0 - 255.
	INPUTS: 	- (int) motor, (int) pwm.
	OUTPUTS: 	- None.
	UPDATED:	- None.
*/
void Tobi::setPwm(int motor, int pwm){
	// Update the stored value and write it out to the pin
	Tobi::__pwmVal[motor] = pwm;
	analogWrite(Tobi::__pwmPins[motor], pwm);	
}

/*  $         GETPWM
	Returns PWM of desired motor from 0 - 255.
	INPUTS: 	- (int) motor
	OUTPUTS: 	- None.
	UPDATED:	- __pwmVal[motor]
*/
int Tobi::getPwm(int motor){
	return Tobi::__pwmVal[motor];
}


///////////////////////// READ ENCODERS /////////////////////////

/*  $         CALIBRATEENCODERS
	Finds max values of each encoder and saves them to an array passed
	in by reference. maxEncoderVals MUST be an array of size NUM_MOTORS.
	If encoders are not numbered 0-NUM_MOTORS, must pass in a third
	argument that is a pointer to an array that indexes encoder numbers.
	INPUTS: 	- int* maxEncoderVals, int NUM_MOTORS (optional int* encoderIndices)
	OUTPUTS: 	- None.
	UPDATED:	- maxEncoderVals
*/
void Tobi::calibrateEncoders(){

	// $$ Print an initial message
	if(Serial) Serial.println("Calibrating encoders...");

  	for(int i = 0; i < NUM_MOTORS; i++){
	    // Set each motor to max speed
	    Tobi::setPwm(i,255);
	    // Record 100 encoder values
	    int maxVal = 0;

	    for (int j = 0; j < 100; j++){
			// $$ Read encoder value
			int v = Tobi::readEncoder(i);
			// $$ Save it if it is the max
			if (v > maxVal) {
				maxVal = v;
			}
			delay(20);
	    }

	    // Save in maxEncoderVals[i]
	    maxEncoderVals[i] = maxVal;
	    // $$ Turn each motor off
    	Tobi::setPwm(i,0);
 	}

	// $$ Print a success message when calibrated
 	if(Serial) Serial.println("Calibration complete.\n");
}

void Tobi::calibrateEncoders(int* encoderIndices){

	// $$ Print an initial message
	if(Serial) Serial.println("Calibrating encoders...");

	for(int i = 0; i < NUM_MOTORS; i++){
	    // Set motor to max speed
	    Tobi::setPwm(encoderIndices[i],255);
	    // Record 100 encoder values
	    int maxVal = 0;
	    for (int j = 0; j < 100; j++){
	      // Read encoder value and save if max
	      int v = Tobi::readEncoder(encoderIndices[i]);
		  if (v > maxVal) {
			  maxVal = v;
		  }
		  delay(20);
	    }
	    // Save in maxEncoderVals[i]
	    maxEncoderVals[i] = maxVal;
	    // Turn motor off
	    Tobi::setPwm(encoderIndices[i],0);
	  }
	// $$ Print a success message when calibrated
	if(Serial) Serial.println("Calibration complete.\n");
}

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
int Tobi::readEncoder(int motor){
	// $$ Update encoder value
	Tobi::_analogUpdate();	
	return (Tobi::__encoderVal[motor]);
}

/////////////////////////// LEDS ////////////////////////////

/*  $         LED
	Turn LED (0,1,2,3,4,5) on (1) or off (0). Note: for state, any nonzero number will be treated as on.
	INPUTS: 	- (int) led, (int) state.
	OUTPUTS: 	- None
	UPDATED:	- __byte2. 
*/
void Tobi::led(int led, int state){
	// $$ To turn an LED to state 1 update the corresponding bit in byte2
	if (state == 1) {
	__byte2 |= (1 << led + 1);
	}
	// $$ To turn an LED to state 0 update the corresponding bit in byte2
	else {
		__byte2 &= ~(1 << led + 1);
	}
	// $$ Write out the byte to PCF2
	Tobi::__write8(__pcf2,__byte2);
}

/*  $         NOSELED
	Turn nose LED off (state = 0) or on (state = 1).
	INPUTS: 	- (int) state.
	OUTPUTS: 	- None.
	UPDATED:	- __byte2.
*/
void Tobi::noseLed(int state){
	// $$ To turn the nose LED to state 1 update the corresponding bit in byte2
	if (state == 1)		__byte2 |= (1<<7); 
	else 				__byte2 &= ~(1<<7) ;
}

/////////////////////////// HELPER METHODS ////////////////////////////

/*  $         _ANALOGUPDATE
	Read encoder values into Tobi::__encoderVal array for easy access. Should be called before 
		acting upon Tobi::__encoderVal values.
	INPUTS: 	- None
	OUTPUTS: 	- None
	UPDATED:	- Tobi::__encoderVal
*/
void Tobi::_analogUpdate(){
	// $$ Read in each encoder value every loop
	Tobi::__encoderVal[0] = analogRead(0);
	Tobi::__encoderVal[1] = analogRead(1);
	Tobi::__encoderVal[2] = analogRead(2);
	Tobi::__encoderVal[3] = analogRead(3);
	Tobi::__encoderVal[4] = analogRead(4);
	Tobi::__encoderVal[5] = analogRead(5);
}

/*  $         WRITE8
	Write one byte to chosen address. Used to write to PCF expander.
	$$ NOTE: If there is not sufficient power to the hardware at the given address,
		the program will hang on Wire.endTransmission().
	INPUTS: 	- (int) address, (byte) value.
	OUTPUTS: 	- None.
	UPDATED:	- None.
*/
void Tobi::__write8(int address, byte value){
	// $$ Open a trasmission to the address of the desired hardware
	Wire.beginTransmission(address);
	// $$ Write out the desired new value
	Wire.write(value);
	// $$ End the transmission, data only updated after completion of this line
	Wire.endTransmission();
}

/***************************************************************************************/
/***************************************************************************************/
/***************************** $ UNFINISHED METHODS ************************************/
/***************************************************************************************/
/***************************************************************************************/

// $ Methods left unfinished by Cherag

void Tobi::print_raw(){
	//TODO
	Serial.print("BIT 1:\t");
	Serial.println(__byte1,BIN);
	Serial.print("BIT 2:\t");
	Serial.println(__byte2,BIN);
	Serial.println("---------------------------");

	}

void Tobi::print(){
	//prints out all angle in format 
	// angle leg <leg #> <angle value>
	for (int i = 0 ; i < NUM_MOTORS; i ++){
		Serial.print("angle leg "); Serial.print(i); Serial.print("  "); Serial.println(Tobi::__encoderVal[i]);
	}

	//TODO
}
