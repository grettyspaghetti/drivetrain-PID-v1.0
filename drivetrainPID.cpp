//library inclusions
#include <Arduino.h>
#include <Encoder.h>

//declare drivetrain pins
byte leftBackward = 2;
byte leftForward = 3;
byte leftPWM = 4;
byte rightForward = 5;
byte rightBackward = 6;
byte rightPWM = 7;

//instantiate Encoder class for each drivetrain motor
Encoder left(18, 19);
Encoder right(20, 21);

//forward declarations of functions
void drivePID(float distance, String direction);
float getPower();

//declare byte variables
byte currentPowerL, currentPowerR;
byte distToTicks = 172;
byte maxPower = 255;
byte goalPower = 200;
byte iterationTime = 10;

//still need to tune
float kp = 0.3;
float ki = 0.0;
float kd = 0.0;

//declare float variables
float setpoint, currentTicksL, currentTicksR, errorPowerL, errorPowerR, newPowerL, newPowerR, lastErrorL, lastErrorR;

float ticksIL, ticksIR, ticksFL, ticksFR, deltaL, deltaR, leftTPS, rightTPS;

float proportionalL, proportionalR, integralL, integralR, derivativeL, derivativeR;

float lastIntegralL= 0;
float lastIntegralR = 0;

//declare array and pointer for returned power variables
float powers[2];
float *power = powers;

//declare int variables
int maxTPS = 4928;
int calculationTime = 50;

void setup() {


	//set all drivetrain motor pins to OUTPUT mode
	pinMode(leftBackward, OUTPUT);
	pinMode(leftForward, OUTPUT);
	pinMode(leftPWM, OUTPUT);
	pinMode(rightForward, OUTPUT);
	pinMode(rightBackward, OUTPUT);
	pinMode(rightPWM, OUTPUT);

	//begin serial terminal and print an opening message to it
	Serial.begin(115200);
	delay(10);
	Serial.println("\n****new test time****\n");

	//drive forward for 60 inches as a test
	drivePID(60, "forward");

	//close serial terminal
	Serial.end();

}

void loop() {

}

void drivePID(float distance, String direction){

	//convert distance in inches to ticks (approximately 172 ticks per inch traveled)
	setpoint = (distance)*(distToTicks);

	if(direction == "forward"){

		//set pins to correct voltages for forward motion
		digitalWrite(leftForward, HIGH);
		digitalWrite(leftBackward, LOW);
		digitalWrite(rightForward, HIGH);
		digitalWrite(rightBackward, LOW);

		//reset both encoders
		left.write(0);
		right.write(0);

		//grab current encoder values
		//the left side must be multiplied by -1 to count upwards from 0
		currentTicksL = (-1)*(left.read());
		currentTicksR = (right.read());

	} else if(direction == "backward"){

		//set pins to correct voltages for backward motion
		digitalWrite(leftForward, LOW);
		digitalWrite(leftBackward, HIGH);
		digitalWrite(rightForward, LOW);
		digitalWrite(rightBackward, HIGH);

		//reset both encoders
		left.write(0);
		right.write(0);

		//grab current encoder values
		//the right side must be multiplied by -1 to count upwards from 0
		currentTicksL = (left.read());
		currentTicksR = (-1)*(right.read());

	}

	//give the motors the goal PWM power
	analogWrite(leftPWM, goalPower);
	analogWrite(rightPWM, goalPower);

	//repeat while the encoders have not reached their goal...
	while((currentTicksL <= setpoint) && (currentTicksR <= setpoint)){

		//grab current encoder values, accounting for direction
		if(direction == "forward"){

			currentTicksL = (-1)*(left.read());
			currentTicksR = (right.read());

		} else if(direction == "backward"){

			currentTicksL = (left.read());
			currentTicksR = (-1)*(right.read());
		}

		//grab current motor powers using the getPower() function and use a pointer to access both values
		*power = getPower();
		currentPowerL = *power;
		currentPowerR = *(power+1);

		//new line in serial terminal
		Serial.println();

		//determine the error between goal power and current power
		errorPowerL = goalPower - currentPowerL;
		errorPowerR = goalPower - currentPowerR;

		//set proportional variables to the error
		proportionalL = errorPowerL;
		proportionalR = errorPowerR;

		//set integral variables to the previous integral variables plus the error multiplied by the iteration time
		integralL = lastIntegralL + (errorPowerL * iterationTime);
		integralR = lastIntegralR + (errorPowerR * iterationTime);

		//set derivative variables to the error minus the previous error, divided by the iteration time
		derivativeL = (errorPowerL - lastErrorL)/iterationTime;
		derivativeR = (errorPowerR - lastErrorR)/iterationTime;

		newPowerL = goalPower + (proportionalL * kp) + (integralL * ki) + (derivativeL * kd);
		newPowerR = goalPower + (proportionalR * kp) + (integralR * ki) + (derivativeR * kd);

		//cap out the new powers at 255 (the maximum power that can be given to the motors)
		if((newPowerL) > 255){
			newPowerL = 255;
		}
		if((newPowerR) > 255){
			newPowerR = 255;
		}

		//print data to the serial terminal
		Serial.println("current right power: " +String(currentPowerR) + ", new right power: " + String(newPowerR) + ", current right position: " + String(currentTicksR));
		Serial.println("current left power: " +String(currentPowerL) + ", new left power: " + String(newPowerL) + ", current left position: " + String(currentTicksL));

		newPowerL = byte(newPowerL);
		newPowerR = byte(newPowerR);

		//give both motors their respective new powers
		analogWrite(leftPWM, newPowerL);
		analogWrite(rightPWM, newPowerR);

		//set the last integral and last error variables to the integral and error variables from the current loop
		lastIntegralL = integralL;
		lastIntegralR = integralR;

		lastErrorL = errorPowerL;
		lastErrorR = errorPowerR;

		//delay for the set iteration time
		delay(iterationTime);
	}

	//after the encoders have reached their goal, stop giving them power
	analogWrite(leftPWM, 0);
	analogWrite(rightPWM, 0);

}

float getPower(){

	//use sign of current left ticks to determine if the robot is moving forwards or backwards,
	//and multiply one side by -1 accordingly, so that both sides will be counting in the positive direction
	if((left.read()) < 0){
		ticksIL = (-1)*(left.read());
		ticksIR = (right.read());
	} else {
		ticksIL = (left.read());
		ticksIR = (-1)*(right.read());
	}

	//delay for the set time interval
	delay(calculationTime);

	//grab ticks after elapsed time using same logic as previous if-else statement
	if((left.read()) < 0){
		ticksFL = (-1)*(left.read());
		ticksFR = (right.read());
	} else {
		ticksFL = (left.read());
		ticksFR = (-1)*(right.read());
	}

	//calculate difference in ticks across time interval
	deltaL = ticksFL - ticksIL;
	deltaR = ticksFR - ticksIR;

	/*calculate current ticks per second as a percentage of the maximum ticks per second
	 * and multiply that by the maximum power value to calculate the approximate current PWM power of each motor
	 * (ticks/milliseconds)*(milliseconds/second) */
	leftTPS = (deltaL/calculationTime)*(1000);
	currentPowerL = (leftTPS/maxTPS) * maxPower;
	*power = currentPowerL;

	rightTPS = (deltaR/calculationTime)*(1000);
	currentPowerR = (rightTPS/maxTPS) * maxPower;
	*(power +1) = currentPowerR;

	//return pointer variable which points to the array containing both motor powers
	return *power;

}
