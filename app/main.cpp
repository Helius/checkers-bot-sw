#include <stdio.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <util/delay.h>
#include <stdlib.h>

#include <uart.h>
#include <misc.h>

// Types

class StepMotor {
	public:
		StepMotor(OutPin & dirPin, InPin & zeroPin) 
		: dir(dirPin)
		, zero(zeroPin)
		{
		}

		bool busy() {
			return stepCount;
		}

		void moveAngle(uint8_t angle, bool direction) {
			if(!busy()) {
				lastAngle = angle;
				moveSteps(stepsPerAngle(angle), direction);
			}
		}

		void moveToHome() {
			// slow go home until onZeroPoint
		}

		void onZeroPoint() {
			// stop timer
			stepCount = 0;
		}

		void onStepHandler() {
			if(stepCount--) {
				// handle timer speed
			} else {
				// done, stop timer
			}
		}
		
	private:
		// we have gear ratio at 10 and 400 steps per rotation;
		// 4000 steps = 360 grad
		//   ?  steps = x grad
		uint16_t stepsPerAngle(uint8_t angle) {
			return (100*angle)/9;
		}
		
		void moveSteps(uint16_t steps, bool direction) {
			if(direction) {
				dir.set();
			} else {
				dir.clear();
			}
			stepCount = steps;
			// start timer
		}

	private:
		OutPin & dir;
		InPin & zero;
		uint8_t lastAngle = 0;
		uint16_t stepCount = 0;
};

class MecanicalArm {
	public:
		MecanicalArm(StepMotor & motor0, StepMotor & motor1)
			: m0(motor0)
			, m1(motor1)
	{}
	
		void onZeroPoint(int ind) {
			switch(ind) {
				case 0:
					m0.onZeroPoint();
					break;
				case 1:
					m0.onZeroPoint();
					break;
			}
		}

	private:
		StepMotor & m0;
		StepMotor & m1;
};

// Objects
OutPin led(&DDRB, &PORTB, PIN5);   // on board led
OutPin m0dir(&DDRB, &PORTB, PIN0); // PB0 is direction for motor 1
OutPin m1dir(&DDRD, &PORTD, PIN7); // PD7 is direction for motor 2
InPin zero0pin(&DDRD, &PORTD, &PIND, PIN2); // ext0 pin
InPin zero1pin(&DDRD, &PORTD, &PIND, PIN3); // ext1 pin

StepMotor m0(m0dir, zero0pin);
StepMotor m1(m1dir, zero1pin);
MecanicalArm arm(m0, m1);

// ISR Handlers
ISR(TIMER2_OVF_vect)
{
}

ISR(INT0_vect)
{
	m0.onZeroPoint();
	int val = !!zero0pin;
	printNumb(val);
	msg("zero 0\n\r");
}

ISR(INT1_vect)
{
	m1.onZeroPoint();
	int val = !!zero1pin;
	printNumb(val);
	msg("zero 1\n\r");
}

int main(void)
{
	// setup external interrupt
	EICRA = (1 << ISC01) | (1 << ISC11);
	EIMSK = (1 << INT0) | (1 << INT1);
	
	// timer 1 init for motors pulse handling
	
	TCCR0A = _BV(WGM01) | _BV(WGM00) | _BV(COM0A1) | _BV(COM0B1); // fast pwm, non-invert
	TCCR0B = _BV(CS01) | _BV(CS00);  // prescaller is 32
	OCR0A  = 0;
	OCR0B  = 0;
	

	uart_init(0);
	msg("\n\r\n\rCheckersBot v1.0\n\r\n\r");

	sei();
	while(1)
	{
		_delay_ms(200);
		led.toggle();
	}
}

