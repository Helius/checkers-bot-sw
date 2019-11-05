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
		StepMotor(OutPin & dirPin, InPin & zeroPin, volatile uint8_t * reg) 
		: dir(dirPin)
		, zero(zeroPin)
		, value(reg)
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
				value++;
			} else {
				// done, stop timer
				value = 0;
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
		volatile uint8_t * value;
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

StepMotor m0(m0dir, zero0pin, &OCR0A);
StepMotor m1(m1dir, zero1pin, &OCR1AL);
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

OutPin tst(&DDRB, &PORTB, PIN4);

ISR(TIMER1_COMPA_vect)
{
	static bool toggle = 0;
	if(toggle) {
		m0.onStepHandler();
	}
	toggle = !toggle;
}

ISR(TIMER0_COMPA_vect)
{
	static bool toggle = 0;
	if(toggle) {
		m1.onStepHandler();
	}
	toggle = !toggle;
}

int main(void)
{
	// setup external interrupt
	EICRA = (1 << ISC01) | (1 << ISC11);
	EIMSK = (1 << INT0) | (1 << INT1);
	
	// timer 0 and 1 init for motors pulse handling
	DDRD |= _BV(PD6);
	OCR0A  = 0;
	TCCR0A = _BV(WGM01) | _BV(COM0A0);
	TCCR0B = _BV(CS02);
	TIMSK0 = _BV(OCIE0A);
	
	DDRB |= _BV(PB1);
	OCR1A  = 0;
	TCCR1A = _BV(COM1A0);
	TCCR1B = _BV(WGM12) | _BV(CS12);
	TIMSK1 = _BV(OCIE1A);
	

	uart_init(0);
	msg("\n\r\n\rCheckersBot v1.0\n\r\n\r");

	sei();
	while(1)
	{
		_delay_ms(200);
		led.toggle();
	}
}

