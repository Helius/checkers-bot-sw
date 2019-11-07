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
		StepMotor(OutPin & dirPin, InPin & zeroPin, volatile uint8_t * reg, volatile uint8_t * control) 
		: dir(dirPin)
		, zero(zeroPin)
		, value(reg)
		, tcr(control)
		{
		}

		bool busy() {
			printNumb(stepCount);
			msg("\n\r");
			return stepCount;
		}

		void moveAngle(uint8_t angle, bool direction) {
			if(!busy()) {
				moveSteps(stepsPerAngle(angle), direction);
			}
		}

		void onZeroPoint() {
			// stop timer
			stopTimer();
			stepCount = 0;
			endPoint = 0;
		}

		void onStepHandler() {
			static constexpr uint8_t speedAdd = speedMax - speedMin;
			uint8_t speed;
			if(stepCount) {
				uint16_t currentStep = endPoint-stepCount;
				if(currentStep < endPoint/4) {
					speed = speedMin + (speedAdd * currentStep * 4)/endPoint;
				} else if (currentStep > (3*endPoint)/4) {
					speed = speedMin + speedAdd - 
						speedAdd * (4 * (currentStep + 1) - 3 * endPoint) / endPoint;
				} else {
					speed = speedMax;
				}
				/*
				printNumb(currentStep);
				printNumb(endPoint);
				printNumb(speed);
				msg("\n\r");
				*/
				stepCount--;
				*value = 300/speed;
			} else {
				stopTimer();
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
			startTimer();
			endPoint = steps;
			if(direction) {
				dir.set();
			} else {
				dir.clear();
			}
			stepCount = steps;
			// start timer
		}

		void stopTimer() {
			*tcr &= ~_BV(2);
			*value = 300/speedMin;
		}
		void startTimer() {
			*tcr |= _BV(2); // SCx2
		}

	private:
		OutPin & dir;
		InPin & zero;
		uint16_t stepCount = 0;
		uint16_t endPoint = 0;
		volatile uint8_t * value;
		volatile uint8_t * tcr;
		// min  = 6 is good but too fast
		// min  = 4 works but has step miss on load
		static constexpr uint8_t speedMin = 6; // x100 pulse per second
		static constexpr uint8_t speedMax = 15;
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
OutPin m0dir(&DDRD, &PORTD, PIN7); // PD7 is direction for motor 2
OutPin m1dir(&DDRB, &PORTB, PIN0); // PB0 is direction for motor 1
InPin zero0pin(&DDRD, &PORTD, &PIND, PIN2); // ext0 pin
InPin zero1pin(&DDRD, &PORTD, &PIND, PIN3); // ext1 pin

StepMotor m0(m0dir, zero0pin, &OCR0A, &TCCR0B);
StepMotor m1(m1dir, zero1pin, &OCR1AL, &TCCR1B);
MecanicalArm arm(m0, m1);

// ISR Handlers
ISR(TIMER2_OVF_vect)
{
}

ISR(INT0_vect)
{
	//m0.onZeroPoint();
	int val = !!zero0pin;
	printNumb(val);
	msg("zero 0\n\r");
}

ISR(INT1_vect)
{
	//m1.onZeroPoint();
	int val = !!zero1pin;
	printNumb(val);
	msg("zero 1\n\r");
}

OutPin tst(&DDRB, &PORTB, PIN4);

ISR(TIMER0_COMPA_vect)
{
	static bool toggle = 0;
	if(toggle) {
		tst.set();
		m0.onStepHandler();
		tst.clear();
	}
	toggle = !toggle;
}

ISR(TIMER1_COMPA_vect)
{
	static bool toggle = 0;
	if(toggle) {
		tst.set();
		m1.onStepHandler();
		tst.clear();
	}
	toggle = !toggle;
}


int main(void)
{
	// setup external interrupt
	EICRA = (1 << ISC01) | (1 << ISC11);
	EIMSK = (1 << INT0) | (1 << INT1);
	
	// timer 0 and 1 init for motors pulse handling
	// 60..20
	//OCR0A = 20;
	DDRD |= _BV(PD6);
	TCCR0A = _BV(WGM01) | _BV(COM0A0);
	TIMSK0 = _BV(OCIE0A);
	//TCCR0B = _BV(2);
	
	DDRB |= _BV(PB1);
	TCCR1A = _BV(COM1A0);
	TCCR1B = _BV(WGM12);
	TIMSK1 = _BV(OCIE1A);
	

	uart_init(0);
	msg("\n\r\n\rCheckersBot v1.0\n\r\n\r");

	sei();

	while(1)
	{
		_delay_ms(500);
		led.toggle();
		
		while(m0.busy());
		m0.moveAngle(50, 0);
		_delay_ms(500);
		while(m0.busy());
		m0.moveAngle(25, 1);
		_delay_ms(500);
		while(m0.busy());
		m0.moveAngle(15, 1);
		_delay_ms(500);
		while(m0.busy());
		m0.moveAngle(5, 1);
		_delay_ms(500);
		while(m0.busy());
		m0.moveAngle(5, 1);
	}
}

