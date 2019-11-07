#include <stdio.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <util/delay.h>
#include <stdlib.h>
#include <math.h>

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
			uint8_t speed;
			if(stepCount) {
				uint16_t currentStep = endPoint - stepCount;
				if(currentStep < endPoint/2) {
					speed = 1 + fmin(speedMax, currentStep/acc_div);
				} else {
					speed = 1 + fmin(speedMax, stepCount/acc_div);
				}
				stepCount--;
				*value = 250/speed;
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
			*value = 250/speedMin;
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
		static constexpr uint8_t speedMin = 1; // x100 pulse per second
		static constexpr uint8_t speedMax = 8;
		static constexpr uint8_t acc_div = 16;
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
StepMotor m1(m1dir, zero1pin, &ICR1L, &TCCR1B);
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

ISR(TIMER1_OVF_vect)
{
	tst.set();
	m1.onStepHandler();
	tst.clear();
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
	OCR1A = 1;
	ICR1 = 30;
	TCCR1A = _BV(COM1A1) | _BV(WGM11);
	TCCR1B = _BV(WGM13) | _BV(WGM12);
	TIMSK1 = _BV(TOIE1);
	

	uart_init(0);
	msg("\n\r\n\rCheckersBot v1.0\n\r\n\r");

	sei();

	while(1)
	{
		_delay_ms(500);
		led.toggle();
		
		while(m1.busy());
		m1.moveAngle(50, 0);
		_delay_ms(500);
		while(m1.busy());
		m1.moveAngle(25, 1);
		_delay_ms(500);
		while(m1.busy());
		m1.moveAngle(15, 1);
		_delay_ms(500);
		while(m1.busy());
		m1.moveAngle(5, 1);
		_delay_ms(500);
		while(m1.busy());
		m1.moveAngle(5, 1);
		
	}
}

