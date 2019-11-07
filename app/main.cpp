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
		StepMotor(OutPin & dirPin, InPin & zeroPin, uint8_t clockSrc, volatile uint8_t * reg, volatile uint8_t * control) 
		: dir(dirPin)
		, zero(zeroPin)
		, clockSource(clockSrc)
		, value(reg)
		, tcr(control)
		{}

		bool busy() {
			return stepCount > 0;
		}

		void moveAngle(uint8_t angle, bool direction) {
			if(!busy()) {
				msg("start\n\r");
				speedMax = 12;
				moveSteps(stepsPerAngle(angle), direction);
			}
		}

		void autoHome() {
			// move slowly until onZeroPoint
			msg("autohome\n\r");
			speedMax = 1;
			uint16_t steps = 2000;
			moveSteps(steps, 1);
		}

		void onStepHandler() {
			uint8_t speed;
			if(!zero && dir.get()) {
				stopTimer();
				stepCount = 0;
				endPoint = 0;
				msg("zero!\n\r");
				return;
			}
			if(--stepCount) {
				uint16_t currentStep = endPoint - stepCount;
				if(currentStep < endPoint/2) {
					speed = speedMin + min(speedMax, currentStep/acc_div);
				} else {
					speed = speedMin + min(speedMax, stepCount/acc_div);
				}
				*value = 250/speed;
			} else {
				stopTimer();
				msg("end\n\r");
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
			if(!zero && direction) {
				msg("can't start: zero!");
				return;
			}
			endPoint = steps;
			if(direction) {
				dir.set();
			} else {
				dir.clear();
			}
			stepCount = steps;
			*value = 250/speedMin;
			startTimer();
		}

		void stopTimer() {
			*tcr &= ~_BV(clockSource);
			*value = 250/speedMin;
		}
		void startTimer() {
			*tcr |= _BV(clockSource); // SCx2
		}

		uint8_t min(uint8_t a, uint8_t b) {
			return a > b ? b : a;
		}

	private:
		OutPin & dir;
		InPin & zero;
		uint16_t endPoint = 0;
		uint16_t stepCount = 0;
		uint8_t clockSource = 2;
		volatile uint8_t * value;
		volatile uint8_t * tcr;
		static constexpr uint8_t speedMin = 2; // x100 pulse per second
		uint8_t speedMax = 12;
		static constexpr uint8_t acc_div = 40;
};

class MecanicalArm {
	public:
		MecanicalArm(StepMotor & motor0, StepMotor & motor1)
			: m0(motor0)
			, m1(motor1)
	{}
	
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

volatile uint8_t t0value;
StepMotor m0(m0dir, zero0pin, 0u, &t0value, &TCCR0B);
StepMotor m1(m1dir, zero1pin, 2u, &ICR1L, &TCCR1B);
MecanicalArm arm(m0, m1);

/*
ISR(INT0_vect)
{
	m0.onZeroPoint();
	msg("zero 0\n\r");
}

ISR(INT1_vect)
{
	m1.onZeroPoint();
	msg("zero 1\n\r");
}
*/

OutPin tst(&DDRB, &PORTB, PIN4);
OutPin t0pin(&DDRD, &PORTD, PIN6);


ISR(TIMER0_OVF_vect)
{
	static uint8_t value = 0;
	if(!value) {
		value = t0value;
	}
	if(value == 1) {
		t0pin.set();
	} else {
		t0pin.clear();
	}
	value--;
	if(!value) {
		m0.onStepHandler();
		value = t0value;
	}
}

ISR(TIMER1_OVF_vect)
{
	m1.onStepHandler();
}


int main(void)
{
	// setup external interrupt for endstops
	//EICRA = (1 << ISC01) | (1 << ISC11);
	//EIMSK = (1 << INT0) | (1 << INT1);
	
	// timer0 has no sutable mode, so do it with simple interrupt
	TIMSK0 = _BV(TOIE0);
	
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

		m1.moveAngle(30, 0);
		m0.moveAngle(40, 0);
		while(m0.busy()) {
			_delay_ms(100);
			msg("busy...\n\r");
		}
		while(m1.busy()) {
			_delay_ms(100);
			msg("busy...\n\r");
		}
		m0.autoHome();
		m1.autoHome();
		while(m0.busy()) {
			_delay_ms(100);
			msg("busy...\n\r");
		}
		while(m1.busy()) {
			_delay_ms(100);
			msg("busy...\n\r");
		}
		//while(m0.busy());
		//while(m1.busy());
		//m1.moveAngle(30, 1);
		_delay_ms(1000);
		//m1.moveAngle(30, 1);
		//m0.moveAngle(20, 0);
	}
}

