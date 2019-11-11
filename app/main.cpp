#include <stdio.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <util/delay.h>
#include <stdlib.h>

#include <uart.h>
#include <misc.h>
#include <FABRIK2D.h>

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

		void wait() {
			while(busy()) {
				_delay_ms(100);
			}
		}

		void moveAngle(uint16_t dAngle, bool direction, uint8_t speed = MaxSpeed) {
			msg("moveAngle ");
			printNumb(dAngle);
			printNumb(direction);
			msg("\n\r");
			if(!busy() && dAngle) {
				speedMax = speed;
				moveSteps(stepsPerDAngle(dAngle), direction);
			} else {
				msg("moveAngle: fuckoff, busy\n\r");
			}
		}

		void stop() {
			stepCount = 0;
			endPoint = 0;
			stopTimer();
		}

		void onStepHandler() {
			uint8_t speed;
			if(!zero && dir.get()) {
				msg("SH: fuck off, !zero\n\r");
				stopTimer();
				stepCount = 0;
				endPoint = 0;
				return;
			}
			if(--stepCount) {
				uint32_t currentStep = endPoint - stepCount;
				if(currentStep < endPoint/2) {
					speed = speedMin + min(speedMax, currentStep/acc_div);
				} else {
					speed = speedMin + min(speedMax, stepCount/acc_div);
				}
				*value = 250/speed;
			} else {
				stopTimer();
			}
		}
		
	private:
		// we have gear ratio at 10 and 8*400 steps per rotation;
		// 32000 steps = 360 grad
		//   ?  steps = x grad
		//   s = x*800/9
		//   argument is grad*10
		uint32_t stepsPerDAngle(uint16_t dAngle) {
			uint32_t tmp = dAngle*9;
			return tmp;
		}
		
		void moveSteps(uint32_t steps, bool direction) {
			if(!zero && direction) {
				msg("moveSteps: fuck off !zero and direction");
				//return;
			}
			endPoint = steps;
			stepCount = steps;
			if(direction) {
				dir.set();
			} else {
				dir.clear();
			}
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

		uint16_t min(uint16_t a, uint16_t b) {
			return a > b ? b : a;
		}

	private:
		OutPin & dir;
		InPin & zero;
		uint32_t endPoint = 0;
		uint32_t stepCount = 0;
		uint8_t clockSource = 2;
		volatile uint8_t * value;
		volatile uint8_t * tcr;
		static constexpr uint8_t speedMin = 2; // x100 pulse per second
		uint8_t speedMax = 12;
		static constexpr uint8_t MaxSpeed = 40;
		static constexpr uint8_t acc_div = 30;
};

class MecanicalArm {
	public:
		MecanicalArm(StepMotor & motor0, StepMotor & motor1, OutPin & motorEn)
			: m0(motor0)
			, m1(motor1)
			, motEn(motorEn)
	{
		motEn.clear();
	}

	void move(int16_t ang0, int16_t ang1) {
		motEn.clear();
		msg("waiting for motors !busy\n\r");
		m0.wait();
		m1.wait();
		msg("arm: enable motors, do autohome\n\r");
		autoHomeImp();
		msg("arm: move to target");
		printNumb(ang0);
		printNumb(ang1);
		msg("\n\r");
		m0.moveAngle(abs(ang0), 0);
		m0.wait();
		m1.moveAngle(abs(ang1), (ang1 > 0) ? false : true);
		m1.wait();
		msg("arm: disable motors\n\r");
		motEn.set();
	}

	void init() {
		motEn.clear();
		autoHomeImp();
		motEn.set();
		msg("\n\r");

	}
	
	private:
	void autoHomeImp() {
		msg("arm: move both until 0 finish\n\r");
		m0.moveAngle(1000, 1, 10);
		m1.moveAngle(1000, 0, 8);
		m0.wait();
		m1.stop();
		msg("arm: ok, stop 1 and move it to zero\n\r");
		m1.moveAngle(1000, 1, 10);
		m1.wait();
		msg("arm: ok, now move both to init pos\n\r");
		m0.moveAngle(m0HomeAngle, 0);
		m1.moveAngle(m1HomeAngle, 0);
		m0.wait();
		m1.wait();
		msg("arm: homing done\n\r");
	}
	StepMotor & m0;
	StepMotor & m1;
	OutPin & motEn;
	static constexpr uint8_t offsetX = 75;
	static constexpr uint8_t offsetY = 105;
	static constexpr uint8_t m0HomeAngle = 10;
	static constexpr uint8_t m1HomeAngle = 10;
};

// Objects
OutPin led(&DDRB, &PORTB, PIN5);     // on board led
OutPin m0dir(&DDRD, &PORTD, PIN7);   // PD7 is direction for motor 2
OutPin m1dir(&DDRB, &PORTB, PIN0);   // PB0 is direction for motor 1
OutPin motEn(&DDRB, &PORTB, PIN2);   // enable morots drivers
OutPin magnito(&DDRC, &PORTC, PIN0); // electro magnet (low: on, hight: off)
InPin zero0pin(&DDRD, &PORTD, &PIND, PIN2); // zero switch pin (low: pressed)
InPin zero1pin(&DDRD, &PORTD, &PIND, PIN4); // zero switch pin (low: pressed)
OutPin servo(&DDRD, &PORTC, PIN3)    // arm servoPin (OC2B)

volatile uint8_t t0value;
StepMotor m0(m0dir, zero0pin, 0u, &t0value, &TCCR0B);
StepMotor m1(m1dir, zero1pin, 2u, &ICR1L, &TCCR1B);

int lengths[] = {245, 176};
Fabrik2D fab(3, lengths);
MecanicalArm arm(m0, m1, motEn);

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

	//arm.init();
	
	int x = 220;
	int y = 40;
	tst.set();
	fab.setTolerance(0.5);
	fab.solve(x, y, lengths);
	msg("for ");
	printNumb(x);
	printNumb(y);
	msg("got ");
	printNumb(180*fab.getAngle(0)/3.14);
	printNumb(180*fab.getAngle(1)/3.14);
	msg("\n\r");
	int32_t ang0 = 900-(1800*fab.getAngle(0))/3.14;
	int32_t ang1 = 1800+(1800*fab.getAngle(1))/3.14;
	tst.clear();
	msg("angles: ");
	printNumb(ang0);
	printNumb(ang1);
	msg("calc motor ang: ");
	int16_t a1 = ang0-50;
	int16_t a2 = ang1-ang0-70;
	printNumb(a1);
	printNumb(a2);
	msg("\n\r move to...\n\r");
	//50 (5 grad) is a zero angle offset for m0
	//arm.move(a1, a2);
	while(1)
	{
		led.toggle();
		magnito.set();
		_delay_ms(300);
		magnito.clear();
		_delay_ms(1000);
	}
}

