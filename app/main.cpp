#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <util/delay.h>

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


class Servo {
	public:
		Servo(OutPin & magnitoPin)
			: magnito(magnitoPin)
		{}

		void init() {
			phase = Up;
			TCCR2B = mask;
		}

		bool busy() {
			return TCCR2B != 0;
		}

		void grabSync() {
			grab();
			while(busy()) {
				_delay_ms(100);
			}
		}

		void putSync() {
			put();
			while(busy()) {
				_delay_ms(100);
			}
		}

		void grab() {
			phase = Down;
			state = Grub;
			TCCR2B = mask;
		}

		void put() {
			phase = Down;
			state = Put;
			TCCR2B = mask;
		}

		void onTick() {
			static uint8_t delayCnt = 0;
			delayCnt++;
			if(delayCnt < 4) {
				return;
			}
			delayCnt = 0;

			if(phase == Up) {
				if(OCR2B < max) {
					OCR2B++;
				} else {
					// stop timer
					TCCR2B = 0;
				}
			} else {
				if(OCR2B > min) {
					OCR2B--;
				} else {
					if(state == Grub) {
						magnito.set();
					} else {
						magnito.clear();
					}
					phase = Up;
				}
			}
		}

	private:
		enum MovePhase {
			Up,
			Down
		};
		enum State {
			Grub,
			Put
		};
		MovePhase phase;
		State state;
		volatile uint8_t * ctl;
		OutPin & magnito;
		static constexpr uint8_t min = 15;
		static constexpr uint8_t max = 34;
		static constexpr uint8_t mask = _BV(CS22) | _BV(CS21) | _BV(CS20);
};

class Point {
public:
	Point(int16_t xa, int16_t ya)
	: x(xa)
	, y(ya)
	{}
	int16_t x = 0;
	int16_t y = 0;
};

/*
Coordinate system looks like that:
		 0,0
		  o----------------> y
		  |
xOff,yOff | board
   o------|--------
   |      |        |
   |      |        |
		  |
		  V x
*/


class Board {
public:
	// offset from
	Board(uint8_t x, uint8_t y)
		: offset(x,y)
	{}

	Point indToPoint(uint8_t ind)
	{
		uint8_t row = ind / 8;
		uint8_t col = ind % 8;

		return Point( (7-row)*cellSize + cellSize/2 + margin + offset.x,
					(col*cellSize) + cellSize/2 + margin + offset.y );
	}

private:
	static constexpr uint16_t size = 325;
	static constexpr uint8_t margin = 25;
	static constexpr uint8_t cellSize = 30;
	const Point offset;
};

class MecanicalArm {
public:
	MecanicalArm(StepMotor & motor0, StepMotor & motor1, OutPin & motorEn, Servo & servo)
		: m0(motor0)
		, m1(motor1)
		, motEn(motorEn)
		, srv(servo)
		, fab(Fabrik2D(3, lengths))
		, b(boardOffsetX, boardOffsetY) // board offset
	{
		fab.setTolerance(0.5);
		motEn.clear();
	}

	void init() {
		//TODO: solve Home point to have init angles
		srv.init();
		motEn.clear();
		autoHomeImp();
		motEn.set();
	}

	// take from board peice from ind field
	void take(uint8_t ind) {
		motEn.clear();
		moveToInd(ind);
		srv.grabSync();
		// TODO: have to move outside board and drop peice
		//moveToInd(-1);
		//srv.put();
		motEn.set();
	}

	// move peice from feild to new field
	void move(uint8_t fromInd, uint8_t toInd)
	{
		motEn.clear();
		moveToInd(fromInd);
		srv.grabSync();
		moveToInd(toInd);
		srv.putSync();
		autoHomeImp();
		motEn.set();
	}

private:

	void moveToInd(uint8_t ind)
	{
		Point target = b.indToPoint(ind);
		fab.solve(target.x, target.y, lengths);
		moveToAng(fab.getAngle(0), fab.getAngle(1));
	}

	void moveToAng(float angle0, float angle1) {
		int32_t ang0 = 900-(1800*angle0)/3.14;
		int32_t ang1 = 1800+(1800*angle1)/3.14;
		msg("angles: ");
		printNumb(ang0);
		printNumb(ang1);
		msg("calc motor ang: ");
		int16_t a1 = ang0-m0HomeAngle;
		int16_t a2 = ang1-ang0-m1HomeAngle;
		printNumb(a1);
		printNumb(a2);
		msg("\n\r move to...\n\r");

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
private:
	StepMotor & m0;
	StepMotor & m1;
	OutPin & motEn;
	Servo & srv;
	Fabrik2D fab;
	Board b;
	const uint8_t lengths[2] = {245, 176};
	static constexpr int8_t boardOffsetX = 65;
	static constexpr int8_t boardOffsetY = -105;
	static constexpr uint8_t m0HomeAngle = 50;
	static constexpr uint8_t m1HomeAngle = 70;
};

// Objects
OutPin led(&DDRB, &PORTB, PIN5);     // on board led
OutPin m0dir(&DDRD, &PORTD, PIN7);   // PD7 is direction for motor 2
OutPin m1dir(&DDRB, &PORTB, PIN0);   // PB0 is direction for motor 1
OutPin motEn(&DDRB, &PORTB, PIN2);   // enable morots drivers
InPin zero0pin(&DDRD, &PORTD, &PIND, PIN2); // zero switch pin (low: pressed)
InPin zero1pin(&DDRD, &PORTD, &PIND, PIN4); // zero switch pin (low: pressed)
OutPin magnito(&DDRC, &PORTC, PIN0); // electro magnet (low: on, hight: off)
OutPin servoPin(&DDRD, &PORTD, PIN3);   // arm servoPin (OC2B)

volatile uint8_t t0value;
StepMotor m0(m0dir, zero0pin, 0u, &t0value, &TCCR0B);
StepMotor m1(m1dir, zero1pin, 2u, &ICR1L, &TCCR1B);

Servo servo(magnito);
MecanicalArm arm(m0, m1, motEn, servo);

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

ISR(TIMER2_OVF_vect)
{
	servo.onTick();
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

	// timer0 has no sutable mode, so use simple interrupt for pwm
	TIMSK0 = _BV(TOIE0);

	DDRB |= _BV(PB1);
	OCR1A = 1;
	ICR1 = 30;
	TCCR1A = _BV(COM1A1) | _BV(WGM11);
	TCCR1B = _BV(WGM13) | _BV(WGM12);
	TIMSK1 = _BV(TOIE1);

	// timer for servo
	TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
	TIMSK2 = _BV(TOIE2);

	motEn.set();

	uart_init(0);
	msg("\n\r\n\rCheckersBot v1.0\n\r\n\r");

	sei();

	arm.init();
	arm.move(5,10);
	arm.take(12);

	while(1)
	{
		led.toggle();
		_delay_ms(500);
	}
}

