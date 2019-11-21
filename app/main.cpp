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

Message msg;
using m = Message;

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

		void moveAngle(int16_t dAngle, uint8_t speed = MaxSpeed) {
			bool direction = dAngle > 0 ? 0 : 1;
			msg << "moveAngle " << dAngle << direction << m::endl;
			if(!busy() && dAngle) {
				speedMax = speed;
				moveSteps(stepsPerDAngle(dAngle), direction);
			} else {
				msg << "moveAngle: fuckoff, busy" << m::endl;
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
				msg << "!zero" << m::endl;
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
				msg << "moveSteps: fuck off !zero and direction" << m::endl;
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
			OCR2B = max-2;
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
	Board(uint16_t x, uint16_t y)
		: offset(x,y)
	{}

	Point indToPoint(uint8_t ind)
	{
		uint8_t row = ind*2 / 8;
		uint8_t col = ind*2 % 8 + (row%2 ? 1 : 0);

		return Point( (7-row)*cellSize + cellSize/2 + margin + offset.x,
					(col*cellSize) + cellSize/2 + margin + offset.y);
	}

private:
	static constexpr uint16_t size = 290;
	static constexpr uint8_t margin = 25;
	static constexpr uint8_t cellSize = 30;
	const Point offset;
};

class Angles {
public:
	Angles(int16_t angle0, int16_t angle1)
		: ang0(angle0)
		, ang1(angle1)
	{}
	int16_t ang0;
	int16_t ang1;
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
		srv.init();
		fab.solve(245-176, 0, lengths); // Home point
		msg << "arm test: " << (573*fab.getAngle(0)/10) << 573*fab.getAngle(1)/10 << m::endl;
		fab.solve(60, 86, lengths); // Home point
		msg << "arm init angles: " << (573*fab.getAngle(0)/10) << 573*fab.getAngle(1)/10 << m::endl;
		motEn.clear();
		autoHomeImp();
		motEn.set();
	}

	// take from board peice from ind field
	void take(uint8_t ind) {
		motEn.clear();
		moveToInd(ind);
		srv.putSync();
		//autoHomeImp();
		// TODO: have to move outside board and drop peice
		//moveToInd(9);
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
		//autoHomeImp();
		motEn.set();
	}

	Angles calcAngles(uint8_t ind)
	{
		Point target = b.indToPoint(ind);
		msg << "calc for x,y " << ind << target.x << target.y << m::tab;

		//fab.createChain(lengths);
		fab.solve(target.x, target.y, lengths);

		msg << "fabric ang: [" << (573*fab.getAngle(0)/10) << (573*fab.getAngle(1)/10) << "] " << m::tab;

		int16_t ang0 = 900 + (573*fab.getAngle(0));
		int16_t ang1 = ang0 - (1800 + (573*fab.getAngle(1)));

		msg << " angles: " << ang0 << ang1 << m::endl;
		return {ang0, ang1};
	}


private:

	void moveToInd(uint8_t ind)
	{
		Angles a = calcAngles(ind);
		moveToAng(a.ang0, a.ang1);
	}

	void moveToAng(int16_t angle0, int16_t angle1) {
		msg << "waiting for motors !busy" << m::endl;
		m0.wait();
		m1.wait();
		m0.moveAngle(angle0);
		m0.wait();
		m1.moveAngle(angle1);
		m1.wait();
		msg << "arm: disable motors" << m::endl;
	}

	void autoHomeImp() {
		msg << "arm: homing" << m::tab;
		m0.moveAngle(-1000, 10);
		m1.moveAngle(1000, 8);
		m0.wait();
		m1.stop();
		msg << " m0 ready;" << m::tab;
		m1.moveAngle(-1000, 10);
		m1.wait();
		msg << " m1 ready" << m::tab;
		m0.moveAngle(m0HomeAngle);
		m1.moveAngle(m1HomeAngle);
		m0.wait();
		m1.wait();
		msg << " done" << m::endl;
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
	static constexpr int16_t boardOffsetY = -160;
	static constexpr uint8_t m0HomeAngle = 60;
	static constexpr uint8_t m1HomeAngle = 20;
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
	msg << m::endl << m::tab << "~ CheckersBot v1.0 ~" << m::endl;

	sei();

	arm.init();

	arm.calcAngles(0);
	arm.calcAngles(3);
	arm.calcAngles(28);
	arm.calcAngles(31);
	//arm.take(0);
	//arm.take(3);
	//arm.take(28);
	//arm.take(31);


	while(1)
	{
		led.toggle();
		_delay_ms(500);
	}
}

