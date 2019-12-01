#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <util/delay.h>

#include <uart.h>
#include <misc.h>

// Types
//
class Angles {
public:
	Angles() = default;
	Angles(int16_t angle0, int16_t angle1)
		: ang0(angle0)
		, ang1(angle1)
	{}
	int16_t ang0;
	int16_t ang1;
};


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
			msg << "moveAngle " << dAngle/10 << direction << "steps " << stepsPerDAngle(abs(dAngle)) << m::endl;
			if(!busy() && dAngle) {
				speedMax = speed;
				moveSteps(stepsPerDAngle(abs(dAngle)), direction);
			} else {
				msg << "moveAngle: busy" << m::endl;
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
				msg << "moveSteps: !zero and direction" << m::endl;
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
		  o----------------> X
		  |
xOff,yOff | board
   o------|--------
   |      |        |
   |      |        |
		  |
		  V Y
*/


class Board {
public:
	// offset from
	Board(uint16_t x, uint16_t y)
		: offset(x,y)
	{}

	Point indToPoint(uint8_t ind)
	{
		uint8_t row = ind % 8;
		uint8_t col = ind / 8;

		return Point(
			row*cellSize + cellSize/2 + margin + offset.x,
			size + offset.y - (col*cellSize + cellSize/2 + margin));
	}

private:
	static constexpr uint16_t size = 290;
	static constexpr uint8_t margin = 25;
	static constexpr uint8_t cellSize = 30;
	const Point offset;
};

class AngleSolver
{
	public:
		Angles solve(int32_t x, int32_t y)
		{
			Angles angs;
			int64_t a = -2*x;
			int64_t b = -2*y;
			msg << "solver: check for: " << x << y << m::endl; 
			int64_t c = x*x + y*y + r02 - r12;
			int64_t a2b2 = x*x + y*y;
			a2b2 <<= 2;
			int64_t x0 = -(a*c)/a2b2;
			int64_t y0 = -(b*c)/a2b2;
			msg << "solver: x0, y0: " << x0 << y0 << m::endl;
			int64_t d = r02 - (c*c)/a2b2;
			//msg << "d: " << sqrt(d) << m::endl;
			int64_t multa = sqrt((a*a*d)/a2b2);
			int64_t multb = sqrt((b*b*d)/a2b2);
			//msg << "ma, mb: " << multa << multb;
			bool diff = !((a>=0 && x>=0) || (a<=0 && b<=0));
			int64_t rx = x0 + multb;
			int64_t ry = 0;

			msg << "solver: ram usage: " << ramUsage();

			if(diff) {
				ry = y0 + multa;
			} else {
				ry = y0 - multa;
			}
			msg << "solver: joint x,y is:" << rx << ry << m::endl; 

			angs.ang0 = 573 * atan2(ry, rx);
			angs.ang1 = 573 * atan2(y-ry, rx - x);

			msg << "solver: angles:" << angs.ang0 << angs.ang1 << m::endl << m::endl;
			return angs;
		}
		static constexpr uint16_t r0 = 245; 
		static constexpr uint16_t r1 = 176; 
		static constexpr uint16_t r02 = r0*r0;
		static constexpr uint16_t r12 = r1*r1;
};

class MecanicalArm {
public:
	MecanicalArm(StepMotor & motor0, StepMotor & motor1, OutPin & motorEn, Servo & servo)
		: m0(motor0)
		, m1(motor1)
		, motEn(motorEn)
		, srv(servo)
		, b(boardOffsetX, boardOffsetY) // board offset
	{
		motEn.clear();
	}

	void init() {
		srv.init();
		motEn.clear();
		autoHomeImp();
		motEn.set();
	}

	void test() {
		motEn.clear();
		moveToInd(6);
		srv.putSync();
		moveToInd(7);
		srv.putSync();
		//moveToInd(56);
		//srv.putSync();
		//moveToInd(63);
		//srv.putSync();
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
		return solver.solve(target.x, target.y);
	}


private:

	void moveToInd(uint8_t ind)
	{
		Angles a = calcAngles(ind);
		moveToAng(a.ang0, a.ang1);
	}

	void moveToAng(int16_t angle0, int16_t angle1) {
		msg << "\n\rarm: move to " << angle0/10 << angle1/10 << ", from " << currentAng.ang0/10 << currentAng.ang1/10 << m::endl;
		m0.wait();
		m1.wait();
		m0.moveAngle(angle0 - currentAng.ang0);
		m0.wait();
		m1.moveAngle(angle1 - currentAng.ang1);
		m1.wait();
		currentAng = Angles(angle0, angle1);
		msg << "arm: new positions is " << currentAng.ang0/10 << currentAng.ang1/10 << m::endl;
		msg << "arm: disable motors" << m::endl;
	}

	void autoHomeImp() {
		msg << "arm: homing" << m::tab;
		m0.moveAngle(-1000, 10);
		m1.moveAngle(1000, 8);
		m0.wait();
		m1.stop();
		msg << "arm: m0 ready;" << m::tab;
		m1.moveAngle(-1000, 10);
		m1.wait();
		msg << "arm: m1 ready" << m::tab;
		m0.moveAngle(m0HomeAngle);
		m1.moveAngle(m1HomeAngle);
		m0.wait();
		m1.wait();
		currentAng = Angles(2*m0HomeAngle, 2*m1HomeAngle);
		msg << "arm: done" << m::endl;
	}
private:
	StepMotor & m0;
	StepMotor & m1;
	OutPin & motEn;
	Servo & srv;
	Board b;
	AngleSolver solver;
	Angles currentAng;
	const uint8_t lengths[2] = {245, 176};
	static constexpr int8_t boardOffsetY = 65;
	static constexpr int16_t boardOffsetX = -160;
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

	arm.test();
	//arm.take(20);
	//arm.take(3);
	//arm.take(28);
	//arm.take(31);
	
	/* solver test
	AngleSolver solver;
	solver.solve(95,   105);
	solver.solve(100,  230);
	solver.solve(100,  330);
	solver.solve(0,    330);
	solver.solve(-100, 330);
	solver.solve(-110, 110);
	solver.solve(0,    200);
	solver.solve(-60,  110);
	solver.solve(-170, 110);
	*/

	while(1)
	{
		led.toggle();
		_delay_ms(500);
	}
}

