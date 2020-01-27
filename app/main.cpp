#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include <util/delay.h>

#include <uart.h>
#include <misc.h>
#include <string.h>


/* TODO
 
 --- feature cut ------
 - moves randomising
 * */

// Types


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

//TODO: fixit
#include <../checkers-simple-algorithm/checkers.h>


class StepMotor {
	public:
		StepMotor(OutPin & dirPin, InPin & zeroPin, uint8_t clockSrc, volatile uint8_t * reg, volatile uint8_t * control, uint8_t defaultSpeed)
		: dir(dirPin)
		, zero(zeroPin)
		, clockSource(clockSrc)
		, value(reg)
		, tcr(control)
		, maxSpeed(defaultSpeed)
		{}

		bool busy() {
			return stepCount > 0;
		}

		void wait() {
			while(busy()) {
				_delay_ms(100);
			}
		}

		void moveAngle(int16_t dAngle, uint8_t speed = 0) {
			if(speed == 0) {
				speed = maxSpeed;
			}
			bool direction = dAngle > 0 ? 0 : 1;
			//msg << "moveAngle " << dAngle/10 << direction << "steps " << stepsPerDAngle(abs(dAngle)) << m::endl;
			if(!busy() && dAngle) {
				speedMax = speed;
				moveSteps(stepsPerDAngle(abs(dAngle)), direction);
			} else {
				//msg << "moveAngle: busy" << m::endl;
			}
		}

		void stop() {
			stopTimer();
			stepCount = 0;
			endPoint = 0;
		}

		void onStepHandler() {
			uint8_t speed;
			if(!zero && dir.get()) {
				//msg << "!zero" << m::endl;
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
				stepCount = 0;
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
				//msg << "moveSteps: !zero and direction" << m::endl;
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
		uint8_t maxSpeed = 40;
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


class BoardGeometry {
public:
	Point indToPoint(uint8_t ind)
	{
		if(ind < 100) { // ind on board
			uint8_t row = ind % 8;
			uint8_t col = ind / 8;

			return Point(
					row*cellSize + cellSize/2 + margin + offsetx,
					size + offsety - (col*cellSize + cellSize/2 + margin));
		} else { // ind outside board
			uint8_t tInd = (ind - 100)%12;
			return Point(offsetx-(tInd%3)*(cellSize+5)-cellSize, offsety+85+(tInd/3)*(cellSize+5));
		}
	}

private:
	static constexpr uint16_t size = 290;
	static constexpr uint8_t margin = 25;
	static constexpr uint8_t cellSize = 30;
	static constexpr int8_t offsety = 65;
	static constexpr int16_t offsetx = -145;//-160;
};

class AngleSolver
{
	public:
		Angles solve(int32_t x, int32_t y)
		{
			Angles angs;
			int64_t a = -2*x;
			int64_t b = -2*y;
			//msg << "solver: check for: " << x << y << m::endl; 
			int64_t c = x*x + y*y + r02 - r12;
			int64_t a2b2 = x*x + y*y;
			a2b2 <<= 2;
			int64_t x0 = -(a*c)/a2b2;
			int64_t y0 = -(b*c)/a2b2;
			//msg << "solver: x0, y0: " << x0 << y0 << m::endl;
			int64_t d = r02 - (c*c)/a2b2;
			//msg << "d: " << sqrt(d) << m::endl;
			int64_t multa = sqrt((a*a*d)/a2b2);
			int64_t multb = sqrt((b*b*d)/a2b2);
			//msg << "ma, mb: " << multa << multb;
			bool diff = !((a>=0 && x>=0) || (a<=0 && b<=0));
			int64_t rx = x0 + multb;
			int64_t ry = 0;

			if(diff) {
				ry = y0 + multa;
			} else {
				ry = y0 - multa;
			}
			//msg << "solver: joint x,y is:" << rx << ry << m::endl; 

			angs.ang0 = 573 * atan2(ry, rx);
			angs.ang1 = 573 * atan2(y-ry, rx - x);

			//msg << "solver: angles:" << angs.ang0 << angs.ang1 << m::endl << m::endl;
			return angs;
		}
		static constexpr uint16_t r0 = 245; 
		static constexpr uint16_t r1 = 167; 
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
	{
		motEn.clear();
	}

	void init() {
		takeInd = 100;
		srv.init();
		motEn.clear();
		autoHomeImp();
		motEn.set();
	}

	void test() {

		motEn.clear();
		//moveToInd(63);
		//srv.putSync();
		//moveToAng(900,0);
		
		//take(25);
		//makeKing(27);
		
		move(27,0);
		move(27,6);
		move(27,57);
		move(27,63);
		
		//move(57,54);
		//move(57,48);
		//move(47,57);
		//move(47,59);
		//move(47,61);
		//move(47,63);

		//moveToPoint(-176,245);
/*		
		int16_t diff0 = 450 - currentAng.ang0;
		int16_t diff1 = 0 - currentAng.ang1;
		m0.moveAngle(diff0);
		m0.wait();
		m1.moveAngle(diff1);
		m1.wait();

		while(){
			uint8_t ind = rand() % 64;
			moveToInd(ind);
			srv.putSync();
		}
*/
		/*
		moveToInd(63-9);
		srv.putSync();
		
		moveToInd(7);
		srv.putSync();
		moveToInd(57);
		srv.putSync();
		moveToInd(63);
		srv.putSync();
		moveToInd(57);
		srv.putSync();
		moveToInd(8);
		srv.putSync();
		moveToInd(63);
		srv.putSync();
		moveToInd(57);
		srv.putSync();
*/
		motEn.set();
	}

	void home() {
		motEn.clear();
		autoHomeImp();
		motEn.set();
	}

	// take from board peice from ind field
	void take(uint8_t ind) {
		motEn.clear();
		moveToInd(ind);
		srv.grabSync();
		moveToInd(takeInd++);
		srv.putSync();
		motEn.set();
	}
	
	// grab taken piece and put it a top of ind
	void makeKing(uint8_t ind, uint8_t from = -1) {
		motEn.clear();
		if(from == -1) {
			moveToInd(--takeInd);
			srv.grabSync();
			makeIndKing(ind);
			srv.putSync();
		} else {
			moveToInd(from);
			srv.grabSync();
			makeIndKing(ind);
			srv.putSync();
		}
		motEn.set();
	}

	// move peice from feild to new field
	void move(uint8_t fromInd, uint8_t toInd)
	{
		//msg << "=== arm: move from " << fromInd << " to " << toInd;
		motEn.clear();
		moveToInd(fromInd);
		srv.grabSync();
		moveToInd(toInd);
		srv.putSync();
		_delay_ms(300);
		motEn.set();
	}

	Angles calcAngles(uint8_t ind)
	{
		Point target = b.indToPoint(ind);
		return solver.solve(target.x, target.y);
	}


private:
	
	void makeIndKing(uint8_t ind)
	{
		Angles a = calcAngles(ind);
		moveToAng(a.ang0+10, a.ang1); // hack for avoid offset
	}

	void moveToInd(uint8_t ind)
	{
		Angles a = calcAngles(ind);
		moveToAng(a.ang0, a.ang1);
	}

	void moveToPoint(int16_t x, int16_t y)
	{
		Angles a = solver.solve(x, y);
		moveToAng(a.ang0, a.ang1);
	}

	void moveToAng(int16_t angle0, int16_t angle1) {
		//msg << "arm: move to " << angle0/10 << angle1/10 << ", from " << currentAng.ang0/10 << currentAng.ang1/10 << m::endl;
		m0.wait();
		m1.wait();
		int16_t diff0 = angle0 - currentAng.ang0;
		int16_t diff1 = angle1 - currentAng.ang1;
		m0.moveAngle(diff0);
		m1.moveAngle(diff1);
		m0.wait();
		m1.wait();
		currentAng = Angles(angle0, angle1);
		//msg << "arm: new positions is " << currentAng.ang0/10 << currentAng.ang1/10 << m::endl;
	}

	void autoHomeImp() {
		//msg << "arm: homing" << m::tab;
		m0.moveAngle(-1150, 10);
		m1.moveAngle(1150, 8);
		m0.wait();
		m1.stop();
		//msg << "arm: m0 ready;" << m::tab;
		m1.moveAngle(-1000, 10);
		m1.wait();
		//msg << "arm: m1 ready" << m::tab;
		m0.moveAngle(initAngle);
		m1.moveAngle(initAngle);
		m0.wait();
		m1.wait();
		currentAng = Angles(initAngle + m0HomeAngle, initAngle + m1HomeAngle);
		//msg << "arm: home done" << m::endl;
	}
private:
	StepMotor & m0;
	StepMotor & m1;
	OutPin & motEn;
	Servo & srv;
	BoardGeometry b;
	AngleSolver solver;
	Angles currentAng;
	//const uint8_t lengths[2] = {245, 176};
	static constexpr uint8_t lengths[2] = {245, 167};
	static constexpr int8_t m0HomeAngle = 35;
	static constexpr int8_t m1HomeAngle = 30;
	static constexpr int8_t initAngle = 50;
	uint8_t takeInd = 100;
};

OutPin eyesLoad(&DDRC, &PORTC, PIN4);
/*
void spi_send16(uint8_t addr, uint8_t data)
{
	_delay_ms(1);
	eyesLoad.clear();
	SPDR = addr;
	while (!(SPSR & (1<<SPIF)));

	SPDR = data;
	while (!(SPSR & (1<<SPIF)));

	SPDR = addr;
	while (!(SPSR & (1<<SPIF)));

	SPDR = data;
	while (!(SPSR & (1<<SPIF)));
	eyesLoad.set();
	_delay_ms(1);
}*/

uint8_t swap(uint8_t b)
{
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

void spi_send16(uint8_t addr, uint8_t data, bool sym = false)
{
	_delay_ms(1);
	eyesLoad.clear();
	SPDR = addr;
	while (!(SPSR & (1<<SPIF)));

	SPDR = data;
	while (!(SPSR & (1<<SPIF)));

	SPDR = addr;
	while (!(SPSR & (1<<SPIF)));
	
	SPDR = sym ? swap(data) : data;
	
	while (!(SPSR & (1<<SPIF)));
	eyesLoad.set();
	_delay_ms(1);
}
const uint8_t eyes_set[4][8] PROGMEM =
{
{ // удивление
	0b11111111,
	0b00000000,
	0b00111100,
	0b01000010,
	0b01000010,
	0b01000010,
	0b00111100,
	0b00000000,
},
{ // ярость
	0b11100000,
	0b00111100,
	0b00000111,
	0b00000000,
	0b00001110,
	0b00001110,
	0b00001110,
	0b00000000,
},
{ // подозрительность
	0b00000000,
	0b00000000,
	0b11111111,
	0b11111111,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
},
{ // normal
	0b00000000,
	0b00000000,
	0b00011000,
	0b00111100,
	0b00111100,
	0b00011000,
	0b00000000,
	0b00000000,
}
};

class Eyes {
	public:
		void init()
		{
			/*
			// "Нормальный" режим работы, не "Тест"
			SendDataSPI2(0x0F00);
			// "Нормальный" режим работы, не "Спящий"
			SendDataSPI2(0x0C01);
			// Максимальная яркость
			SendDataSPI2(0x0A0F);
			// Активные все 8 индикаторов
			SendDataSPI2(0x0B07);
			// "Режим без декодирования"
			SendDataSPI2(0x0900); 
			*/
			spi_send16(0x0A, 0x01);
			spi_send16(0x0B, 0x07);
			spi_send16(0x0C, 0x01);
			spi_send16(0x0F, 0x00);
			spi_send16(0x09, 0x00);
			_delay_ms(100);
			show(Thin);
		}

		void clear()
		{
			for(int i = 1; i < 9; ++i)
			{
				spi_send16(i, 0);
			}
		}

		enum Yeys {
			Curios = 0,
			Anger  = 1,
			Thin   = 2,
			Normal = 3,
			Up     = 4,
			Down   = 5,
			Right  = 6,
			Left   = 7,
		};

		void printBoard(uint32_t state) {
			
			for(uint8_t i = 0; i < 8; i++)
			{
				uint8_t mask = 0;

				for(uint8_t j = 0; j < 4; j++) {
					uint32_t tmp = 1;
					tmp <<= i * 4 + j;
					if(!(state & tmp)) {
						mask |= 1 << (i%2 ? j*2+1 : j*2);
					}
				}
				spi_send16(i+1, mask, false);
			}
		}

		void randomNormal() {
			show(rand()%5 + Thin);
		}

		void show(uint8_t set) {
			switch(set) {
				case Normal:
				case Curios:
				case Anger:
				case Thin:
					for(int i = 1; i < 9; i++)
					{
						spi_send16(i, pgm_read_byte(&(eyes_set[set][8-i])), true);
					}
					break;
				case Up:
					for(int i = 1; i < 5; i++)
					{
						spi_send16(i, 0x00);
					}
					for(int i = 5; i < 9; i++)
					{
						spi_send16(i, pgm_read_byte(&(eyes_set[Normal][10-i])), true);
					}
					break;
				case Down:
					for(int i = 1; i < 5; i++)
					{
						spi_send16(i, pgm_read_byte(&(eyes_set[Normal][6-i])), true);
					}
					for(int i = 5; i < 9; i++)
					{
						spi_send16(i, 0x00);
					}
					break;
				case Right:
					for(int i = 1; i < 9; i++)
					{
						spi_send16(i, pgm_read_byte(&(eyes_set[Normal][8-i]))>>2);
					}
					break;
				case Left:
					for(int i = 1; i < 9; i++)
					{
						spi_send16(i, pgm_read_byte(&(eyes_set[Normal][8-i]))<<2);
					}
					break;
				default:
					break;
			}
		}
};

class VoiceModule {
	
	public:
		VoiceModule(OutPin & txPin, InPin & busyPin)
		: tx(txPin)
		, busy(busyPin)
		{}
		void play(uint8_t fold, uint8_t track)
		{
			wait();
			//7E FF 06 0F 00 01 02 EF 
			//Specify track "002" in the folder “01”
			cmd[3] = 0x0F;
			cmd[5] = fold;
			cmd[6] = track;
			sendData(cmd, 8);
			_delay_ms(100);
		}
/*
		void sleep()
		{
			wait();
			cmd[3] = 0x0A;
			cmd[4] = 0;
			cmd[5] = 0;
			cmd[6] = 0;
			sendData(cmd, 8);
			_delay_ms(100);

		}
*/		
		void wait() {
			while(!busy) {
				//msg << "busy is " << (PINC & (1 << 5)) << m::endl;
				_delay_ms(200);
			}
		}

	private:
		uint8_t cmd[8] = {0x7E, 0xFF, 0x06, 0x00, 0x00, 0x00, 0x00, 0xEF};

		void sendData(uint8_t * cmd, uint8_t len)
		{
			for(int i = 0; i < len; ++i) {
				sendChar(cmd[i]);
			}
		}
		
		void sendChar(uint8_t c)
		{
			c = ~c;
			tx.clear();
			for (uint8_t i = 10; i; i--)
			{
				_delay_us( 1e6 / 9600 );         // bit duration
				if( c & 1 )
					tx.clear();
				else
					tx.set();
				c >>= 1;
			}
		}
	private:
		OutPin & tx;
		InPin & busy;
};

const uint8_t hello[11] PROGMEM = {1,3,4,5,6,7,8,9,119,120,121};
const uint8_t rules[1] PROGMEM = {10};
const uint8_t myMove[3] PROGMEM = {20,21,24};
const uint8_t oppMoves[10]  PROGMEM = {50,51,61,62,63,64,65,68,69,70};
const uint8_t lostPieces[28]  PROGMEM = {40,41,42,43,44,45,46,47,48,49,50,51,52,53,60,61,62,63,64,65,66,67,68,69,70,71,72,73};
const uint8_t winPieces[16]  PROGMEM = {74,75,76,80,20,21,22,23,24,25,26,27,28,29,30,122};
const uint8_t winGame[12]  PROGMEM = {80,81,82,83,84,85,86,87,90,91,92,94};
const uint8_t lostGame[10]  PROGMEM = {91,92,93,94,95,100,101,102,103,104};
const uint8_t waiting[7]  PROGMEM = {110,111,112,113,114,115,119};
const uint8_t jokes[3]  PROGMEM = {116,117,118};
const uint8_t yourTurn[4]  PROGMEM = {131,132,133,134};
const uint8_t arrangeBoard[1]  PROGMEM = {136};
const uint8_t igiveup[3]  PROGMEM = {128,128,130};
const uint8_t hopeyoumakeaturn[1]  PROGMEM = {135};
const uint8_t youhavenopeices[1]  PROGMEM = {127};
const uint8_t iwin[1]  PROGMEM = {125};
const uint8_t youhavenomove[1]  PROGMEM = {126};

class EmoCore 
{
	public:
		enum Event {
			WakeUp,
			Waiting,
			OppMoves,
			IMove,
			LostPieces,
			WinPieces,
			LostGame,
			WinGame,
			YourTurn,
			ArrangeBoard,
			Rules,
			YouHaveNoMove,
			IWin,
			GiveUp,
		};

		EmoCore(VoiceModule & voiceModule)
		: voiceM(voiceModule)
		{}

		void say(Event e)
		{
			uint8_t ind = rand();	
			switch(e) {
				case GiveUp:
					voiceM.play(1,pgm_read_byte(&(igiveup[ind%sizeof(igiveup)])));
					break;
				case YouHaveNoMove:
					voiceM.play(1,pgm_read_byte(&(youhavenomove[0])));
					break;
				case Rules:
					voiceM.play(1,pgm_read_byte(&(rules[0])));// always say rules
					break;
				case WakeUp: // say hello
					voiceM.play(1, pgm_read_byte(&(hello[ind % sizeof(hello)])));
					voiceM.play(1, pgm_read_byte(&(hello[ind % sizeof(hello)])));
					voiceM.wait();
					_delay_ms(1000);
					voiceM.play(1,pgm_read_byte(&(rules[0])));// always say rules
					break;
				case Waiting: // say smth, joke?
					if(rand()%10 == 5) {
						voiceM.play(1, pgm_read_byte(&(jokes[ind % sizeof(jokes)])));
					} else {
						voiceM.play(1, pgm_read_byte(&(waiting[waitingInd++ % sizeof(waiting)])));
					}
					break;
				case YourTurn:
						voiceM.play(1, pgm_read_byte(&(yourTurn[ind % sizeof(yourTurn)])));
					break;
				case OppMoves:
					voiceM.play(1, pgm_read_byte(&(oppMoves[ind % sizeof(oppMoves)])));
					break;
				case IMove:
					voiceM.play(1, pgm_read_byte(&(myMove[ind % sizeof(myMove)])));
					break;
				case LostPieces:
					voiceM.play(1, pgm_read_byte(&(lostPieces[ind % sizeof(lostPieces)])));
					break;
				case WinPieces: 
					voiceM.play(1, pgm_read_byte(&(winPieces[ind % sizeof(winPieces)])));
					break;
				case LostGame:
					voiceM.play(1, pgm_read_byte(&(lostGame[ind % sizeof(lostGame)])));
					break;
				case IWin:
					voiceM.play(1, pgm_read_byte(&(iwin[0])));
				case WinGame:
					voiceM.play(1, pgm_read_byte(&(winGame[ind % sizeof(winGame)])));
					break;
				case ArrangeBoard:
					voiceM.play(1, pgm_read_byte(&(arrangeBoard[ind % sizeof(arrangeBoard)])));
					break;
				default:
					break;
			}
			
		}
	private:
	
		VoiceModule & voiceM;
		uint8_t waitingInd = 0;
};

class HumanMoveDetector 
{
	public:
		HumanMoveDetector(OutPin & loadPin)
			: load(loadPin)
		{}

		uint32_t getState() {
			uint32_t s;
			load.set();
			uint8_t d[4];
			for(int i = 0; i < 4; ++i) {
				d[i] = spi_read();
			}
			load.clear();
/*
			msg << "spi: ";
			printHex(d[0]);	
			printHex(d[1]);	
			printHex(d[2]);	
			printHex(d[3]);	
			msg << m::endl;
*/
			s = d[3];
			s <<= 8;
			s |= d[2];
			s <<= 8;
			// hack because i soldered 0xF000 pin wrong
			uint8_t tmp = swap(d[1]);
			d[1] = ((tmp << 4) & 0xF0) + (d[1] & 0x0F);
			s |= d[1];
			s <<= 8;
			s |= d[0];

			//msg << "board getState: ";
			//printHex32(s);
			//msg << m::endl;
			return s;
		}

		void saveBoard() {
			//msg << "------- save board --------" << m::endl;
			prev = getState();
		}

		bool waitForMoveStarted() {
			return getState() != prev;
		}
/*
		uint8_t getChangesCount()
		{
			uint32_t state = getState();
			uint32_t diff = state ^ prev;
			uint8_t cnt = 0;
			
			msg << "board changed ";
			printHex32(state);
			printHex32(prev);
			printHex32(diff);
			printHex32(diff&state);
			msg << m::endl;
			
			for(int i = 0; i < 32; ++i) {
				if(diff & 1)
				{
					cnt++;
				}
				diff >>= 1;
			}
			return cnt;
		}
*/
		BoardDiff getBoardDiff() {
			BoardDiff bdiff;
			uint32_t state = getState();
			uint32_t diff = state ^ prev;
			for(uint8_t i = 0; i < 32; ++i) {
				if(diff & 1)
				{
					if(state & diff & 1) {
						bdiff.setUp((i/4)%2 ? i*2+1 : i*2);
					} else {
						bdiff.setDown((i/4)%2 ? i*2+1 : i*2);
					}
				}
				diff >>= 1;
				state >>= 1;
			}
			return bdiff;
		}

		inline bool isBoardInit()
		{
			return getState() == 0x000ff000;
		}
	
	private:

		uint8_t spi_read()
		{
			SPDR = 0;
			/* Wait for reception complete */
			while (!(SPSR & (1<<SPIF)));
			/* Return data register */
			return SPDR;
		}

	private:
		OutPin & load;
		uint32_t prev;
};

// Objects
//OutPin led(&DDRB, &PORTB, PIN5);     // on board led
OutPin m0dir(&DDRD, &PORTD, PIN7);   // PD7 is direction for motor 2
OutPin m1dir(&DDRB, &PORTB, PIN0);   // PB0 is direction for motor 1
OutPin motEn(&DDRB, &PORTB, PIN2);   // enable morots drivers
InPin zero0pin(&DDRD, &PORTD, &PIND, PIN2); // zero switch pin (low: pressed)
InPin zero1pin(&DDRD, &PORTD, &PIND, PIN4); // zero switch pin (low: pressed)
OutPin magnito(&DDRC, &PORTC, PIN0); // electro magnet (low: on, hight: off)
OutPin servoPin(&DDRD, &PORTD, PIN3);   // arm servoPin (OC2B)
OutPin boardLoad(&DDRC, &PORTC, PIN1);
// voice module
InPin voiceBusy (&DDRC, &PORTC, &PINC, 5);
OutPin voiceTx (&DDRC, &PORTC, 2);

volatile uint8_t t0value;
StepMotor m0(m0dir, zero0pin, 0u, &t0value, &TCCR0B, 45);
StepMotor m1(m1dir, zero1pin, 2u, &ICR1L, &TCCR1B, 18);

Servo servo(magnito);
MecanicalArm arm(m0, m1, motEn, servo);
Eyes eyes;
VoiceModule voice(voiceTx, voiceBusy);
EmoCore emoCore(voice);
HumanMoveDetector moveDetector(boardLoad);
Game game;

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

class LoopDelay {
	public:
		LoopDelay() = delete;
		LoopDelay(uint8_t delayCnt, bool fireOnStart) 
			: delay(delayCnt)
		{
			if(fireOnStart) {
				cnt = delay;
			}
		}

		operator bool() {
			if(cnt == delay) {
				cnt = 0;
				return true;
			} else {
				cnt++;
				return false;
			}
		}
	
	private:
		uint8_t cnt = 0;
		uint8_t delay = 0;
};

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
	eyesLoad.set();
	
	// init spi for checkers board and eyes
	InPin miso(&DDRB, &PORTB, &PINB, PIN4);
	OutPin mosi(&DDRB, &PORTB, PIN3);
	OutPin sck(&DDRB, &PORTB, PIN5);
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR1); // enable SPI, MASTER mode, prescaler 1/v	

	uart_init(0);
	msg << m::endl << m::tab << "~ CBot v1.0 ~" << m::endl;
	//msg << "mem: " << (uint16_t)memfree() << "sizes:" << sizeof(Moves) << sizeof(Move2) << sizeof(Step) << m::endl;

	// use eeprom for seed
	uint16_t val = eeprom_read_word(0);
	srand(val);
	eeprom_write_word(0, rand());

	sei();

	eyes.init();
	arm.init();
	//arm.test();
	emoCore.say(EmoCore::Event::WakeUp);
	eyes.randomNormal();

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
		_delay_ms(100);
		
		//msg << "game st is " << game.getState() << m::endl;

		switch(game.getState())
		{
			case Game::WaitForBoardInit:
				{
					LoopDelay d10(20, true);
					LoopDelay d100(4, false);
					while(!moveDetector.isBoardInit()) {
						if(d10) {
							emoCore.say(EmoCore::Waiting);
							_delay_ms(1000);
							if(d100) {
								emoCore.say(EmoCore::Rules);
							}

						} else {
							eyes.printBoard(moveDetector.getState());
							_delay_ms(1000);
						}
					}
					moveDetector.saveBoard();
					game.startGame();
				}
				break;
			case Game::TheirMove:
				{
					// calc possible moves
					Moves ms;
					game.getTheirMove(ms);
					if(!ms.size()) {
						//msg << "They have no moves!" << m::endl;
						if(game.doIWin()) {
							eyes.show(Eyes::Curios);
							emoCore.say(EmoCore::WinGame);
							_delay_ms(1000);
							emoCore.say(EmoCore::WinGame);
							game.reset();
							arm.init();
						} else {
							eyes.show(Eyes::Curios);
							emoCore.say(EmoCore::YouHaveNoMove);
							_delay_ms(1000);
							emoCore.say(EmoCore::WinGame);
							_delay_ms(1000);
							game.reset();
							arm.init();
						}

					}

					// wait move started
					LoopDelay ld20 (12, true);
					while(!moveDetector.waitForMoveStarted()) {
						if(ld20) {
							emoCore.say(EmoCore::YourTurn);
							_delay_ms(1000);
						} else {
							_delay_ms(2000);
							eyes.randomNormal();
						}
					}

					// move started
					uint8_t waitCount = 0;
					while(1) { // wait move finish
						waitCount++;
						// check board changes match moves
						BoardDiff bdiff = moveDetector.getBoardDiff();
						if(bdiff) {
							//msg << "diff: " << bdiff.up[0] << " to " << bdiff.down << m::endl;
							if(ms.size() && ms.hasTakes()) {
								//msg << "They have to take" << m::endl;
								Move2 m = bdiff.match(ms);
								if(m) {
									//msg << "move recognized" << m::endl;
									game.applyTheirMove(m);
									moveDetector.saveBoard();
									_delay_ms(500);
									emoCore.say(EmoCore::LostPieces);
									break;
								}
							} else {
								Move2 rawMove(bdiff.up[0]);
								Step step(bdiff.down);
								// became King
								if(bdiff.down > 54) {
									step.becameKing = true;
								}
								rawMove.addStep(step);
								game.applyTheirMove(rawMove);
								moveDetector.saveBoard();
								_delay_ms(500);
								emoCore.say(EmoCore::OppMoves);
								break;
							}
						}
						_delay_ms(300);
					}
				}
				break;

			case Game::MyMove:
				{
					// say smth
					Move2 move = game.getMyMove();
					if(move) {
						//msg << "find my move " << move.getFrom() << move.front().to << m::endl;
						bool hasTake = false;
						arm.move(move.getFrom(),move.getStep(move.size()-1).to);

						bool needPlaceKing = move.becameKing();

						for(uint8_t i = 0; i < move.size(); ++i) {
							Step s = move.getStep(i);
							if(s.take != -1) {
								if(needPlaceKing) {
									arm.makeKing(move.front().to, s.take);
									needPlaceKing = false;
								} else {
									arm.take(s.take);
								}
								hasTake = true;
							}
							if(needPlaceKing) {
								arm.makeKing(move.front().to);
							}
						}

						if(hasTake) {
							emoCore.say(EmoCore::WinPieces);
						} else {
							emoCore.say(EmoCore::IMove);
						} 
						moveDetector.saveBoard();
						arm.home();
						_delay_ms(500);
						game.myMoveApplyed(move);
					} else {
						//msg << "MyMove:no move" << m::endl;
						eyes.show(Eyes::Anger);
						if(!game.doTheirWin()) {
							emoCore.say(EmoCore::GiveUp);
						}
						emoCore.say(EmoCore::LostGame);
						// i give up (no move were found)
						game.giveUp();
						arm.init();
					}
				}
				break;
		}
	}
}

