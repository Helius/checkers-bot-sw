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
		StepMotor(OutPin dirPin) 
		: dir(dirPin)
		{
		}

		bool busy() {
			return stepCount;
		}

		void moveAngle(uint8_t angle, bool direction) {
			if(!stepCount) {
				moveSteps(stepsPerAngle(angle), direction);
			}
		}

		void onStepHandler()
		{
			if(stepCount--) {
				// handle timer speed
			} else {
				// done, stop timer
			}
		}
		
	private:
		// we have 10 gear ration and 400 steps per rotation;
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
		OutPin dir;
		uint16_t stepCount = 0;
};

class MecanicalArm {
	public:
		MecanicalArm() = default;
	private:
		StepMotor m1;
		StepMotor m2;
};

// Objects
OutPin led(&DDRB, &PORTB, PIN5);   // on board led
OutPin m1dir(&DDRB, &PORTB, PIN0); // PB0 is direction for motor 1
OutPin m2dir(&DDRD, &PORTD, PIN7); // PD7 is direction for motor 2

StepMotor m1(m1dir);
StepMotor m2(m2dir);

// ISR Handlers
void uart_rx_handler(unsigned char ch)
{
}

// Initialisation
void timer1init()
{
}

void externalInterruptInit()
{

}

int main(void)
{
	uart_init();
	set_receive_interrupt_handler(&uart_rx_handler);
	msg("\n\r\n\rCheckersBot v1.0\n\r\n\r");

	sei();
	while(1)
	{
		_delay_ms(200);
		led.toggle();
	}
}

