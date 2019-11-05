#pragma once

char htoa(uint8_t a);
void printHex(uint8_t value);
void printNumb(int16_t numb);
void msg(const char * str);
uint8_t ramUsage();

// OutPin led(&DDRB, &PORTB, 5);
class OutPin {
public:
	OutPin() = delete;
	OutPin(volatile uint8_t * ddr, volatile uint8_t * port, int pin)
		: port(port)
	{
		pinM = 1 << pin;
		*ddr |= pinM;
	}
	void set()
	{
		*port |= pinM;
	}
	void clear()
	{
		*port &= ~pinM; 
	}
	void toggle()
	{
		*port ^= pinM;
	}
private:
	volatile uint8_t * port;
	int pinM;
};

class InPin{
public:
	InPin() = delete;
	InPin(volatile uint8_t * ddr, volatile uint8_t * port, volatile uint8_t * in, int pin)
		: port(port)
		, in(in)
	{
		setPullUp(false);
		pinM = 1 << pin;
		*ddr &= ~pinM;
	}

	void setPullUp(bool enable)
	{
		if(enable) {
			*port |= pinM;
		} else {
			*port &= ~pinM;
		}
	}
	explicit operator bool() {
		return *in & pinM;
	}
private:
	volatile uint8_t * port;
	volatile uint8_t * in;
	int pinM;
	
};
