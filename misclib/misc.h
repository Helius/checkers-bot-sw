#pragma once

#include <stdlib.h>
#include <uart.h>

char htoa(uint8_t a);
void printHex(uint8_t value);
void printHex32(uint32_t value);
/*
void printNumb(int16_t numb);
void msg(const char * str);
void msg(int16_t numb);
*/
uint8_t ramUsage();

class Message {
public:
	enum M_ {
		endl,
		tab
	};

	Message & operator<<(const char * str) {
		while(*(str)) {
			uart_putchar(*(str++));
		}
		return *this;
	}

	Message & operator<<(int16_t numb) {
		char buf[6] = {0};
		int i = 0;
		itoa(numb, buf, 10);
		while(buf[i] != 0) {
			uart_putchar(buf[i]);
			i++;
		}
		uart_putchar(' ');
		return *this;
	}
	
	Message & operator<<(M_ m) {
		switch(m) {
		case endl:
			return *this << endlStr;

		case tab:
			return * this << tabStr;
		}
		return *this;
	}

	static constexpr const char * endlStr = "\n\r";
	static constexpr const char * tabStr = "\t";
};

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
	bool get() {
		return *port & pinM;
	}
private:
	volatile uint8_t * port;
	uint8_t pinM;
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
