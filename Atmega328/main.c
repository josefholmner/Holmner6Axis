#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/interrupt.h>

// Using the internal oscillator at 8 Mhz.
// Remember to set the appropriate fuses for clock selection prior to loading this program.
#define F_CPU 8000000UL
#define I2C_ADDR 0x11

volatile uint8_t bytesReceived;
volatile uint8_t data[4];

void debugBlinkBit(uint8_t b)
{
	int numBlinks = b ? 2 : 1;
	int i;	
	for (i = 0; i < numBlinks; i++)
	{
		PORTB |= (1<<PORTB1);
		_delay_ms(200);
		PORTB &= ~(1<<PORTB1);
		_delay_ms(200);		
	}
}

void debugBlinkByte(uint8_t byte)
{
	// LED "prints" a byte, flashes twice for a 1-bit and once for a 0-bit.
	// Most significant bit first.
	int i;
	for (i = 7; i >= 0; i--)
	{
		debugBlinkBit(byte & (1<<i));
		_delay_ms(1000);
	}
	
	_delay_ms(3000);
}

void dataReceived(uint8_t inData)
{	
	data[bytesReceived++] = inData;
}

void setupI2c()
{
	cli();
	// Set device address.
	TWAR = (I2C_ADDR << 1);
	
	// Setup TWCR, see data sheet for details.
	TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
	sei();
}

void run()
{
	while (1)
	{
		if (bytesReceived >= 4)
		{
			bytesReceived = 0;
			debugBlinkByte(data[0]);
			debugBlinkByte(data[1]);
			debugBlinkByte(data[2]);
			debugBlinkByte(data[3]);
		}		
	}
}

int main(void)
{
	bytesReceived = 0;
	
	// Set port PB1 as output (used for LED debugging).
	DDRB |= (1<<DDB1);
	
	setupI2c();
	debugBlinkByte(0x42);	
	run();
}

// Data incoming from the I2C bus.
ISR(TWI_vect)
{
	if (TW_STATUS == TW_SR_DATA_ACK)
	{
		// Got data from the I2C master.
		dataReceived(TWDR);
	}
	else if (TW_STATUS == TW_BUS_ERROR)
	{
		TWCR = 0;
	}

	// Prepare TWCR register for more data.
	TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
}

