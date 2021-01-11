#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/interrupt.h>

// Using the internal oscillator at 8 Mhz.
// Remember to set the appropriate fuses for clock selection prior to loading this program.
#define F_CPU 8000000UL
#define I2C_ADDR 0x11
#define STOP_BYTE 0xFF
#define BUFFER_SIZE 256
#define FLOAT_SIZE 4

union floatint_t
{
	float f;
	uint8_t i[FLOAT_SIZE];
};

volatile int32_t numBytesReceived;
volatile union floatint_t data[BUFFER_SIZE];

void debugBlinkBit(uint8_t b)
{
	uint8_t numBlinks = b ? 2 : 1;
	uint8_t i;
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
	int32_t i;
	for (i = 7; i >= 0; i--)
	{
		debugBlinkBit(byte & (1<<i));
		_delay_ms(1000);
	}
}

void debugBlinkFloat(float val)
{
	// LED "prints" a float (4 bytes), flashes twice for a 1-bit and once for a 0-bit.
	// Most significant bit first.
	union floatint_t fi;
	fi.f = val;
	debugBlinkByte(fi.i[3]);
	_delay_ms(3000);
	debugBlinkByte(fi.i[2]);
	_delay_ms(3000);
	debugBlinkByte(fi.i[1]);
	_delay_ms(3000);
	debugBlinkByte(fi.i[0]);
}

void dataReceived(uint8_t inData)
{
	uint8_t index = numBytesReceived / FLOAT_SIZE;
	data[index].i[numBytesReceived % FLOAT_SIZE] = inData;
	numBytesReceived++;
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

uint8_t isTransmissionComplete()
{
	if (numBytesReceived >= FLOAT_SIZE && numBytesReceived % FLOAT_SIZE == 0)
	{
		uint8_t lastIndex = (numBytesReceived / FLOAT_SIZE) - 1;

		// Return true if all bytes of the last sent float was STOP_BYTE.
		return data[lastIndex].i[0] == STOP_BYTE && data[lastIndex].i[1] == STOP_BYTE &&
			data[lastIndex].i[2] == STOP_BYTE && data[lastIndex].i[3] == STOP_BYTE;
	}
	return 0;
}

uint8_t min(uint8_t a, uint8_t b)
{
	if (a > b)
	{
		return b;
	}
	return a;
}

void run()
{
	while (1)
	{
		if (isTransmissionComplete())
		{
			// Uncomment this line when temporary test is removed.
			//numBytesReceived = 0;

			// ************ TEMPORARY TEST START ************ //
			// ********************************************** //
			// Compare the first 3 (at most) floats received
			// from the I2C master against 3.14;
			uint8_t numFloats = numBytesReceived / FLOAT_SIZE;
			numBytesReceived = 0;
			uint8_t num = min(3, numFloats);
			uint8_t i;
			for (i = 0; i < num; i++)
			{
				debugBlinkBit(data[i].f < 3.15f && data[i].f > 3.13f);
				_delay_ms(1000);
			}

			_delay_ms(5000);
			// Flash once for each float received.
			for (i = 0; i < numFloats; i++)
			{
				debugBlinkBit(0);
			}
			// ********************************************** //
			// ************* TEMPORARY TEST END ************* //
		}
	}
}

int main(void)
{
	numBytesReceived = 0;

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
