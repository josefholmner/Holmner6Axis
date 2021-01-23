// Using the internal oscillator at 8 Mhz.
// Remember to set the appropriate fuses for clock selection prior to loading this program.
#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/interrupt.h>

#define I2C_ADDR 0x11
#define BUFFER_SIZE 80
#define COMMAND_SIZE 8

struct MotorCommand
{
	float speed;
	float angle;
};

union command_t
{
	struct MotorCommand mc;
	uint8_t i[COMMAND_SIZE];
};

volatile uint8_t lastByteRecievedInd;
volatile uint8_t lastExecutedCommandInd;
volatile union command_t circularBuffer[BUFFER_SIZE];
uint8_t MAX_COMMAND_INDEX;

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

void debugBlinkNum(uint8_t num)
{
	uint8_t i;
	for(i = 0; i < num; i++)
	{
		debugBlinkBit(0);
	}
}

void dataReceived(uint8_t inData)
{
	lastByteRecievedInd++;	
	if (lastByteRecievedInd >= BUFFER_SIZE)
	{
		// Reset the buffer index, i.e. make it behave as a circular buffer.
		lastByteRecievedInd = 0;
	}

	uint8_t index = lastByteRecievedInd / COMMAND_SIZE;
	circularBuffer[index].i[lastByteRecievedInd % COMMAND_SIZE] = inData;
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

uint8_t getLastReceivedCommandInd()
{
	if (lastByteRecievedInd < 7)
	{
		return MAX_COMMAND_INDEX;
	}
	else
	{
		return (lastByteRecievedInd - 7) / COMMAND_SIZE;
	}
}

void run()
{
	while (1)
	{
		uint8_t lastReceivedCommandInd = getLastReceivedCommandInd();
		if (lastReceivedCommandInd == lastExecutedCommandInd)
		{
			continue;
		}
		
		uint8_t commandToExecuteInd = lastExecutedCommandInd + 1;
		if (commandToExecuteInd > MAX_COMMAND_INDEX)
		{
			commandToExecuteInd = 0;
		}
		
		// Temporary debug blink:
		debugBlinkNum((uint8_t)circularBuffer[commandToExecuteInd].mc.speed);
		_delay_ms(1000);
		debugBlinkNum((uint8_t)circularBuffer[commandToExecuteInd].mc.angle);
		_delay_ms(1000);

		lastExecutedCommandInd = commandToExecuteInd;	
	}
}

int main(void)
{
	MAX_COMMAND_INDEX = (BUFFER_SIZE / COMMAND_SIZE) - 1;
	
	// Buffer index is initialized at last index, that way the first byte received is written to index 0.
	lastByteRecievedInd = BUFFER_SIZE - 1;
	lastExecutedCommandInd = MAX_COMMAND_INDEX;

	// Set port PB1 as output (used for LED debugging).
	DDRB |= (1<<DDB1);

	setupI2c();
	float debugFloat = 4.0f;
	debugBlinkNum((uint8_t)debugFloat);
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
