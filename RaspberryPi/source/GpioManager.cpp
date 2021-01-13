#include "GpioManager.h"

#include <iostream>
#include <string>

#ifdef __linux__
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <unistd.h>
#endif

union floatint_t
{
	float f;
	unsigned int i;
};

namespace
{
	void printNotInitializedError(std::string caller)
	{
		std::cerr << caller << " failed, GpioManager has not been initialized. Do so by calling "
			<< "GpioManager::initialize(...).\n";
	}
}

bool GpioManager::initialize(const std::vector<int>& motorAddresses)
{
	constexpr int numMotors = 1; // <- for debugging, set to 6 later.
	if (motorAddresses.size() != numMotors)
	{
		std::cerr << "Initialization of GpioManager failed, expected " << numMotors << " motor "
			<< "addresses but got "<< motorAddresses.size() << ".\n";
		return false;
	}

#ifdef __linux__
	if (wiringPiSetupGpio() < 0)
	{
		std::cerr << "Initialization of GpioManager failed, call to wiringPiSetupGpio returned "
			<< "error.\n";
		return false;
	}

	for (int i = 0; i < numMotors; i++)
	{
		int handle = wiringPiI2CSetup(motorAddresses[i]);
		if (handle == -1)
		{
			std::cerr << "Initialization of GpioManager failed, call to wiringPiI2CSetup() "
				<< "returned -1.\n";
			motorHandles.clear();
			return false;
		}

		motorHandles.push_back(handle);
	}
#endif

	initialized = true;
	return true;
}


void GpioManager::sendI2C(int motor, float inData)
{
	std::vector<float> data = {inData};
	sendI2C(motor, data);
}

void GpioManager::sendI2C(int motor, std::vector<float> data)
{
	constexpr unsigned int I2CStop = 0xFFFFFFFF;
	constexpr unsigned int MaxNumBytes = 256;

	if (!initialized)
	{
		printNotInitializedError("GpioManager::sendI2C(...)");
		return;
	}

	if (sizeof(float) * data.size() > MaxNumBytes)
	{
		std::cerr << "Cannot send more than " << MaxNumBytes << "bytes in one transfer. "
			<< "Nothing is sent.";
		return;
	}

	// Append the I2CStop bytes. This wont effect the original passed vector since it is
	// passed by value.
	floatint_t stop;
	stop.i = I2CStop;
	data.push_back(stop.f);

#ifdef __linux__
	int bytesSent = write(motorHandles[motor - 1], &data[0], sizeof(float) * data.size());
	if (static_cast<unsigned int>(bytesSent) != sizeof(float) * data.size())
	{
		std::cerr << "sendI2C failed. Please check the I2C connection. \n";
	}
#endif
}

void GpioManager::debugSendI2C(int motor, unsigned int data)
{
	
#ifdef __linux__
	int bytesSent = write(motorHandles[motor - 1], &data, sizeof(unsigned int));
	if (static_cast<unsigned int>(bytesSent) != sizeof(unsigned int))
	{
		std::cerr << "debugSendI2C failed. Please check the I2C connection.\n";
	}

	std::cout << "Byes sent: " << bytesSent << "\n";
#endif
}

bool GpioManager::isInitialized()
{
	return initialized;
}
