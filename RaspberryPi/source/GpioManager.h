#pragma once

#include <vector>

class GpioManager
{
public:
	bool initialize(const std::vector<int>& motorAddresses);

	// TODO: motor should be an enum, not an int.
	void sendI2C(int motor, float data);
	void sendI2C(int motor, std::vector<float> data);
	void debugSendI2C(int motor, unsigned int data);

	bool isInitialized();

private:
	bool initialized = false;
	std::vector<int> motorHandles;
};
