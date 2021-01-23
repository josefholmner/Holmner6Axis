#pragma once

#include <vector>

class GpioManager
{
public:
	bool initialize(const std::vector<int>& motorAddresses);

	// TODO: motor should be an enum, not an int.
	void sendI2C(int motor, float data) const;
	void sendI2C(int motor, const std::vector<float>& data) const;
	void debugSendI2C(int motor, unsigned int data) const;

	bool isInitialized() const;

private:
	bool initialized = false;
	std::vector<int> motorHandles;
};
