#pragma once

#include <vector>

class GpioManager
{
public:
	GpioManager(const GpioManager&) = delete;
	void operator=(const GpioManager&) = delete;

	static GpioManager* getInstance();

	void initialize(const std::vector<int>& motorAddresses);

	void sendI2C(int motor, float data);
	void sendI2C(int motor, const std::vector<float>& data);
	void debugSendI2C(int motor, unsigned int data);

private:
	GpioManager() = default;

	bool isInitialized = false;
	std::vector<int> motorHandles;
};
