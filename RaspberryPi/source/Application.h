#pragma once

#include "GpioManager.h" // <- for debugging, remove later.

class Configuration;
class Command;


class Application
{
public:
	Application();

	void run(const Configuration& configuration);

private:
	void execute(const Command& command);

	GpioManager gpioManager;
};
