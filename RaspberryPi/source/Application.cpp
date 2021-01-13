#include "Application.h"

#include <iostream>
#include <stdexcept>
#include <string>

#include "Configuration.h"
#include "Command.h"

namespace
{
	Command getCommandFromCommandLine()
	{
		Command command;

		do
		{
			std::cout << "Rob>: " << std::flush;
			std::string input;
			std::getline(std::cin, input);

			command.parse(input);
			if (command.type == Command::CommandType::Invalid)
			{
				std::cout << "Invalid command. Type 'help' for usage instructions.\n";
			}
		} while (command.type == Command::CommandType::Invalid);
		
		return command;
	}

	void printWelcomeMessage()
	{
		std::cout << "[Welcome message placeholder]\n";
	}
}

Application::Application()
{
	gpioManager.initialize({ 0x11 }); // <- for debugging, remove later.
}

void Application::run(const Configuration& configuration)
{
	if (!configuration.isValid())
	{
		throw std::runtime_error(std::string(
			"Invalid configuration passed to Application::run(...)."));
	}

	printWelcomeMessage();

	while (1)
	{
		const Command command = getCommandFromCommandLine();
		if (command.type == Command::CommandType::Quit)
		{
			return;
		}

		execute(command);
	}
}

void Application::execute(const Command& command)
{
	if (command.type == Command::CommandType::Invalid)
	{
		throw std::runtime_error(std::string(
			"Tried to execute invalid command."));
	}

	switch (command.type)
	{
		case Command::CommandType::Debug:
			gpioManager.sendI2C(1, { 3.141592f, 3.f, 3.1492f, 2.f }); // <- for debugging, remove later.
			break;
		default:
			break;
	}
}
