#include "Configuration.h"

#include <iostream>

#include "Utilities.h"

void Configuration::load(std::string configFilePath)
{
	if (!Utilities::fileExists(configFilePath))
	{
		std::cerr << "Could not load configuration file: " << configFilePath <<
			". File does not exist.\n";
		return;
	}
	
	valid = true;
}

bool Configuration::isValid() const
{
	return valid;
}
