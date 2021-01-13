#pragma once

#include <string>

class Configuration
{
public:
	void load(std::string configFilePath);
	bool isValid() const;

private:
	bool valid = false;
};
