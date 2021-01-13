#include "Command.h"


Command::Command(const std::string& str)
{
	parse(str);
}

void Command::parse(const std::string& str)
{
	// Todo: add proper parsing.
	if (str.find("quit") != std::string::npos)
	{
		type = CommandType::Quit;
	}
	if (str.find("debug") != std::string::npos)
	{
		type = CommandType::Debug;
	}
}
