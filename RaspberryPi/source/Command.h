#pragma once

#include <vector>
#include <string>

class Command
{
public:
	Command() = default;
	Command(const std::string& str);

	void parse(const std::string& str);

	union CommandData_t
	{
		bool b;
		int i;
		float f;
	};

	enum CommandType
	{
		Quit,
		Debug,
		Invalid
	};

	CommandType type = CommandType::Invalid;
	std::vector<CommandData_t> data;
};

