#pragma once

#include <string>

class Utilities
{
public:
	static bool directoryExists(const std::string& dirPath);
	static bool fileExists(const std::string& filePath);

	static std::string getHomePath();

	static std::string makePath(const std::string& dirPath, const std::string& fileName);

	// Dissallow instantiation of class Utilities.
	Utilities() = delete;
};

