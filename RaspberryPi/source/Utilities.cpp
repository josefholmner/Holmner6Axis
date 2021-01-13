#include "Utilities.h"

#include <stdexcept>
#include <fstream>
#include <sys/stat.h>

#ifdef __linux__
#include <unistd.h>
#include <limits.h>
#endif

#ifdef _WIN32
#include <Windows.h>
#endif


namespace
{
#if defined(__linux__)
    std::string getExecutablePath()
    {
        const size_t bufSize = PATH_MAX + 1;
        char buffer[bufSize];
        const char* linkName = "/proc/self/exe";
        readlink(linkName, buffer, bufSize - 1);
        return std::string(buffer);
    }
#elif defined(_WIN32)
    std::string getExecutablePath()
    {
        wchar_t buffer[MAX_PATH];
        GetModuleFileName(NULL, buffer, MAX_PATH);
        std::wstring ws(buffer);
        return std::string(ws.begin(), ws.end());
    }
#else
    std::string getExecutablePath()
    {
        throw std::runtime_error(std::string(
            "Tried to get executable path on unsupported platform."));
    }
#endif

    std::string getDirectoryPathFromFilePath(const std::string& filePath)
    {
        size_t lastSeparator = filePath.find_last_of("/\\");
        if (lastSeparator == std::string::npos)
        {
            return filePath;
        }

        // The path passed points to a file.
        return filePath.substr(0, lastSeparator);
    }

    std::string getPathSeparator()
    {
#if defined(_WIN32)
        return "\\";
#else
        return "/";
#endif
    }
}

bool Utilities::directoryExists(const std::string& dirPath)
{
    struct stat info;
    return stat(dirPath.c_str(), &info) == 0;
}

bool Utilities::fileExists(const std::string& filePath)
{
    std::ifstream f(filePath.c_str());
    return f.good();
}

std::string Utilities::getHomePath()
{
    const std::string exPath = getExecutablePath();
    const std::string exDir = getDirectoryPathFromFilePath(exPath);
    const std::string homeDir = makePath(exDir, "Home");

    if (!directoryExists(homeDir))
    {
        throw std::runtime_error(std::string(
            "getHomePath failed, directory: " + homeDir + " does not exists.\n"));
    }

    return homeDir;
}

std::string Utilities::makePath(const std::string& dirPath, const std::string& fileName)
{
    size_t lastSeparator = dirPath.find_last_of("/\\");
    if (lastSeparator + 1 == dirPath.length())
    {
        // dirPath ends with a path separator.
        return dirPath + fileName;
    }

    return dirPath + getPathSeparator() + fileName;
}
