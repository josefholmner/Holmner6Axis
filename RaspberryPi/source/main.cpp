#include <iostream>
#include <stdexcept>

#include "Application.h"
#include "Configuration.h"
#include "Utilities.h"

int main()
{
    std::cout << "-------------Holmner6Axis-------------\n";
    std::cout << "Copywrite 2020. Made by Josef Holmner.\n";

    try
    {
        const std::string homePath = Utilities::getHomePath();
        const std::string configPath = Utilities::makePath(homePath, "RobotConfig.conf");

        Configuration configuration;
        configuration.load(configPath);

        Application application;
        application.run(configuration);
    }
    catch (const std::runtime_error& ex)
    {
        std::cerr << "Error: " << ex.what() << "\n";
    }
    catch (...)
    {
        std::cerr << "Unknown fatal error occurred.\n";
    }

    std::cout << "---------------Goodbye!---------------\n";
    return 0;
}
