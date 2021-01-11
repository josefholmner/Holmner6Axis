#include <iostream>
#include "GpioManager.h"

int main()
{
    std::cout << "--Holmner6Axis--\n\n";
    GpioManager* gpioManager = GpioManager::getInstance();
    gpioManager->initialize({0x11});
    gpioManager->sendI2C(1, {3.141592f, 2.f, 3.141592f, 3.f});

    return 0;
}
