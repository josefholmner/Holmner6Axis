
// Pigpio includes.
#include <pigpio.h>

// Standard library includes.
#include <iostream>
#include <thread>

void dummy(int pin)
{
    if (gpioInitialise() < 0)
    {
        std::cerr << "gpioInitialize failed\n";
        gpioTerminate();
        return;
    }

    if (gpioSetMode(pin, PI_OUTPUT) != 0)
    {
        std::cerr << "gpioSetMode failed\n";
        gpioTerminate();
        return;
    }


    for (int i = 0; i < 5; i++)
    {
        gpioWrite(pin, 1);
        gpioDelay(1000000);
        gpioWrite(pin, 0);
        gpioDelay(1000000);
    }

    gpioTerminate();
}

int main()
{
    std::cout << "Hello from Holmner6Axis\n";
    std::thread th1([]{dummy(21);}); // Just for fun, run in worker thread.
    th1.join();
    return 0;
}