
// Pigpio includes.
#include <pigpio.h>

// Standard library includes.
#include<cmath>
#include <iostream>
#include <thread>
#include <vector>

std::vector<double> calcRamp(double th_3, double w_max, double a, double step_angle)
{
    double t1 = w_max / a;
    double th_1 = w_max * w_max / (2.0 * a);
    double th_2 = th_3 - th_1;
    double t2 = t1 + (th_2 - th_1) / w_max;

    std::vector<double> ts;
    ts.reserve(static_cast<size_t>(1.0 + th_3 / step_angle));
    double th = step_angle;
    
    while (th < th_1)
    {
        ts.push_back(sqrt(2.0 * th / a));
        th += step_angle;
    }

    while (th < th_2)
    {
        ts.push_back(t1 + (th - th_1) / w_max);
        th += step_angle;
    }

    while (th <= th_3)
    {
        ts.push_back(t2 + w_max / a - sqrt(pow(w_max/a, 2.0) - 2.0 * (th - th_2) / a));
        th += step_angle;
    }

    return ts;
}

void executeRamp(int pin, const std::vector<double>& ts)
{
    if (gpioSetMode(pin, PI_OUTPUT) != 0)
    {
        std::cerr << "gpioSetMode failed\n";
        return;
    }
    
    const uint32_t startTime = gpioTick();
    for (const double t : ts)
    {
        // double t in seconds and uint32_t times in microseconds.
        const uint32_t step_delta_time = static_cast<uint32_t>(t * 1000000.0);
        const uint32_t next_step_half_time = startTime + step_delta_time / 2;
        const uint32_t next_step_time = startTime + step_delta_time;

        while (gpioTick() < next_step_half_time){}
        gpioWrite(pin, 1);
        while (gpioTick() < next_step_time){}
        gpioWrite(pin, 0);
    }

    const uint32_t endTime = gpioTick();
    std::cout << "Total time: " << endTime - startTime << "us.\n";
    std::cout << "Total steps: " << ts.size() << ".\n";
}

int main()
{
    std::cout << "Hello from Holmner6Axis\n";

    if (gpioInitialise() < 0)
    {
        std::cerr << "gpioInitialize failed\n";        
        return 1;
    }

    const auto ramp = calcRamp(2000.0, 400.0, 400.0, 1.8);
    std::thread th1([&ramp]{executeRamp(ramp);}); // Just for fun, run on worker thread.
    th1.join();

    gpioTerminate();
    return 0;
}