
// Pigpio includes.
#include <pigpio.h>

// Standard library includes.
#include<cmath>
#include <iostream>
#include <thread>
#include <vector>

std::vector<double> calcRamp(double angleGoal, double wMax, double maxAcc, double stepAngle)
{
    const double a = maxAcc;
    const double th3 = angleGoal;
    const double t1 = wMax / a;
    const double th1 = wMax * wMax / (2.0 * a);
    const double th2 = th3 - th1;
    const double t2 = t1 + (th2 - th1) / wMax;

    std::vector<double> ts;
    ts.reserve(static_cast<size_t>(1.0 + th3 / stepAngle));
    double th = stepAngle;
    
    while (th < th1)
    {
        ts.push_back(sqrt(2.0 * th / a));
        th += stepAngle;
    }

    while (th < th2)
    {
        ts.push_back(t1 + (th - th1) / wMax);
        th += stepAngle;
    }

    while (th <= th3)
    {
        ts.push_back(t2 + wMax / a - sqrt(pow(wMax/a, 2.0) - 2.0 * (th - th2) / a));
        th += stepAngle;
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

    gpioWrite(pin, 0);
    
    const size_t Max = ts.size();
    const uint32_t startTime = gpioTick();
    for (size_t i = 1; i < Max; i++)
    {
        // double t in seconds and uint32_t times in microseconds.
        const uint32_t mid_step_rel_time = static_cast<uint32_t>((ts[i-1] + ts[i]) / 2.0 * 1000000.0);
        const uint32_t step_rel_time = static_cast<uint32_t>(ts[i] * 1000000.0);
        const uint32_t mid_step_wall_time = startTime + mid_step_rel_time;
        const uint32_t step_wall_time = startTime + step_rel_time;

        while (gpioTick() < mid_step_wall_time){}
        gpioWrite(pin, 1);
        while (gpioTick() < step_wall_time){}
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

    const std::vector<double> ramp = calcRamp(2160.0, 2000.0, 2000.0, 0.45);
    std::thread th1([&]{executeRamp(21, ramp);}); // Just for fun, run on worker thread.
    th1.join();

    gpioTerminate();
    return 0;
}