# Holmner6Axis (Under construction...)

This project started in May 2020. Made By Josef Holmner.

A DIY 6 axis robot made more or less from scratch. All the way from hardware, though software down to low level motor drivers.

New material will be uploded as the project progresses. One goal is to provide all material needed for anyone to build the same robot themselves. The current status of the project can be gathered by reading the progress below, which will grow as the project progresses.

## Progress (list will be appended to along the way)
- [x] Rough 3d-model
- [ ] Refined 3d-model ready for printing
- [x] Forward / inverse kinematics prototype script
- [ ] C++ motor driver
- [ ] Complete C++ application for Raspberry Pi 3

## Raspberry Pi setup
1. Install CMake (sudo apt-get install cmake).

### Build and run the Raspberry Pi application Holmner6Axis

Clone this repo to a Raspberry Pi.
To build, run the following from the repo root:

```
> cd RaspberryPi/Holmner6Axis
> mkdir build
> cd build
> cmake ..
> sudo cmake --build . --config Release --target install
```

Now the application can be started by running:

```
> Holmner6Axis
```
