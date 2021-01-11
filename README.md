# Holmner6Axis (Under construction...)

This project started in May 2020. Made By Josef Holmner.

A DIY 6 axis robot made more or less from scratch. All the way from hardware, though software down to low level motor drivers.

New material will be uploded as the project progresses. One goal is to provide all material needed for anyone to build the same robot themselves. The current status of the project can be gathered by reading the progress below, which will grow as the project progresses.

## Progress (list will be appended to along the way)
- [x] Rough 3d-model
- [ ] Refined 3d-model ready for printing
- [x] Forward / inverse kinematics prototype script
- [x] I2C slave AVR Atmega328
- [x] I2C master Raspberry Pi 3
- [ ] Complete application for Raspberry Pi 3

## Raspberry Pi setup
1. Get the wiringPi library: `sudo apt-get install wiringpi`
2. Download the `RaspberryPi` directory from this repo to your Raspberry Pi 3
3. Run the build script inside the RaspberryPi directory: `./build_linux.sh` (optional). You may have to change access permission, e.g. `chmod 777 build_linux.sh`.
4. Run the application (must be run with sudo): `sudo ./Holmner6Axis`

## AVR Atmega328 setup
1. Download the `Atmega328` directory from this repo to your computer.
2. Configure your 6 Atmega328's to run at 8 Mhz (or update the source code according to your clock)
3. Built the source using `Microship Studio` (optional)
4. Upload the right `MotorStepperN.hex` to your Atmega328's (note unique .hex files for each)
