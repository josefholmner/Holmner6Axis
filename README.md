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
1. Enable I2C on your Raspberry Pi. A quick web search will provide clear instructions how to.
2. (optional) Increase the I2C baud rate to 400000 kHz. Can be done by adding dtparam=i2c1_baudrate=400000 to /boot/config.txt and then rebooting.
3. Get the wiringPi library: `sudo apt-get install wiringpi`
4. Download the `RaspberryPi` directory from this repo to your Raspberry Pi 3
5. (optional) Run the build script inside the RaspberryPi directory: `./build_linux.sh`. You may have to change access permission, e.g. `chmod 777 build_linux.sh`.
6. Run the application (must be run with sudo): `sudo ./Holmner6Axis`

## AVR Atmega328 setup
1. Download the `Atmega328` directory from this repo to your computer.
2. Configure your 6 Atmega328's to run at 8 Mhz (or update the source code according to your clock)
3. (optional) Built the source using `Microship Studio`
4. Upload the right `MotorStepperN.hex` to your Atmega328's (note unique .hex files for each)
