## Setting up
Download and install arduino-cli: https://github.com/arduino/arduino-cli
### Install dependencies
```sh
arduino-cli core update-index
arduino-cli core install ardiuno:avr
arduino-cli lib install PID Encoder
```
### Compile and upload the project
```
arduino-cli board list                          # Find your arduino's serial port
arduino-cli board attach serial:///dev/ttyACM0  # Attach to the port
arduino-cli compile AIMC                        # Compile
arduino-cli upload AIMC -p /dev/ttyACM0         # Upload
```
