all: compile upload monitor
compile:
	arduino-cli compile -b esp32:esp32:esp32
upload:
	arduino-cli upload -p /dev/ttyUSB0 -b esp32:esp32:esp32
monitor:
	arduino-cli monitor -p /dev/ttyUSB0
