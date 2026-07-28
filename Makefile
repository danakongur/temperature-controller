all: compile upload monitor
compile:
	arduino-cli compile -b esp32:esp32:esp32c3
upload:
	arduino-cli upload -p /dev/ttyACM0 -b esp32:esp32:esp32c3
monitor:
	arduino-cli monitor -p /dev/ttyACM0
