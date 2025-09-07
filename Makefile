PACKAGES := gateway holoprojector remote interfaces

PROTO_SRC := holoprojector/proto/holoprojector.proto
PROTO_OUT_C := holoprojector/firmware/holoprojector/holoprojector.pb.c
PROTO_OUT_H := holoprojector/firmware/holoprojector/holoprojector.pb.h
ARDUINO_SOURCES := holoprojector/firmware/holoprojector/holoprojector.ino
ARDUINO_OUT_DIR := fw_build/holoprojector
ARDUINO_BIN := $(ARDUINO_OUT_DIR)/holoprojector.ino.bin

all: $(ARDUINO_SOURCES) nanopb
	colcon build --packages-select $(PACKAGES)

deps:
	rosdep install -i --from-path . --rosdistro jazzy -y

nanopb: $(PROTO_OUT_C) $(PROTO_OUT_H)

$(PROTO_OUT_C) $(PROTO_OUT_H): $(PROTO_SRC)
	protoc -I=holoprojector/proto --nanopb_out=holoprojector/firmware/holoprojector $(PROTO_SRC)

arduino-setup:
	arduino-cli lib install "Adafruit NeoPixel"
	arduino-cli lib install "Servo"

$(ARDUINO_BIN): $(ARDUINO_SOURCES) $(PROTO_OUT_C) $(PROTO_OUT_H)
	arduino-cli compile --fqbn arduino:megaavr:nona4809 --output-dir $(ARDUINO_OUT_DIR) holoprojector/firmware/holoprojector

arduino-upload: $(ARDUINO_BIN)
	arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:megaavr:nona4809 holoprojector/firmware/holoprojector