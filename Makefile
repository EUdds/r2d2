PACKAGES := gateway holoprojector remote

PROTO_SRC := holoprojector/proto/holoprojector.proto
PROTO_OUT_C := holoprojector/firmware/holoprojector/holoprojector.pb.c
PROTO_OUT_H := holoprojector/firmware/holoprojector/holoprojector.pb.h
ARDUINO_SOURCES := holoprojector/firmware/holoprojector/holoprojector.ino

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

arduino-compile: nanopb
	arduino-cli compile --fqbn arduino:megaavr:nona4809 holoprojector/firmware/holoprojector

$(ARDUINO_SOURCES): arduino-compile
	arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:megaavr:nona4809 holoprojector/firmware/holoprojector