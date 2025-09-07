#include <Adafruit_NeoPixel.h>
#include <Servo.h>

#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "holoprojector.pb.h"

#define NEOPIXEL_PIN    2
#define NEOPIXEL_COUNT  7

#define SERVO1_PIN     3
#define SERVO2_PIN     5

Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
holoprojector_LEDCommand led_cmds[NEOPIXEL_COUNT];
size_t led_count = 0;
Servo servo1;
Servo servo2;

// Callback for decoding repeated LEDCommand
bool led_command_callback(pb_istream_t *stream, const pb_field_t *field, void **arg) {
    if (led_count >= NEOPIXEL_COUNT) return false;
    return pb_decode(stream, holoprojector_LEDCommand_fields, &led_cmds[led_count++]);
}

void setup()
{
  // Start the Serial communication to send messages to the computer
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  pixels.begin(); // This initializes the NeoPixel library. 
  pixels.fill(pixels.Color(0, 212, 255)); // Initialize all pixels to 'off'
  pixels.show(); // Initialize all pixels to 'off'
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    // Robust framed protocol: [0xAA][0x55][len_lo][len_hi][payload...][crc_lo][crc_hi]
    // CRC is CRC16-CCITT over payload only.

    enum ParserState { WAIT_PREAMBLE1, WAIT_PREAMBLE2, READ_LEN_LO, READ_LEN_HI, READ_PAYLOAD, READ_CRC_LO, READ_CRC_HI };
    static ParserState state = WAIT_PREAMBLE1;
    static uint16_t expected_len = 0;
    static uint16_t bytes_read = 0;
    static uint8_t payload[512];
    static uint16_t recv_crc = 0;

    auto crc16_ccitt = [](const uint8_t* data, size_t len) -> uint16_t {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; ++i) {
            crc ^= (uint16_t)data[i] << 8;
            for (uint8_t j = 0; j < 8; ++j) {
                if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
                else crc <<= 1;
            }
        }
        return crc;
    };

    while (Serial.available() > 0) {
        uint8_t b = Serial.read();
        switch (state) {
            case WAIT_PREAMBLE1:
                if (b == 0xAA) state = WAIT_PREAMBLE2;
                break;
            case WAIT_PREAMBLE2:
                if (b == 0x55) state = READ_LEN_LO;
                else state = WAIT_PREAMBLE1; // Resync
                break;
            case READ_LEN_LO:
                expected_len = b;
                state = READ_LEN_HI;
                break;
            case READ_LEN_HI:
                expected_len |= ((uint16_t)b) << 8;
                if (expected_len > sizeof(payload)) {
                    // Invalid length; resync
                    state = WAIT_PREAMBLE1;
                    expected_len = 0;
                    bytes_read = 0;
                    break;
                }
                bytes_read = 0;
                state = expected_len ? READ_PAYLOAD : READ_CRC_LO;
                break;
            case READ_PAYLOAD:
                payload[bytes_read++] = b;
                if (bytes_read >= expected_len) {
                    state = READ_CRC_LO;
                }
                break;
            case READ_CRC_LO:
                recv_crc = b;
                state = READ_CRC_HI;
                break;
            case READ_CRC_HI: {
                recv_crc |= ((uint16_t)b) << 8;
                uint16_t calc_crc = crc16_ccitt(payload, expected_len);
                if (recv_crc == calc_crc) {
                    // Valid frame: decode payload as nanopb message
                    holoprojector_HoloprojectorCommand cmd = holoprojector_HoloprojectorCommand_init_zero;
                    pb_istream_t stream = pb_istream_from_buffer(payload, expected_len);
                    cmd.led_commands.funcs.decode = &led_command_callback;
                    cmd.led_commands.arg = NULL;
                    if (pb_decode(&stream, holoprojector_HoloprojectorCommand_fields, &cmd)) {
                        // Apply command
                        for (size_t i = 0; i < led_count && i < NEOPIXEL_COUNT; i++) {
                            pixels.setPixelColor(i, pixels.Color(led_cmds[i].r, led_cmds[i].g, led_cmds[i].b));
                        }
                        led_count = 0; // Reset for next message
                        pixels.show();
                        servo1.write((int)cmd.servo1_angle);
                        servo2.write((int)cmd.servo2_angle);
                        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // heartbeat on message
                    } else {
                        Serial.println("Decode failed");
                        // Decode failed; discard frame
                    }
                } else {
                    Serial.println("CRC mismatch");
                    // CRC mismatch; discard and resync
                }
                // Reset for next frame
                state = WAIT_PREAMBLE1;
                expected_len = 0;
                bytes_read = 0;
                recv_crc = 0;
                break;
            }
        }
    }
}
