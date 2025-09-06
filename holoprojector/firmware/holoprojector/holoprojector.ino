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
    static uint8_t size = 0;
    static uint8_t buffer[512];
    static size_t buffer_pos = 0;

    // Receive a uint32_t size prefix then read that many bytes
    if (size == 0 && Serial.available() >= sizeof(uint32_t)) {
        Serial.readBytes(reinterpret_cast<char*>(&size), sizeof(uint32_t));
        buffer_pos = 0;
        Serial.print("Expecting message of size: "); Serial.println(size);
    }
    if (size > 0 && Serial.available() > 0) {
        buffer[buffer_pos++] = Serial.read();
        if (buffer_pos >= size) {
            // We have a full message
            holoprojector_HoloprojectorCommand cmd = holoprojector_HoloprojectorCommand_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_pos);
            cmd.led_commands.funcs.decode = &led_command_callback;
            cmd.led_commands.arg = NULL;
          if (pb_decode(&stream, holoprojector_HoloprojectorCommand_fields, &cmd))
          {
              Serial.println("Decoded proto!");
              Serial.print("Servo1: "); Serial.println(cmd.servo1_angle);
              Serial.print("Servo2: "); Serial.println(cmd.servo2_angle);
              for (size_t i = 0; i < led_count && i < NEOPIXEL_COUNT; i++)
              {
                  pixels.setPixelColor(i, pixels.Color(led_cmds[i].r, led_cmds[i].g, led_cmds[i].b));
                  Serial.print("LED "); Serial.print(i);
                  Serial.print(": R="); Serial.print(led_cmds[i].r);
                  Serial.print(" G="); Serial.print(led_cmds[i].g);
                  Serial.print(" B="); Serial.println(led_cmds[i].b);
              }
              led_count = 0; // Reset for next message
              pixels.show();
              servo1.write(cmd.servo1_angle);
              servo2.write(cmd.servo2_angle);
            } else {
                Serial.print("Decode failed: ");
                Serial.println(PB_GET_ERROR(&stream));
            }
            size = 0; // Reset for next message
            buffer_pos = 0;
        }
    }


    // // Read bytes into buffer
    // while (Serial.available() > 0 && buffer_pos < sizeof(buffer)) {
    //     buffer[buffer_pos++] = Serial.read();
    // }

    // // Example: Assume message ends when buffer is full
    // if (buffer_pos == sizeof(buffer)) {
    //     HoloprojectorCommand cmd = HoloprojectorCommand_init_zero;
    //     pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_pos);

    //     if (pb_decode(&stream, HoloprojectorCommand_fields, &cmd)) {
    //         // Successfully decoded
    //         Serial.println("Decoded proto!");
    //         Serial.print("Servo1: "); Serial.println(cmd.servo1_angle);
    //         Serial.print("Servo2: "); Serial.println(cmd.servo2_angle);
    //     } else {
    //         Serial.print("Decode failed: ");
    //         Serial.println(PB_GET_ERROR(&stream));
    //     }
    //     buffer_pos = 0; // Reset for next message
    // }
}