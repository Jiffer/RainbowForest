#include <Wire.h>
#include <FastLED.h>
#include "src/components/sensor/sensor.h"

#define COLUMN_LEN 7

#define DEBUG_PORT Serial
#define DEBUG_BAUD (115200)

#define DATA_PIN 18


#define PRINT_INTERVAL_MS 1000  // change this to vary print rate

SensorString sensors(COLUMN_LEN, DATA_PIN, &Wire);

struct reading_t {
  unsigned int r, g, b;
  bool valid;
};

reading_t readings[COLUMN_LEN];

void readNode(SensorNode* node, size_t idx, void* args) {
  node->read();
  readings[idx].r = node->getRed();
  readings[idx].g = node->getGreen();
  readings[idx].b = node->getBlue();
  readings[idx].valid = (readings[idx].r != 65535);
}

void setup() {
  DEBUG_PORT.begin(DEBUG_BAUD);
  delay(1000);
  Wire.begin(23, 22);
  FastLED.addLeds<WS2811, DATA_PIN, RGB>(sensors.getControl(), sensors.getNumControlElements());
  SensorStatus_e ret = sensors.begin();
  DEBUG_PORT.printf("sensors.begin() returned: %d\n", ret);
}

void loop() {
  sensors.forEach(readNode);

  DEBUG_PORT.print("[ ");
  for (size_t i = 0; i < COLUMN_LEN; i++) {
    if (readings[i].valid) {
      DEBUG_PORT.printf("%d: R=%5d G=%5d B=%5d | ", i, readings[i].r, readings[i].g, readings[i].b);
    } else {
      DEBUG_PORT.printf("%d: NO SENSOR       | ", i);
    }
  }
  DEBUG_PORT.println("]");

  delay(PRINT_INTERVAL_MS);
}