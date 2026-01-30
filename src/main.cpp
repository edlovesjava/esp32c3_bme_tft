// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ST7735.h>

#include "Config.h"
#include "core/SensorRegistry.h"
#include "display/DisplayManager.h"
#include "sensors/BME280Sensor.h"

// Hardware
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Core system
SensorRegistry registry;
DisplayManager display;

// Sensors
BME280Sensor bme280;

// Timing
unsigned long lastUpdate = 0;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\nModular Sensor System");

    // Backlight
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    // Init TFT
    tft.initR(INITR_MINI160x80);
    tft.setRotation(DISPLAY_ROTATION);
    tft.fillScreen(Colors::BACKGROUND);

    // Splash
    tft.setTextColor(0xFFFF);
    tft.setCursor(20, 30);
    tft.print("Sensor Monitor");

    // Init I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_FREQUENCY);

    // Register sensors
    registry.registerSensor(&bme280);

    // Initialize sensors
    if (!registry.initializeAll()) {
        tft.fillScreen(Colors::BACKGROUND);
        tft.setTextColor(0xF800);
        tft.setCursor(10, 35);
        tft.print("Sensor Error!");
        while (1) delay(1000);
    }

    // Initialize display
    display.begin(&tft);
    display.setLayout(LayoutMode::VALUES_WITH_GRAPH);

    Serial.printf("Sensors: %d, Values: %d\n",
                  registry.getSensorCount(),
                  registry.getTotalValueCount());

    delay(500);
    display.invalidate();
}

void loop() {
    unsigned long now = millis();

    if (now - lastUpdate >= SENSOR_POLL_INTERVAL_MS) {
        lastUpdate = now;

        // Poll sensors
        registry.pollAll();

        // Collect readings
        uint8_t count = registry.getTotalValueCount();
        SensorReading readings[MAX_DISPLAY_VALUES];
        const SensorValueDescriptor* descriptors[MAX_DISPLAY_VALUES];

        for (uint8_t i = 0; i < count; i++) {
            readings[i] = registry.getReading(i);
            descriptors[i] = registry.getDescriptor(i);
        }

        // Update display
        display.updateHistory(readings, count);
        display.render(readings, descriptors, count);

        // Debug output
        for (uint8_t i = 0; i < count; i++) {
            if (readings[i].valid && descriptors[i]) {
                Serial.printf("%s: %.1f %s\n",
                              descriptors[i]->name,
                              readings[i].value,
                              descriptors[i]->unit);
            }
        }
    }
}
