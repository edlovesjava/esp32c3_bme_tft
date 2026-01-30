/*
 * BME280 Graphical Display
 * ESP32-C3 Super Mini + ST7735 160x80
 * 
 * Features:
 *   - Color-coded values
 *   - Horizontal bar graphs
 *   - Temperature trend mini-graph
 *   - Clean layout for 160x80 display
 */

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// BME280 I2C pins
#define BME280_I2C_SDA_PIN 8
#define BME280_I2C_SCL_PIN 9
#define BME280_I2C_ADDRESS 0x76
#define SEALEVELPRESSURE_HPA (1013.25)

// TFT pins
#define TFT_SCLK 4
#define TFT_MOSI 6
#define TFT_CS   7
#define TFT_DC   5
#define TFT_RST  10
#define TFT_BL   3

// Objects
Adafruit_BME280 bme;
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Temperature history for graph
#define HIST_SIZE 50
float tempHistory[HIST_SIZE];
int histIndex = 0;
bool histFilled = false;

// Colors
#define COLOR_BG       ST77XX_BLACK
#define COLOR_TEMP     0xFD20  // Orange
#define COLOR_HUMID    0x07FF  // Cyan
#define COLOR_PRESS    0x07E0  // Green
#define COLOR_LABEL    0x8410  // Gray
#define COLOR_GRAPH_BG 0x1082  // Dark blue-gray

// Display regions (160x80)
// Left side: values (0-95)
// Right side: temp graph (100-159)

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nBME280 Graphical Display");

  // Backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  // Init TFT
  tft.initR(INITR_MINI160x80);
  tft.setRotation(1);
  tft.fillScreen(COLOR_BG);
  
  // Splash
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(20, 30);
  tft.print("BME280 Monitor");
  tft.setCursor(35, 45);
  tft.setTextColor(COLOR_LABEL);
  tft.print("Starting...");

  // Init I2C and BME280
  Wire.begin(BME280_I2C_SDA_PIN, BME280_I2C_SCL_PIN);
  
  if (!bme.begin(BME280_I2C_ADDRESS)) {
    tft.fillScreen(COLOR_BG);
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(10, 35);
    tft.print("BME280 ERROR!");
    while (1) delay(100);
  }

  // Init history with current temp
  float initTemp = bme.readTemperature();
  for (int i = 0; i < HIST_SIZE; i++) {
    tempHistory[i] = initTemp;
  }

  delay(500);
  tft.fillScreen(COLOR_BG);
  drawStaticElements();
  
  Serial.println("Ready");
}

void loop() {
  static unsigned long lastUpdate = 0;
  
  if (millis() - lastUpdate >= 2000) {
    lastUpdate = millis();
    updateDisplay();
  }
}

void drawStaticElements() {
  // Labels on left
  tft.setTextColor(COLOR_LABEL);
  tft.setTextSize(1);
  tft.setCursor(2, 2);
  tft.print("T");
  tft.setCursor(2, 28);
  tft.print("H");
  tft.setCursor(2, 54);
  tft.print("P");
  
  // Graph border
  tft.drawRect(99, 0, 61, 80, COLOR_LABEL);
  
  // Graph title
  tft.setTextColor(COLOR_LABEL);
  tft.setCursor(110, 2);
  tft.print("TREND");
}

void updateDisplay() {
  // Read all values once
  float temp = bme.readTemperature();
  float humid = bme.readHumidity();
  float press = bme.readPressure() / 100.0F;
  
  // Update history
  tempHistory[histIndex] = temp;
  histIndex = (histIndex + 1) % HIST_SIZE;
  if (histIndex == 0) histFilled = true;
  
  // Draw temperature section (y: 0-25)
  drawValueWithBar(12, 0, temp, -10, 50, COLOR_TEMP, "C", 1);
  
  // Draw humidity section (y: 26-51)
  drawValueWithBar(12, 26, humid, 0, 100, COLOR_HUMID, "%", 0);
  
  // Draw pressure section (y: 52-77)
  drawPressure(12, 52, press);
  
  // Draw temperature graph
  drawTempGraph();
  
  // Serial output
  Serial.printf("T:%.1fC H:%.0f%% P:%.0fhPa\n", temp, humid, press);
}

void drawValueWithBar(int x, int y, float value, float minVal, float maxVal, 
                      uint16_t color, const char* unit, int decimals) {
  // Clear area
  tft.fillRect(x, y, 85, 24, COLOR_BG);
  
  // Value text
  tft.setTextColor(color);
  tft.setTextSize(2);
  tft.setCursor(x, y);
  
  if (decimals == 1) {
    tft.print(value, 1);
  } else {
    tft.print((int)value);
  }
  
  // Unit
  tft.setTextSize(1);
  tft.print(unit);
  
  // Bar graph
  int barY = y + 17;
  int barWidth = 80;
  int barHeight = 5;
  
  // Background bar
  tft.fillRect(x, barY, barWidth, barHeight, COLOR_GRAPH_BG);
  
  // Value bar
  float pct = constrain((value - minVal) / (maxVal - minVal), 0, 1);
  int fillWidth = (int)(pct * barWidth);
  tft.fillRect(x, barY, fillWidth, barHeight, color);
}

void drawPressure(int x, int y, float press) {
  // Clear area
  tft.fillRect(x, y, 85, 24, COLOR_BG);
  
  // Pressure value
  tft.setTextColor(COLOR_PRESS);
  tft.setTextSize(2);
  tft.setCursor(x, y);
  tft.print((int)press);
  
  // Unit
  tft.setTextSize(1);
  tft.setCursor(x + 48, y + 4);
  tft.print("hPa");
  
  // Trend indicator based on graph data
  int trend = getPressureTrend();
  tft.setCursor(x + 70, y + 4);
  if (trend > 0) {
    tft.setTextColor(COLOR_PRESS);
    tft.print("^");  // Rising
  } else if (trend < 0) {
    tft.setTextColor(COLOR_TEMP);
    tft.print("v");  // Falling
  } else {
    tft.setTextColor(COLOR_LABEL);
    tft.print("-");  // Stable
  }
  
  // Mini bar showing relative pressure (950-1050 range)
  int barY = y + 17;
  tft.fillRect(x, barY, 80, 5, COLOR_GRAPH_BG);
  float pct = constrain((press - 950) / 100.0, 0, 1);
  tft.fillRect(x, barY, (int)(pct * 80), 5, COLOR_PRESS);
}

int getPressureTrend() {
  // Simple trend: compare oldest vs newest in history
  if (!histFilled && histIndex < 5) return 0;
  
  int oldIdx = histFilled ? (histIndex + 1) % HIST_SIZE : 0;
  int newIdx = (histIndex - 1 + HIST_SIZE) % HIST_SIZE;
  
  float diff = tempHistory[newIdx] - tempHistory[oldIdx];
  if (diff > 0.5) return 1;
  if (diff < -0.5) return -1;
  return 0;
}

void drawTempGraph() {
  // Graph area: x=100-158, y=12-78
  int gx = 101;
  int gy = 12;
  int gw = 57;
  int gh = 65;
  
  // Clear graph area
  tft.fillRect(gx, gy, gw, gh, COLOR_GRAPH_BG);
  
  // Find min/max for scaling
  float minT = tempHistory[0];
  float maxT = tempHistory[0];
  int count = histFilled ? HIST_SIZE : histIndex;
  
  for (int i = 0; i < count; i++) {
    if (tempHistory[i] < minT) minT = tempHistory[i];
    if (tempHistory[i] > maxT) maxT = tempHistory[i];
  }
  
  // Ensure minimum range
  if (maxT - minT < 2) {
    float mid = (maxT + minT) / 2;
    minT = mid - 1;
    maxT = mid + 1;
  }
  
  // Draw center line
  int centerY = gy + gh / 2;
  for (int x = gx; x < gx + gw; x += 3) {
    tft.drawPixel(x, centerY, COLOR_LABEL);
  }
  
  // Draw temperature line
  if (count < 2) return;
  
  for (int i = 1; i < count; i++) {
    int idx1 = histFilled ? (histIndex + i - 1) % HIST_SIZE : i - 1;
    int idx2 = histFilled ? (histIndex + i) % HIST_SIZE : i;
    
    // Map to graph coordinates
    int x1 = gx + map(i - 1, 0, HIST_SIZE - 1, 0, gw - 1);
    int x2 = gx + map(i, 0, HIST_SIZE - 1, 0, gw - 1);
    
    int y1 = gy + gh - 1 - (int)((tempHistory[idx1] - minT) / (maxT - minT) * (gh - 1));
    int y2 = gy + gh - 1 - (int)((tempHistory[idx2] - minT) / (maxT - minT) * (gh - 1));
    
    y1 = constrain(y1, gy, gy + gh - 1);
    y2 = constrain(y2, gy, gy + gh - 1);
    
    // Color based on temperature
    uint16_t lineColor = getTempColor(tempHistory[idx2], minT, maxT);
    tft.drawLine(x1, y1, x2, y2, lineColor);
  }
  
  // Current value dot
  int lastIdx = (histIndex - 1 + HIST_SIZE) % HIST_SIZE;
  int dotX = gx + gw - 2;
  int dotY = gy + gh - 1 - (int)((tempHistory[lastIdx] - minT) / (maxT - minT) * (gh - 1));
  dotY = constrain(dotY, gy + 2, gy + gh - 3);
  tft.fillCircle(dotX, dotY, 2, ST77XX_WHITE);
}

uint16_t getTempColor(float temp, float minT, float maxT) {
  float norm = (temp - minT) / (maxT - minT);
  norm = constrain(norm, 0, 1);
  
  // Blue (cold) -> Cyan -> Green -> Yellow -> Red (hot)
  uint8_t r, g, b;
  
  if (norm < 0.25) {
    // Blue to Cyan
    float t = norm * 4;
    r = 0; g = (uint8_t)(t * 255); b = 255;
  } else if (norm < 0.5) {
    // Cyan to Green
    float t = (norm - 0.25) * 4;
    r = 0; g = 255; b = (uint8_t)((1 - t) * 255);
  } else if (norm < 0.75) {
    // Green to Yellow
    float t = (norm - 0.5) * 4;
    r = (uint8_t)(t * 255); g = 255; b = 0;
  } else {
    // Yellow to Red
    float t = (norm - 0.75) * 4;
    r = 255; g = (uint8_t)((1 - t) * 255); b = 0;
  }
  
  return tft.color565(r, g, b);
}
