/*
  tcs_column_test.ino -- Rainbow Forest v2, single-column bring-up

  Hardware under test:
    ESP32-DevKitC (classic ESP32-WROOM-32)
    -> DFRobot DFR0576 I2C mux (TCA9548A, address 0x70)
       -> 7x DFRobot SEN0212 (TCS34725, address 0x29), one per mux port 0..6
    7 individual LED-enable lines, one GPIO per sensor.

  What this does: proves the wiring. No WiFi, no OSC, no color classification yet.
  Open the serial monitor at 115200 baud, line ending "Newline", and type "?".

  Arduino IDE setup:
    Board:           ESP32 Dev Module   (arduino-esp32 core 3.x)
    No extra libraries needed: Wire only. The TCS34725 is driven directly
    through its registers so the sensor stays behind a few small functions
    (tcsInit / tcsReadRaw) and can be swapped if the chip is discontinued.

  Row r (note C..B) lives on mux port r. Port 7 is unused.
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ------------------------------------------------------------------ pins
// Classic ESP32-DevKitC (ESP32-WROOM-32).
// Avoided on purpose: 6-11 (flash), 34-39 (input only), 0/2/5/12/15 (strapping), 1/3 (USB serial).
#define PIN_SDA        21
#define PIN_SCL        22
#define PIN_MUX_RESET  23     // TCA9548A /RESET, active low. Set to -1 if not wired.

// One GPIO per sensor LED-enable line, row 0..6. Use -1 for a row with no LED wire.
static const int8_t LED_PINS[7] = {13, 14, 25, 26, 27, 32, 33};

// ------------------------------------------------------------------ constants
#define NUM_ROWS       7
#define MUX_ADDR       0x70
#define TCS_ADDR       0x29
#define I2C_HZ         100000UL   // star-wired cables: stay at 100 kHz, not 400

#define TCS_CMD        0x80
#define TCS_CMD_AUTO   0x20       // auto-increment on multi-byte reads
#define REG_ENABLE     0x00
#define REG_ATIME      0x01
#define REG_CONTROL    0x0F
#define REG_ID         0x12
#define REG_CDATAL     0x14       // C, R, G, B as little-endian 16-bit, in that order
#define ENABLE_PON     0x01
#define ENABLE_AEN     0x02
#define ID_TCS34725    0x44
#define ID_TCS34727    0x4D

// ------------------------------------------------------------------ state
static uint8_t g_atime = 0xEB;    // 21 cycles * 2.4 ms = ~50 ms integration
static uint8_t g_gain  = 0x00;    // 0=1x 1=4x 2=16x 3=60x
static bool    g_present[8];
static uint8_t g_chipId[8];
static bool    g_ledOn[NUM_ROWS];
static bool    g_continuous = false;
static uint32_t g_lastRead  = 0;

struct Sample {
  uint16_t c, r, g, b;
  bool ok;
};

// ------------------------------------------------------------------ mux
// ch 0..7 selects one port; ch < 0 disconnects all ports.
static bool muxSelect(int8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(ch < 0 ? (uint8_t)0x00 : (uint8_t)(1u << ch));
  return Wire.endTransmission() == 0;
}

static void muxReset() {
  if (PIN_MUX_RESET < 0) return;
  pinMode(PIN_MUX_RESET, OUTPUT);
  digitalWrite(PIN_MUX_RESET, LOW);
  delay(2);
  digitalWrite(PIN_MUX_RESET, HIGH);
  delay(2);
}

// ------------------------------------------------------------------ TCS34725
static bool tcsWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(TCS_ADDR);
  Wire.write((uint8_t)(TCS_CMD | reg));
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool tcsRead(uint8_t reg, uint8_t *buf, uint8_t n) {
  uint8_t cmd = TCS_CMD | (n > 1 ? TCS_CMD_AUTO : 0) | reg;
  Wire.beginTransmission(TCS_ADDR);
  Wire.write(cmd);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((uint8_t)TCS_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// Mux port must already be selected.
static bool tcsInit() {
  uint8_t id = 0;
  if (!tcsRead(REG_ID, &id, 1)) return false;
  if (id != ID_TCS34725 && id != ID_TCS34727) return false;
  if (!tcsWrite8(REG_ATIME, g_atime)) return false;
  if (!tcsWrite8(REG_CONTROL, g_gain)) return false;
  if (!tcsWrite8(REG_ENABLE, ENABLE_PON)) return false;
  delay(3);                                          // oscillator start-up
  return tcsWrite8(REG_ENABLE, ENABLE_PON | ENABLE_AEN);
}

// Mux port must already be selected. The sensor free-runs after tcsInit(),
// so this returns the most recently completed integration.
static bool tcsReadRaw(Sample &s) {
  uint8_t b[8];
  s.ok = tcsRead(REG_CDATAL, b, 8);
  if (!s.ok) return false;
  s.c = (uint16_t)(b[0] | (b[1] << 8));
  s.r = (uint16_t)(b[2] | (b[3] << 8));
  s.g = (uint16_t)(b[4] | (b[5] << 8));
  s.b = (uint16_t)(b[6] | (b[7] << 8));
  return true;
}

static float integrationMs() { return (256 - g_atime) * 2.4f; }

static uint32_t maxCount() {
  uint32_t m = (uint32_t)(256 - g_atime) * 1024UL;
  return m > 65535UL ? 65535UL : m;
}

static const char *gainName() {
  switch (g_gain & 0x03) {
    case 0: return "1x";
    case 1: return "4x";
    case 2: return "16x";
    default: return "60x";
  }
}

// Hue in degrees from raw R,G,B (same formula family as the original firmware's rgbToHsv).
// Returns NAN when there is no chroma.
static float hueDeg(float r, float g, float b) {
  float mx = fmaxf(r, fmaxf(g, b));
  float mn = fminf(r, fminf(g, b));
  float d = mx - mn;
  if (d < 1e-6f || mx <= 0.0f) return NAN;
  float h;
  if (mx == r)      h = (g - b) / d;
  else if (mx == g) h = 2.0f + (b - r) / d;
  else              h = 4.0f + (r - g) / d;
  h *= 60.0f;
  if (h < 0.0f) h += 360.0f;
  return h;
}

// ------------------------------------------------------------------ LEDs
// SEN0212 LED pad is active-high and ON when left unconnected, so:
//   on  -> release the pin (INPUT, floating)
//   off -> drive it LOW
// Never drives the pad high, so nothing here can fight the board's own pull-up.
static void ledSet(uint8_t row, bool on) {
  if (row >= NUM_ROWS || LED_PINS[row] < 0) return;
  if (on) {
    pinMode(LED_PINS[row], INPUT);
  } else {
    pinMode(LED_PINS[row], OUTPUT);
    digitalWrite(LED_PINS[row], LOW);
  }
  g_ledOn[row] = on;
}

// ------------------------------------------------------------------ actions
static void scanBus() {
  Serial.println("--- scan ---");
  muxSelect(-1);

  bool muxFound = false;
  for (uint8_t a = 0x70; a <= 0x77; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("mux acknowledged at 0x%02X%s\n", a, a == MUX_ADDR ? "" : "  <-- expected 0x70, check the address switches");
      if (a == MUX_ADDR) muxFound = true;
    }
  }
  if (!muxFound) {
    Serial.printf("NO MUX at 0x70. Check SDA/SCL (GPIO %d/%d), mux power, and that RESET is not held low.\n", PIN_SDA, PIN_SCL);
  }

  for (int8_t ch = 0; ch < 8; ch++) {
    g_present[ch] = false;
    g_chipId[ch] = 0;
    if (!muxFound || !muxSelect(ch)) { Serial.printf("port %d: mux select failed\n", ch); continue; }
    uint8_t id = 0;
    bool got = tcsRead(REG_ID, &id, 1);
    g_chipId[ch] = id;
    if (got && (id == ID_TCS34725 || id == ID_TCS34727)) {
      g_present[ch] = true;
      Serial.printf("port %d: TCS3472x  ID=0x%02X  OK\n", ch, id);
    } else if (got) {
      Serial.printf("port %d: device answered at 0x29 but ID=0x%02X is not a TCS3472x\n", ch, id);
    } else {
      Serial.printf("port %d: nothing at 0x29%s\n", ch, ch < NUM_ROWS ? "" : "  (unused)");
    }
  }
  muxSelect(-1);
}

static void initAll() {
  int n = 0;
  for (int8_t ch = 0; ch < 8; ch++) {
    if (!g_present[ch]) continue;
    if (muxSelect(ch) && tcsInit()) n++;
    else { g_present[ch] = false; Serial.printf("port %d: init failed\n", ch); }
  }
  muxSelect(-1);
  delay((uint32_t)(integrationMs() * 1.5f) + 5);   // let the first integration finish
  Serial.printf("%d sensor(s) running: gain %s, integration %.1f ms (counts saturate at %lu)\n",
                n, gainName(), integrationMs(), (unsigned long)maxCount());
}

static void applySettings() {
  for (int8_t ch = 0; ch < 8; ch++) {
    if (!g_present[ch]) continue;
    if (muxSelect(ch)) { tcsWrite8(REG_ATIME, g_atime); tcsWrite8(REG_CONTROL, g_gain); }
  }
  muxSelect(-1);
  delay((uint32_t)(integrationMs() * 2.0f) + 5);   // old integration must flush
  Serial.printf("gain %s, integration %.1f ms (counts saturate at %lu)\n",
                gainName(), integrationMs(), (unsigned long)maxCount());
}

static void readAll() {
  uint32_t t0 = millis();
  Sample s[NUM_ROWS];
  for (uint8_t row = 0; row < NUM_ROWS; row++) {
    s[row].ok = false;
    if (g_present[row] && muxSelect(row)) tcsReadRaw(s[row]);
  }
  muxSelect(-1);
  uint32_t dt = millis() - t0;

  Serial.printf("--- read all (%lu ms for %d ports) ---\n", (unsigned long)dt, NUM_ROWS);
  uint32_t sat = maxCount() - 8;
  for (uint8_t row = 0; row < NUM_ROWS; row++) {
    if (!s[row].ok) {
      Serial.printf("row %u | %s\n", row, g_present[row] ? "READ FAILED" : "absent");
      continue;
    }
    float c = s[row].c ? (float)s[row].c : 1.0f;
    float h = hueDeg(s[row].r, s[row].g, s[row].b);
    Serial.printf("row %u | C %5u R %5u G %5u B %5u | r/c %.2f g/c %.2f b/c %.2f | hue ",
                  row, s[row].c, s[row].r, s[row].g, s[row].b,
                  s[row].r / c, s[row].g / c, s[row].b / c);
    if (isnan(h)) Serial.print("  -- "); else Serial.printf("%5.1f", h);
    Serial.printf(" | LED %s%s\n", g_ledOn[row] ? "on " : "off", s[row].c >= sat ? " | SATURATED" : "");
  }
}

static void printHelp() {
  Serial.println(
    "commands (end with Enter):\n"
    "  s            scan mux + sensors\n"
    "  r            read all 7 once\n"
    "  c            toggle continuous read (4 Hz)\n"
    "  l <n|a> <0|1>  LED row n (0-6) or all, 0=off 1=on     e.g. 'l 3 0', 'l a 1'\n"
    "  g <0-3>      gain 1x / 4x / 16x / 60x\n"
    "  t <ms>       integration time, 2.4 - 614 ms\n"
    "  ?            this help");
}

static void handleLine(const char *line) {
  char cmd = line[0];
  switch (cmd) {
    case 's': scanBus(); initAll(); break;
    case 'r': readAll(); break;
    case 'c':
      g_continuous = !g_continuous;
      Serial.printf("continuous %s\n", g_continuous ? "on" : "off");
      break;
    case 'l': {
      char which = 0; int v = 0;
      if (sscanf(line + 1, " %c %d", &which, &v) != 2 || (v != 0 && v != 1)) { Serial.println("usage: l <0-6|a> <0|1>"); break; }
      if (which == 'a') { for (uint8_t i = 0; i < NUM_ROWS; i++) ledSet(i, v == 1); }
      else if (which >= '0' && which <= '6') ledSet(which - '0', v == 1);
      else { Serial.println("row must be 0-6 or a"); break; }
      Serial.println("ok");
      break;
    }
    case 'g': {
      int v = -1;
      if (sscanf(line + 1, " %d", &v) != 1 || v < 0 || v > 3) { Serial.println("usage: g <0-3>"); break; }
      g_gain = (uint8_t)v;
      applySettings();
      break;
    }
    case 't': {
      float ms = 0;
      if (sscanf(line + 1, " %f", &ms) != 1 || ms < 2.4f || ms > 614.4f) { Serial.println("usage: t <2.4-614 ms>"); break; }
      int cycles = (int)(ms / 2.4f + 0.5f);
      if (cycles < 1) cycles = 1;
      if (cycles > 256) cycles = 256;
      g_atime = (uint8_t)(256 - cycles);
      applySettings();
      break;
    }
    case '?': printHelp(); break;
    default:  Serial.println("unknown command, '?' for help"); break;
  }
}

// ------------------------------------------------------------------ Arduino entry points
void setup() {
  Serial.begin(115200);
  delay(800);   // let the USB-serial bridge settle so the banner is not lost

  for (uint8_t i = 0; i < NUM_ROWS; i++) ledSet(i, true);   // default state: all LEDs on

  muxReset();
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(50);   // ms; a dead port must not stall the loop

  Serial.println("\nRainbow Forest v2 -- column bring-up");
  scanBus();
  initAll();
  printHelp();
}

void loop() {
  static char line[32];
  static uint8_t len = 0;
  while (Serial.available()) {
    int ch = Serial.read();
    if (ch < 0 || ch == '\r') continue;
    if (ch == '\n') {
      line[len] = 0;
      if (len) handleLine(line);
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = (char)ch;
    }
  }

  if (g_continuous && millis() - g_lastRead >= 250) {
    g_lastRead = millis();
    readAll();
  }
}
