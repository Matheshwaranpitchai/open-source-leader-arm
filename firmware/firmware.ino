#include <Wire.h>

#define MUX_ADDR     0x70
#define AS5600_ADDR  0x36
#define N_CH         6
#define SDA_PIN      18
#define SCL_PIN      19
#define SAMPLE_HZ    100

#define REG_STATUS    0x0B
#define REG_RAW_ANGLE 0x0C

bool muxSelect(uint8_t ch) {
  if (ch >= 8) return false;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  return Wire.endTransmission() == 0;
}

void muxDisableAll() {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

int16_t readRawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(REG_RAW_ANGLE);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(AS5600_ADDR, 2) != 2) return -1;
  uint8_t hi = Wire.read(), lo = Wire.read();
  return ((hi << 8) | lo) & 0x0FFF;
}

uint8_t readStatus() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(REG_STATUS);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(AS5600_ADDR, 1) != 1) return 0;
  return Wire.read();
}

void diagnose() {
  Serial.println("# channel scan");
  for (uint8_t ch = 0; ch < N_CH; ch++) {
    if (!muxSelect(ch)) {
      Serial.printf("# ch%u: mux not responding at 0x%02X\n", ch, MUX_ADDR);
      continue;
    }
    Wire.beginTransmission(AS5600_ADDR);
    if (Wire.endTransmission() != 0) {
      Serial.printf("# ch%u: no device at 0x36 -- check SDn/SCn and power\n", ch);
      continue;
    }
    uint8_t st = readStatus();
    const char *magnet = (st & 0x20) ? "ok"
                       : (st & 0x10) ? "too weak / too far"
                       : (st & 0x08) ? "too strong / too close"
                                     : "not detected";
    Serial.printf("# ch%u: AS5600 found, magnet %s, raw %d\n",
                  ch, magnet, readRawAngle());
  }
  muxDisableAll();
  Serial.println("# scan done");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(300);
  diagnose();
}

void loop() {
  static uint32_t next = 0;
  const uint32_t period = 1000 / SAMPLE_HZ;
  if (millis() < next) return;
  next = millis() + period;

  int16_t angle[N_CH];
  for (uint8_t ch = 0; ch < N_CH; ch++) {
    angle[ch] = muxSelect(ch) ? readRawAngle() : -1;
  }

  for (uint8_t ch = 0; ch < N_CH; ch++) {
    Serial.print(angle[ch]);
    Serial.print(ch == N_CH - 1 ? '\n' : ',');
  }
}
