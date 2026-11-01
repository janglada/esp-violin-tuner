/*
 * Violin tuner for ESP8266 + FC-109 (MAX9812) microphone module
 *
 * Wiring (NodeMCU / Wemos D1 mini, which have a 0-3.3 V divider on A0):
 *   FC-109 VCC -> 3V3
 *   FC-109 GND -> GND
 *   FC-109 OUT -> A0
 * On a bare ESP-12 module A0 only accepts 0-1 V: add a divider first.
 *
 * TFT 2.4" 240x320 (TJCTM24024, ILI9341 controller, SPI):
 *   TFT VCC -> 3V3     TFT GND   -> GND     TFT LED -> 3V3
 *   TFT SCK -> D5 (GPIO14)   TFT SDI/MOSI -> D7 (GPIO13)
 *   TFT CS  -> D8 (GPIO15)   TFT DC/RS    -> D2 (GPIO4)
 *   TFT RESET -> D1 (GPIO5)  (SDO/MISO and the touch pins are not used)
 *
 * Output: Serial Monitor, plus a graphical tuner on the TFT (note, cents meter, strings).
 * Pitch detection: YIN algorithm with parabolic interpolation.
 */

#include <ESP8266WiFi.h>
#include <math.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// ---------- Settings ----------
const int   MIC_PIN     = A0;
const float TARGET_FS   = 10000.0f;  // target sample rate (Hz)
const int   N           = 1024;      // samples per frame (~100 ms)
const int   W           = N / 2;     // YIN integration window
const float MIN_FREQ    = 150.0f;    // a bit below G3 (196 Hz)
const float MAX_FREQ    = 1100.0f;   // well above E5 (659 Hz)
const float YIN_THRESH  = 0.15f;     // lower = stricter
const float YIN_LOOSE   = 0.35f;     // fallback: accept the deepest dip if below this
const float SILENCE_RMS = 8.0f;      // in ADC counts; raise if it reacts to room noise
const int   MAX_TAU_CAP = 128;

const int TFT_CS  = 15;  // D8
const int TFT_DC  = 4;   // D2
const int TFT_RST = 5;   // D1

// ---------- Buffers ----------
int16_t buf[N];
float   yin[MAX_TAU_CAP + 1];
float   lastDip = 1.0f;   // deepest YIN dip of the last frame (diagnostics)

const char* NOTE_NAMES[12] = {"Do", "Do#", "Re", "Re#", "Mi", "Fa",
                              "Fa#", "Sol", "Sol#", "La", "La#", "Si"};

// ---------- TFT user interface (landscape 320x240) ----------
const uint16_t C_BG    = 0x0000;
const uint16_t C_HDR   = 0x10A2;
const uint16_t C_DIM   = 0x4A49;
const uint16_t C_TEXT  = 0xFFFF;
const uint16_t C_TRACK = 0x2124;
const uint16_t C_ACC   = 0x07FF;
const uint16_t ZONE_COLOR[3] = {0x07E0, 0xFE60, 0xF800};  // in tune / close / off

const int   METER_Y   = 126;           // top of the meter area
const float PX_CENT   = 2.8f;          // meter pixels per cent
const int   STR_Y     = 208;
const int   STRINGS   = 4;
const char* STR_NAME[STRINGS] = {"Sol3", "Re4", "La4", "Mi5"};
const float STR_FREQ[STRINGS] = {196.00f, 293.66f, 440.00f, 659.25f};
const int   STR_MIDI[STRINGS] = {55, 62, 69, 76};

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// What is currently drawn, so only the parts that changed are repainted.
int  shownMidi = -1000, shownZone = -1, shownNeedle = -1000, shownString = -2;
int  shownLevel = -1, shownHint = -1;
char shownFreq[16] = "", shownCents[16] = "";

void drawCentered(const char* txt, int cx, int y, int size, uint16_t fg, uint16_t bg) {
  int w = 6 * size * strlen(txt);
  tft.setTextSize(size);
  tft.setTextColor(fg, bg);
  tft.setCursor(cx - w / 2, y);
  tft.print(txt);
}

void drawHeader() {
  tft.fillRect(0, 0, 320, 24, C_HDR);
  tft.fillRect(0, 24, 320, 2, C_ACC);
  tft.setTextSize(2);
  tft.setTextColor(C_TEXT, C_HDR);
  tft.setCursor(8, 5);
  tft.print("VIOLIN TUNER");
  tft.drawRect(209, 7, 104, 10, C_DIM);
}

void drawLevel(float rms) {
  int w = constrain((int)(rms / 150.0f * 100.0f), 0, 100);
  if (abs(w - shownLevel) < 2) return;
  shownLevel = w;
  tft.fillRect(211, 9, w, 6, C_ACC);
  tft.fillRect(211 + w, 9, 100 - w, 6, C_HDR);
}

const int WIN_X = 160 - (int)(5 * PX_CENT);   // in-tune window on the track
const int WIN_W = (int)(10 * PX_CENT);

// Static part of the meter; the needle is drawn and erased separately.
void drawMeterBase() {
  tft.fillRect(0, METER_Y, 320, 60, C_BG);
  // Track with the in-tune window highlighted.
  tft.fillRoundRect(16, METER_Y + 20, 288, 12, 4, C_TRACK);
  tft.fillRect(WIN_X, METER_Y + 20, WIN_W, 12, 0x0320);
  // Ticks every 10 cents, taller at -50 / 0 / +50.
  for (int c = -50; c <= 50; c += 10) {
    int x = 160 + (int)(c * PX_CENT);
    int h = (c % 50 == 0) ? 12 : 6;
    tft.drawFastVLine(x, METER_Y + 36, h, c == 0 ? ZONE_COLOR[0] : C_DIM);
  }
  tft.setTextSize(1);
  tft.setTextColor(C_DIM, C_BG);
  tft.setCursor(16, METER_Y + 48);   tft.print("BAIX (b)");
  tft.setCursor(260, METER_Y + 48);  tft.print("(#) ALT");
}

// Draw the needle in `col`, or erase it (col < 0) restoring the track beneath.
void drawNeedle(int x, int32_t col) {
  if (col >= 0) {
    tft.fillTriangle(x - 8, METER_Y, x + 8, METER_Y, x, METER_Y + 16, col);
    tft.fillRect(x - 1, METER_Y + 16, 3, 20, col);
    return;
  }
  tft.fillRect(x - 8, METER_Y, 17, 20, C_BG);
  tft.fillRect(x - 1, METER_Y + 32, 3, 4, C_BG);
  for (int cx = x - 1; cx <= x + 1; cx++) {
    uint16_t c = (cx >= WIN_X && cx < WIN_X + WIN_W) ? 0x0320 : C_TRACK;
    tft.drawFastVLine(cx, METER_Y + 20, 12, c);
  }
}

void drawStrings(int active, int zone) {
  for (int i = 0; i < STRINGS; i++) {
    int x = 6 + i * 78;
    bool on = (i == active);
    uint16_t col = on ? ZONE_COLOR[zone] : C_DIM;
    tft.fillRoundRect(x, STR_Y, 72, 30, 6, on ? col : C_BG);
    if (!on) tft.drawRoundRect(x, STR_Y, 72, 30, 6, col);
    uint16_t fg = on ? C_BG : C_DIM;
    uint16_t bg = on ? col : C_BG;
    drawCentered(STR_NAME[i], x + 36, STR_Y + 3, 2, fg, bg);
    char fr[12];
    snprintf(fr, sizeof fr, "%.1f Hz", STR_FREQ[i]);
    drawCentered(fr, x + 36, STR_Y + 21, 1, fg, bg);
  }
}

void showIdle() {
  if (shownMidi == -1) return;
  shownMidi = -1; shownZone = -1; shownNeedle = -1000; shownString = -2;
  shownFreq[0] = shownCents[0] = '\0';
  shownHint = -1;
  tft.fillRect(0, 28, 320, 96, C_BG);
  drawCentered("--", 160, 42, 7, C_TRACK, C_BG);
  drawCentered("Toca una corda...", 160, 106, 2, C_DIM, C_BG);
  drawMeterBase();
  tft.fillRect(0, 183, 320, 25, C_BG);
  drawStrings(-1, 0);
}

void showNote(float f, int midi, float cents) {
  int zone = fabsf(cents) < 5.0f ? 0 : (fabsf(cents) < 15.0f ? 1 : 2);
  uint16_t col = ZONE_COLOR[zone];

  // Note name (large) + octave. A colour-only change repaints the same glyphs
  // opaquely instead of clearing the area, which avoids a visible blink.
  if (midi != shownMidi || zone != shownZone) {
    const char* name = NOTE_NAMES[midi % 12];
    char oct[4];
    snprintf(oct, sizeof oct, "%d", midi / 12 - 1);
    int nw = 42 * strlen(name), total = nw + 24;
    int x0 = 160 - total / 2;
    if (midi != shownMidi) tft.fillRect(0, 28, 320, 96, C_BG);
    tft.setTextSize(7);
    tft.setTextColor(col, C_BG);
    tft.setCursor(x0, 42);
    tft.print(name);
    tft.setTextSize(3);
    tft.setCursor(x0 + nw + 4, 78);
    tft.print(oct);
    if (midi != shownMidi) shownFreq[0] = shownCents[0] = '\0';
    shownMidi = midi;
  }

  // Frequency and cents readout.
  char fr[16], ce[16];
  snprintf(fr, sizeof fr, "%7.2f Hz", f);
  snprintf(ce, sizeof ce, "%+6.1f c", cents);
  if (strcmp(fr, shownFreq) || strcmp(ce, shownCents) || zone != shownZone) {
    tft.setTextSize(2);
    tft.setTextColor(C_TEXT, C_BG);
    tft.setCursor(16, 106);  tft.print(fr);
    tft.setTextColor(col, C_BG);
    tft.setCursor(208, 106); tft.print(ce);
    strcpy(shownFreq, fr); strcpy(shownCents, ce);
  }

  // Needle.
  int needle = 160 + (int)(constrain(cents, -50.0f, 50.0f) * PX_CENT);
  if (needle != shownNeedle || zone != shownZone) {
    if (shownNeedle > -1000) drawNeedle(shownNeedle, -1);
    drawNeedle(needle, col);
    shownNeedle = needle;
  }

  // Hint.
  int hint = zone == 0 ? 0 : (cents < 0 ? 1 : 2);
  if (hint != shownHint || zone != shownZone) {
    tft.fillRect(0, 183, 320, 25, C_BG);
    drawCentered(hint == 0 ? "AFINAT!" : (hint == 1 ? "APRETA >>" : "<< AFLUIXA"),
                 160, 184, 3, col, C_BG);
    shownHint = hint;
  }

  // Open string whose note is the one shown, so the chip always agrees with
  // the big letters.
  int best = -1;
  for (int i = 0; i < STRINGS; i++) {
    if (midi == STR_MIDI[i]) best = i;
  }
  if (best != shownString || zone != shownZone) {
    drawStrings(best, zone);
    shownString = best;
  }
  shownZone = zone;
}

float history[3];
int   histCount = 0, histIdx = 0;
bool  rateReported = false;

// Sample N values at a fixed rate; returns the sample rate actually achieved.
float captureFrame() {
  const uint32_t period = (uint32_t)(1000000.0f / TARGET_FS);
  uint32_t next = micros();
  uint32_t tFirst = 0, tLast = 0;

  for (int i = 0; i < N; i++) {
    while ((int32_t)(micros() - next) < 0) { }
    uint32_t t = micros();
    if (i == 0) tFirst = t;
    tLast = t;
    buf[i] = analogRead(MIC_PIN);
    next += period;
  }
  return (N - 1) * 1000000.0f / (float)(tLast - tFirst);
}

// Remove the DC offset (mic is biased at ~VCC/2) and return RMS level.
float removeDcAndRms() {
  int32_t sum = 0;
  for (int i = 0; i < N; i++) sum += buf[i];
  int16_t mean = sum / N;

  // Integer accumulation (no FPU); max 1023^2 * 1024 fits in 32 bits.
  uint32_t sq = 0;
  for (int i = 0; i < N; i++) {
    buf[i] -= mean;
    sq += (int32_t)buf[i] * buf[i];
  }
  return sqrtf((float)sq / N);
}

// YIN pitch detection. Returns frequency in Hz, or -1 if no clear pitch.
float detectPitch(float fs) {
  int minTau = max(2, (int)(fs / MAX_FREQ));
  int maxTau = min(MAX_TAU_CAP - 1, (int)(fs / MIN_FREQ));

  // Cumulative mean normalized difference function, computed lag by lag.
  // The search (first dip below the threshold, then down to its local
  // minimum) runs as we go, so we stop as soon as the minimum is known:
  // high notes need only a fraction of the lags.
  // Inner loop in integers: the ESP8266 has no FPU.
  yin[0] = 1.0f;
  float running = 0;
  int start = -1, tau = -1, deepest = -1;
  float deepestVal = 1.0f;
  for (int t = 1; t <= maxTau; t++) {
    uint32_t d = 0;
    const int16_t* a = buf;
    const int16_t* b = buf + t;
    for (int j = 0; j < W; j++) {
      int32_t diff = a[j] - b[j];
      d += diff * diff;
    }
    running += d;
    yin[t] = running > 0 ? (float)d * t / running : 1.0f;

    if (start >= 0 && yin[t] >= yin[t - 1]) { tau = t - 1; break; }
    if (start < 0 && t >= minTau) {
      if (yin[t] < YIN_THRESH) start = t;
      else if (yin[t] < deepestVal) { deepestVal = yin[t]; deepest = t; }
    }
  }
  if (start < 0) {
    // No dip below the strict threshold (weak fundamental, noisy room):
    // fall back to the deepest dip if it is still convincing.
    lastDip = deepestVal;
    if (deepest < 0 || deepestVal >= YIN_LOOSE) return -1;
    tau = deepest;
  } else {
    lastDip = yin[tau < 0 ? maxTau : tau];
    if (tau < 0) tau = maxTau;   // still descending at the last lag
  }

  // Parabolic interpolation for sub-sample accuracy, only around a true
  // minimum; otherwise the vertex can land far away (even at a negative lag).
  float refined = tau;
  if (tau > 1 && tau < maxTau) {
    float s0 = yin[tau - 1], s1 = yin[tau], s2 = yin[tau + 1];
    float denom = s0 - 2.0f * s1 + s2;
    if (denom > 1e-9f) {
      float off = 0.5f * (s0 - s2) / denom;
      if (fabsf(off) <= 1.0f) refined = tau + off;
    }
  }
  return fs / refined;
}

float median3(float a, float b, float c) {
  return max(min(a, b), min(max(a, b), c));
}

void printNote(float f) {
  float midiF  = 69.0f + 12.0f * log2f(f / 440.0f);
  int   midi   = (int)lroundf(midiF);
  float cents  = (midiF - midi) * 100.0f;

  Serial.printf("%-4s%d  %7.2f Hz  %+6.1f cents\n",
                NOTE_NAMES[midi % 12], midi / 12 - 1, f, cents);
  showNote(f, midi, cents);
}

void setup() {
  // WiFi off: it disturbs ADC timing and readings.
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);

  Serial.begin(9600);
  tft.begin(40000000);
  tft.setRotation(1);
  tft.fillScreen(C_BG);
  tft.setTextWrap(false);
  drawHeader();
  showIdle();
  Serial.println();
  Serial.println("ESP8266 violin tuner - strings: Sol3 196, Re4 294, La4 440, Mi5 659");
}

int silentFrames = 0;

void loop() {
  float fs = captureFrame();
  if (!rateReported) {
    Serial.printf("Measured sample rate: %.0f Hz (target %.0f)\n", fs, TARGET_FS);
    rateReported = true;
  }

  float rms = removeDcAndRms();
  drawLevel(rms);
  if (rms < SILENCE_RMS) {
    histCount = 0;          // reset smoothing between notes
    // Keep the last note on screen briefly while a string rings down.
    if (++silentFrames > 8) showIdle();
    delay(20);
    return;
  }

  float f = detectPitch(fs);
  if (f <= 0) {
    // Loud but unpitched (noise, bow scratch): also counts towards idle,
    // otherwise a stale note would stay on screen indefinitely.
    if (++silentFrames > 8) showIdle();
  } else {
    silentFrames = 0;
    history[histIdx] = f;
    histIdx = (histIdx + 1) % 3;
    if (histCount < 3) histCount++;
    if (histCount == 3) {
      printNote(median3(history[0], history[1], history[2]));
    }
  }
  yield();
}
