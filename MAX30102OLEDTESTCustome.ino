// MAX30102 + 128x32 OLED + Buzzer + Line Wave
// Guardian Grip: robust contact detection, invalid-reading stop, summary stats,
// JSON events for Smart Home, and more stable HR calculation.

#include <Arduino.h>
#include <Wire.h>
#include "max30102_driver.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- I/O + display ----------
const int SDA_PIN = 8;
const int SCL_PIN = 9;
TwoWire CustomWire(SDA_PIN, SCL_PIN);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &CustomWire, -1);

// ---------- Sensor & state ----------
MAX30102_Driver sensor;
bool ok = false;

// ---------- Sampling & buffers ----------
#define SAMPLE_RATE 100     // effective output sample rate (Hz) - set to 400 if you use 400Hz w/o kernel averaging
#define BUF_SECONDS 4       // seconds of samples retained for compute (increase for higher stability)
#define BUF_SIZE (SAMPLE_RATE * BUF_SECONDS)

uint32_t redBuf[BUF_SIZE];
uint32_t irBuf[BUF_SIZE];
int bufIndex = 0;
uint32_t totalSamples = 0;
bool dataStarted = false;

int spo2 = 0, hr = 0;
float spo2Smoothed = NAN;
unsigned long lastSampleMillis = 0;

// global AC buffer for IR used in HR detection
static float g_acIR[BUF_SIZE];

// ---------- waveform ----------
#define WAVE_SAMPLES 80
uint16_t wf[WAVE_SAMPLES];
uint8_t wfIndex = 0;
uint8_t wfFilled = 0;

// ---------- alarm / beeper ----------
const int BEEPER_PIN = 15;
const bool BEEPER_ENABLED = true;
const int SPO2_WARNING_THRESHOLD = 90;
bool alarmActive = false;
unsigned long lastBeepTime = 0;
const unsigned long BEEP_COOLDOWN_MS = 5000;

// ---------- HR freshness ----------
unsigned long lastHrUpdateMs = 0;
const unsigned long HR_STALE_MS = 2000;  // HR stales after 2 seconds

// ---------- debug & re-init ----------
unsigned long lastDiag = 0;
const unsigned long NO_DATA_TIMEOUT_MS = 4000;
const unsigned long RECOVERY_INTERVAL_MS = 10000;
unsigned long lastRecoveryAttempt = 0;

// ---------- computation thresholds ----------
const float MIN_AC_RMS = 8.0f;
const float MIN_DC_LEVEL = 200.0f;
const float SPO2_A = 110.0f;
const float SPO2_B = 25.0f;

// ---------- contact & recording states ----------
bool contactOK = false;          // current contact quality
bool recording = false;          // whether we are currently recording (placing finger started)
bool invalidReadingActive = false;
unsigned long contactStartMs = 0;
const unsigned long CONTACT_STABLE_MS = 3000; // require 3s stable contact to start recording

// ---------- invalid reading hysteresis ----------
int invalidCount = 0;
const int INVALID_COUNT_THRESHOLD = 3;
const int SPO2_VALID_LOW = 70;  // if below this for N times -> invalid
const int SPO2_VALID_HIGH = 100;

// ---------- "stable show" variables ----------
const float MIN_AC_RATIO = 0.03f; // require AC/DC ≥ 3% to consider real pulsatility
bool showReadings = false;
int validReadingCount = 0;
const int VALID_READINGS_TO_SHOW = 3; // need 3 consecutive valid windows before showing/recording

// ---------- page UI ----------
enum Page { PAGE_SPO2 = 0, PAGE_HR = 1, PAGE_WAVE = 2, PAGE_STATS = 3 };
Page currentPage = PAGE_SPO2;
Page nextPage = PAGE_HR;
unsigned long lastPageSwitchMs = 0;
const unsigned long PAGE_INTERVAL_MS = 3000; // rotate every 3 seconds

bool transitionActive = false;
unsigned long transitionStartMs = 0;
const unsigned long TRANSITION_MS = 350;  // transition duration (ms)
const unsigned long FRAME_MS = 40;

// ---------- results history (small) ----------
#define RESULT_HISTORY 32
int resSpo2[RESULT_HISTORY];
int resHr[RESULT_HISTORY];
int resIdx = 0;
bool resFilled = false;

// ---------- session stats ----------
int sessionMinSpo2 = 100, sessionMaxSpo2 = 0, sessionSumSpo2 = 0;
int sessionMinHr = 999, sessionMaxHr = 0, sessionSumHr = 0;
int sessionCount = 0;

// ---------- alarm hysteresis ----------
const unsigned long ALARM_DELAY_MS = 5000; // require 5s sustained low
unsigned long lowSinceMs = 0;

// ---------- HR thresholds for status ----------
const int HR_WARNING_HIGH = 120; // example
const int HR_WARNING_LOW = 40;

// ---------- smoothing & HR helper ----------
int hrPrevSmoothed = 0;
const float HR_ALPHA = 0.25f; // EWMA smoothing

// ---------- forward declarations ----------
void computeSpo2AndHr();
float computeRMSFromBuffer(const uint32_t *buf, int len, uint32_t avg);
void startTransition(Page to);
void drawTransitionFrame();
void drawPageFrame(Page p, int16_t xOffset);
void updateDisplay();
void checkSpO2AndAlert(); // uses immediate smoothed values + sustained low timer
void beepPattern(int count, int durationMs, int freqHz);
bool checkContactQuality(uint32_t rAvg, uint32_t iAvg, float rRMS, float iRMS);
void startRecordingSession();
void stopRecordingSession(const char *reason);
void sensorPlacedMsg(bool on);
void tryRecoverSensor();
void shiftWaveOne();
void pushResultsHistory(int s, int h);
void computeResultsStats(int &avgSpo2, int &avgHr, int &minSpo2, int &maxSpo2, int &count);

// ---------- setup ----------
void resetSessionStats() {
  sessionMinSpo2 = 100; sessionMaxSpo2 = 0; sessionSumSpo2 = 0;
  sessionMinHr = 999; sessionMaxHr = 0; sessionSumHr = 0; sessionCount = 0;
  for (int i = 0; i < RESULT_HISTORY; ++i) { resSpo2[i] = 0; resHr[i] = 0; }
  resIdx = 0; resFilled = false;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  CustomWire.begin();
  CustomWire.setClock(400000);
  delay(200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("No OLED found");
    while (1) delay(100);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  if (BEEPER_ENABLED) {
    pinMode(BEEPER_PIN, OUTPUT);
    digitalWrite(BEEPER_PIN, LOW);
  }

  ok = sensor.begin(CustomWire);
  if (!ok) Serial.println("Sensor init failed (PID mismatch) — will continue and attempt recovery.");
  else Serial.println("Sensor init OK");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Guardian Grip Ready");
  display.println("Place finger to start...");
  display.display();

  spo2Smoothed = NAN;
  lastHrUpdateMs = 0;
  lastPageSwitchMs = millis();
  transitionActive = false;
  lowSinceMs = 0;

  resetSessionStats();
  showReadings = false;
  validReadingCount = 0;
}

// ---------- main loop ----------
void loop() {
  // Diagnostics periodically
  if (millis() - lastDiag > 2000) {
    lastDiag = millis();
    sensor.printDiagnostics();
  }

  // Read FIFO up to 4 samples each loop (driver handles actual FIFO)
  ppg_data_t samples[4];
  uint8_t n = sensor.readFIFO(samples, 4);

  if (n > 0) {
    lastSampleMillis = millis();
    if (!dataStarted) dataStarted = true;
  }

  // If we are not recording, still keep a small buffer for contact detection
  for (uint8_t i = 0; i < n; ++i) {
    redBuf[bufIndex] = samples[i].red;
    irBuf[bufIndex] = samples[i].ir;
    ++bufIndex;
    if (bufIndex >= BUF_SIZE) bufIndex = 0;
    ++totalSamples;

    // update waveform scaling (for visual preview even before recording)
    uint32_t low = irBuf[0], high = irBuf[0];
    int end = bufIndex;
    if (end <= 0) end = 1;
    for (int k = 1; k < end; ++k) {
      if (irBuf[k] > high) high = irBuf[k];
      if (irBuf[k] < low) low = irBuf[k];
    }
    uint32_t range = (high - low);
    uint16_t scaled = 0;
    if (range > 0) scaled = (uint16_t)((samples[i].ir - low) * (SCREEN_HEIGHT - 10) / (float)range);
    wf[wfIndex] = scaled;
    wfIndex = (wfIndex + 1) % WAVE_SAMPLES;
    if (!wfFilled && wfIndex == 0) wfFilled = true;
  }

  static unsigned long lastCompute = 0;
  // compute roughly twice per second for BUF_SECONDS >= 4; adjust if you need faster/slower
  if (millis() - lastCompute > 500 && (bufIndex > 0)) {
    computeSpo2AndHr();
    lastCompute = millis();
  }

  if ((millis() - lastSampleMillis) > NO_DATA_TIMEOUT_MS) {
    shiftWaveOne();
    if ((millis() - lastRecoveryAttempt) > RECOVERY_INTERVAL_MS) {
      tryRecoverSensor();
      lastRecoveryAttempt = millis();
    }
  }

  // page auto-switcher
  if (!transitionActive && (millis() - lastPageSwitchMs >= PAGE_INTERVAL_MS)) {
    Page to = (Page)((currentPage + 1) % 4);
    startTransition(to);
  }

  updateDisplay();
  checkSpO2AndAlert();

  delay(10);
}

// ---------- contact & recording helpers ----------
bool checkContactQuality(uint32_t rAvg, uint32_t iAvg, float rRMS, float iRMS) {
  // contact requires DC above threshold and AC amplitude above threshold
  if (iAvg < MIN_DC_LEVEL || rAvg < 10) return false;
  if (iRMS < MIN_AC_RMS || rRMS < MIN_AC_RMS) return false;
  // Require a minimal AC/DC ratio to rule out ambient light
  float iRatio = (iAvg > 0) ? (iRMS / (float)iAvg) : 0.0f;
  if (iRatio < MIN_AC_RATIO) return false;
  return true;
}

void startRecordingSession() {
  if (!recording) {
    recording = true;
    resetSessionStats();
    invalidReadingActive = false;
    invalidCount = 0;
    Serial.println("{\"event\":\"SENSOR_PLACED\",\"state\":\"ON\"}");
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"event\":\"SESSION_START\",\"time\":%lu}", millis());
    Serial.println(buf);
    display.clearDisplay(); display.setCursor(0, 0); display.println("Recording started");
    display.display();
    delay(200);
  }
}

void stopRecordingSession(const char *reason) {
  if (recording) {
    recording = false;
    // Hide values
    showReadings = false;
    validReadingCount = 0;

    char buf[256];
    snprintf(buf, sizeof(buf), "{\"event\":\"SENSOR_PLACED\",\"state\":\"OFF\",\"reason\":\"%s\"}", reason ? reason : "unknown");
    Serial.println(buf);
    // Print summary on JSON
    if (sessionCount > 0) {
      int avgS = sessionSumSpo2 / sessionCount;
      int avgH = sessionSumHr / sessionCount;
      snprintf(buf, sizeof(buf), "{\"event\":\"SESSION_SUMMARY\",\"count\":%d,\"min_spo2\":%d,\"max_spo2\":%d,\"avg_spo2\":%d,\"min_hr\":%d,\"max_hr\":%d,\"avg_hr\":%d}",
               sessionCount, sessionMinSpo2, sessionMaxSpo2, avgS, sessionMinHr, sessionMaxHr, avgH);
      Serial.println(buf);
    }
    display.clearDisplay(); display.setCursor(0, 0); display.println("Recording stopped.");
    display.display();
    delay(200);
  }
}

void sensorPlacedMsg(bool on) {
  if (on) {
    Serial.println("{\"event\":\"SENSOR_PLACED\",\"state\":\"ON\"}");
  } else {
    Serial.println("{\"event\":\"SENSOR_PLACED\",\"state\":\"OFF\"}");
  }
}

// ---------- compute ----------
float computeRMSFromBuffer(const uint32_t *buf, int len, uint32_t avg) {
  if (len <= 0) return 0.0f;
  double sum = 0.0;
  for (int i = 0; i < len; ++i) {
    double v = (double)buf[i] - (double)avg;
    sum += v * v;
  }
  double mean = sum / len;
  return (float)sqrt(mean);
}

void pushResultsHistory(int s, int h) {
  resSpo2[resIdx] = s;
  resHr[resIdx] = h;
  resIdx = (resIdx + 1) % RESULT_HISTORY;
  if (!resFilled && resIdx == 0) resFilled = true;
}

void computeResultsStats(int &avgSpo2, int &avgHr, int &minSpo2, int &maxSpo2, int &count) {
  long long spSum = 0, hrSum = 0; count = 0; minSpo2 = 200; maxSpo2 = -1;
  for (int i = 0; i < RESULT_HISTORY; ++i) {
    int val = resSpo2[i];
    int hv = resHr[i];
    if (val == 0 && hv == 0) continue; // empty slot condition
    count++;
    spSum += val;
    hrSum += hv;
    if (val < minSpo2) minSpo2 = val;
    if (val > maxSpo2) maxSpo2 = val;
  }
  if (count == 0) { avgSpo2 = 0; avgHr = 0; minSpo2 = 0; maxSpo2 = 0; return; }
  avgSpo2 = (int)(spSum / count);
  avgHr = (int)(hrSum / count);
}

// Main compute: calculates Spo2 & HR; uses contact quality and invalid detection
void computeSpo2AndHr() {
  // compute DCs across the entire BUF_SIZE
  uint64_t rSum = 0, iSum = 0;
  for (int i = 0; i < BUF_SIZE; ++i) { rSum += redBuf[i]; iSum += irBuf[i]; }
  uint32_t rAvg = rSum / BUF_SIZE, iAvg = iSum / BUF_SIZE;

  float rRMS = computeRMSFromBuffer(redBuf, BUF_SIZE, rAvg);
  float iRMS = computeRMSFromBuffer(irBuf, BUF_SIZE, iAvg);

  // contact detection (now includes AC/DC ratio)
  bool wasContact = contactOK;
  contactOK = checkContactQuality(rAvg, iAvg, rRMS, iRMS);

  // If contact lost -> stop recording immediately
  if (!contactOK) {
    contactStartMs = 0;
    validReadingCount = 0;
    if (recording) {
      stopRecordingSession("contact_lost");
    }
    invalidReadingActive = false;
    invalidCount = 0;
    // early return
    spo2Smoothed = NAN; hr = 0; lastHrUpdateMs = 0;
    return;
  } else {
    // contact OK: wait CONTACT_STABLE_MS to allow sensor & finger to stabilize
    if (contactStartMs == 0) contactStartMs = millis();
  }

  // If we don't have stable contact yet, skip compute
  if (millis() - contactStartMs < CONTACT_STABLE_MS) {
    return;
  }

  // Basic validity checks for DC & AC once contact stable
  if (iAvg < MIN_DC_LEVEL || rAvg < 10) {
    spo2Smoothed = NAN; hr = 0; lastHrUpdateMs = 0;
    Serial.println("Contact poor; skipping SpO2/HR.");
    return;
  }
  if ((iRMS < MIN_AC_RMS) || (rRMS < MIN_AC_RMS)) {
    spo2Smoothed = NAN; hr = 0; lastHrUpdateMs = 0;
    Serial.print("Weak AC; rRMS="); Serial.print(rRMS); Serial.print(" iRMS="); Serial.println(iRMS);
    return;
  }

  // ratio & spo2 calc
  float ratio = (rRMS / (float)rAvg) / (iRMS / (float)iAvg);
  if (isnan(ratio) || ratio <= 0) { spo2Smoothed = NAN; return; }
  float spo2Calc = SPO2_A - SPO2_B * ratio;
  spo2Calc = constrain(spo2Calc, 50.0f, 100.0f);

  // HR detection via acIR peaks (smoother)
  for (int i = 0; i < BUF_SIZE; ++i) g_acIR[i] = (float)irBuf[i] - (float)iAvg;

  float abs_min_thr = 6.0f;
  float peakThreshold = fmaxf(iRMS * 0.6f, abs_min_thr);

  int lastPeakIdx = -10000;
  int peakIndices[64]; int peakIdxCount = 0;
  int minSamplesBetweenPeaks = (int)(SAMPLE_RATE * 0.30f);

  for (int i = 1; i < BUF_SIZE - 1; ++i) {
    if (g_acIR[i] > g_acIR[i - 1] && g_acIR[i] > g_acIR[i + 1] && g_acIR[i] > peakThreshold) {
      if ((i - lastPeakIdx) > minSamplesBetweenPeaks) {
        if (peakIdxCount < 64) peakIndices[peakIdxCount++] = i;
        lastPeakIdx = i;
      }
    }
  }

  int hrCalc = hr;
  if (peakIdxCount >= 3) {
    double sumIntervals = 0.0;
    for (int k = 1; k < peakIdxCount; ++k) sumIntervals += (peakIndices[k] - peakIndices[k - 1]);
    double avgInterval = sumIntervals / (peakIdxCount - 1);
    hrCalc = (int)round(60.0 * (double)SAMPLE_RATE / avgInterval);
    hrCalc = constrain(hrCalc, 30, 220);
    lastHrUpdateMs = millis();

    // reject sudden extreme jumps (smooth)
    if (hr > 0) {
      float change = fabs((float)hrCalc - (float)hr) / (float)hr;
      if (change > 0.20f) {
        hrCalc = (int)(hr + (hrCalc - hr) * 0.25f);
      }
    }
  }

  // Candidate good reading criteria
  float iRatio = (iAvg > 0) ? (iRMS / (float)iAvg) : 0.0f;
  bool candidateValid = (peakIdxCount >= 3 && iRatio >= MIN_AC_RATIO && spo2Calc >= 50.0f && spo2Calc <= 100.0f && hrCalc >= 30 && hrCalc <= 220);

  // update valid reading counter (debounce)
  if (candidateValid) {
    validReadingCount = min(validReadingCount + 1, VALID_READINGS_TO_SHOW);
  } else {
    validReadingCount = 0;
  }

  // commit "showReadings" and start session only after stable consecutive readings
  if (!showReadings && validReadingCount >= VALID_READINGS_TO_SHOW) {
    showReadings = true;
    startRecordingSession();  // now we start the session (not on contact alone)
  }

  // if previously showing and we lost candidate validity -> stop & hide values
  if (showReadings && !candidateValid) {
    showReadings = false;
    validReadingCount = 0;
    if (recording) stopRecordingSession("unstable_value");
  }

  // EWMA smoothing for HR
  if (hrPrevSmoothed == 0) hrPrevSmoothed = hrCalc;
  hrPrevSmoothed = (int)round(HR_ALPHA * hrCalc + (1.0f - HR_ALPHA) * hrPrevSmoothed);
  hr = hrPrevSmoothed;
  // spo2 smoothing
  const float alpha = 0.25f;
  if (isnan(spo2Smoothed)) spo2Smoothed = spo2Calc;
  else spo2Smoothed = alpha * spo2Calc + (1.0f - alpha) * spo2Smoothed;

  spo2 = (int)round(spo2Smoothed);

  // If not showing readings, do not push results or modify session stats
  if (!showReadings) {
    char dbg[128];
    snprintf(dbg, sizeof(dbg), "Not showing values yet; validCount=%d candidate=%d iRatio=%.3f peaks=%d", validReadingCount, candidateValid ? 1 : 0, iRatio, peakIdxCount);
    Serial.println(dbg);
    return;
  }

  // Standardized value checks - invalid ranges detection (while showing)
  if (spo2 < SPO2_VALID_LOW || spo2 > SPO2_VALID_HIGH || hr < 30 || hr > 220) {
    invalidCount++;
    if (invalidCount >= INVALID_COUNT_THRESHOLD) {
      // set invalid and stop recording
      invalidReadingActive = true;
      stopRecordingSession("invalid_value");
      char evtBuf[128];
      snprintf(evtBuf, sizeof(evtBuf), "{\"event\":\"INVALID_READING\",\"spo2\":%d,\"hr\":%d}", spo2, hr);
      Serial.println(evtBuf);
      beepPattern(2, 120, 2000);
      return;
    }
  } else {
    invalidCount = 0;
    invalidReadingActive = false;
  }

  // Save session stats & history if valid
  if (!isnan(spo2Smoothed) && hr > 0 && !invalidReadingActive && showReadings) {
    pushResultsHistory(spo2, hr);

    sessionSumSpo2 += spo2;
    if (spo2 < sessionMinSpo2) sessionMinSpo2 = spo2;
    if (spo2 > sessionMaxSpo2) sessionMaxSpo2 = spo2;

    sessionSumHr += hr;
    if (hr < sessionMinHr) sessionMinHr = hr;
    if (hr > sessionMaxHr) sessionMaxHr = hr;

    sessionCount++;
  }

  // Debug output:
  char buf[200];
  snprintf(buf, sizeof(buf), "calc: DC_R=%lu DC_I=%lu rRMS=%.1f iRMS=%.1f ratio=%.3f iRatio=%.3f spo2=%.1f peaks=%d hr=%d show=%d",
           (unsigned long)rAvg, (unsigned long)iAvg, rRMS, iRMS, ratio, iRatio, spo2Smoothed, peakIdxCount, hr, showReadings ? 1 : 0);
  Serial.println(buf);
}

// ---------- UI + transitions ----------
void startTransition(Page to) {
  if (to == currentPage) return;
  nextPage = to;
  transitionStartMs = millis();
  transitionActive = true;
}

void drawTransitionFrame() {
  unsigned long elapsed = millis() - transitionStartMs;
  float t = min(1.0f, (float)elapsed / (float)TRANSITION_MS);
  int16_t oldX = (int16_t)(- (float)SCREEN_WIDTH * t);
  int16_t newX = (int16_t)((float)SCREEN_WIDTH * (1.0f - t));
  display.clearDisplay();
  drawPageFrame(currentPage, oldX);
  drawPageFrame(nextPage, newX);
  display.display();
  if (t >= 1.0f) {
    currentPage = nextPage; transitionActive = false; lastPageSwitchMs = millis();
  }
}

void drawStatusTopBar(int16_t xOffset) {
  // top left: recording/contact status
  display.setTextSize(1);
  display.setCursor(0 + xOffset, 0);
  if (invalidReadingActive) {
    display.print("INVALID!");
  } else if (!contactOK) {
    display.print("Place finger");
  } else if (!showReadings) {
    display.print("Stabilizing...");
  } else {
    display.print("Recording");
  }
  // small right indicator for RTC or heartbeat icon (optional)
}

void updateDisplay() {
  static unsigned long lastStaticFrame = 0, lastTransitionFrame = 0;
  if (transitionActive) {
    if ((millis() - lastTransitionFrame) >= FRAME_MS) {
      drawTransitionFrame();
      lastTransitionFrame = millis();
    }
    return;
  }
  if ((millis() - lastStaticFrame) > 500) {
    display.clearDisplay();
    drawPageFrame(currentPage, 0);
    display.display();
    lastStaticFrame = millis();
  }
}

void drawPageFrame(Page p, int16_t xOffset) {
  drawStatusTopBar(xOffset + 2);
  if (p == PAGE_SPO2) {
    display.setTextSize(1);
    display.setCursor(2 + xOffset, 8); display.print("SpO2"); // small label
    display.setTextSize(3);
    char spo2Text[8];
    // show only if reading is allowed
    if (!showReadings || isnan(spo2Smoothed)) snprintf(spo2Text, sizeof(spo2Text), "--");
    else snprintf(spo2Text, sizeof(spo2Text), "%d%%", spo2);
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(spo2Text, 0, 0, &x1, &y1, &w, &h);
    int16_t x = ((SCREEN_WIDTH - w) / 2) + xOffset;
    int16_t y = 8;
    display.setCursor(x, y); display.print(spo2Text);
  } else if (p == PAGE_HR) {
    display.setTextSize(3);
    char hrText[8];
    bool hrValid = (lastHrUpdateMs != 0 && (millis() - lastHrUpdateMs) <= HR_STALE_MS);
    // show only if allowed by showReadings
    if (!showReadings || !hrValid || hr == 0) snprintf(hrText, sizeof(hrText), "--");
    else snprintf(hrText, sizeof(hrText), "%d", hr);
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(hrText, 0, 0, &x1, &y1, &w, &h);
    int16_t x = ((SCREEN_WIDTH - w) / 2) + xOffset; int16_t y = 8;
    display.setCursor(x, y); display.print(hrText);
    display.setTextSize(1); display.setCursor(x + w + 2, y + 6); display.print("BPM");
  } else if (p == PAGE_WAVE) {
    display.setTextSize(1); display.setCursor(0 + xOffset, 8); display.print("Waveform");
    int waveTop = SCREEN_HEIGHT - 8; int waveHeight = 8;
    int samplesToDraw = wfFilled ? WAVE_SAMPLES : wfIndex;
    if (samplesToDraw > 1) {
      int startIdx = (wfIndex + WAVE_SAMPLES - samplesToDraw) % WAVE_SAMPLES;
      int prevX = 0 + xOffset, prevY = waveTop;
      for (int i = 0; i < samplesToDraw; ++i) {
        int idx = (startIdx + i) % WAVE_SAMPLES;
        int xCol = map(i, 0, samplesToDraw - 1, 0, SCREEN_WIDTH - 1) + xOffset;
        uint16_t val = wf[idx];
        int yOff = map(val, 0, SCREEN_HEIGHT - 10, 0, waveHeight - 1);
        int yPos = waveTop - yOff;
        if (i == 0) { prevX = xCol; prevY = yPos; }
        else { display.drawLine(prevX, prevY, xCol, yPos, SSD1306_WHITE); prevX = xCol; prevY = yPos; }
      }
    } else {
      display.setCursor(0 + xOffset, 12); display.setTextSize(1); display.print("Waiting for waveform...");
    }
  } else if (p == PAGE_STATS) {
    int avgSpo2 = 0, avgHr = 0, minSpo2 = 0, maxSpo2 = 0, count = 0;
    computeResultsStats(avgSpo2, avgHr, minSpo2, maxSpo2, count);

    display.setTextSize(1);
    display.setCursor(0 + xOffset, 8);
    if (sessionCount == 0) display.print("No session data");
    else {
      int avgS = sessionSumSpo2 / sessionCount;
      int avgH = sessionSumHr / sessionCount;
      display.print("Session:");
      display.setCursor(0 + xOffset, 16);
      display.print("SpO2 avg:"); display.print(avgS); display.print("%");
      display.setCursor(0 + xOffset, 24);
      display.print("Min:"); display.print(sessionMinSpo2); display.print(" Max:"); display.print(sessionMaxSpo2);
      display.setCursor(64 + xOffset, 16);
      display.print("HR avg:"); display.print(avgH);
      display.setCursor(64 + xOffset, 24);
      display.print("Min:"); display.print(sessionMinHr); display.print(" Max:");
      display.print(sessionMaxHr);
    }
  }
}

// ---------- Alerts & beeper ----------
void checkSpO2AndAlert() {
  if (!recording) {
    return;
  }
  if (isnan(spo2Smoothed) || hr == 0) {
    lowSinceMs = 0;
    return;
  }
  if (spo2 <= SPO2_WARNING_THRESHOLD) {
    if (lowSinceMs == 0) lowSinceMs = millis();
    else {
      if ((millis() - lowSinceMs) >= ALARM_DELAY_MS && !alarmActive) {
        alarmActive = true;
        if (BEEPER_ENABLED) {
          beepPattern(4, 150, 2000);
          digitalWrite(BEEPER_PIN, HIGH);
        }
        char buf[128];
        snprintf(buf, sizeof(buf), "{\"event\":\"SPO2_ALERT\",\"state\":\"ON\",\"spo2\":%d,\"hr\":%d}", spo2, hr);
        Serial.println(buf);
      }
    }
  } else {
    if (alarmActive) {
      alarmActive = false;
      if (BEEPER_ENABLED) {
        digitalWrite(BEEPER_PIN, LOW);
        beepPattern(1, 200, 1500);
      }
      char buf[128];
      snprintf(buf, sizeof(buf), "{\"event\":\"SPO2_ALERT\",\"state\":\"OFF\",\"spo2\":%d,\"hr\":%d}", spo2, hr);
      Serial.println(buf);
    }
    lowSinceMs = 0;
  }
}

// beep utility
void beepPattern(int count, int durationMs, int freqHz) {
  if (!BEEPER_ENABLED) return;
  const int halfPeriodUs = (1000000 / freqHz) / 2;
  for (int b = 0; b < count; ++b) {
    unsigned long start = millis();
    while ((millis() - start) < durationMs) {
      digitalWrite(BEEPER_PIN, HIGH); delayMicroseconds(halfPeriodUs);
      digitalWrite(BEEPER_PIN, LOW); delayMicroseconds(halfPeriodUs);
    }
    delay(120);
  }
}

// ---------- small utility ----------
void shiftWaveOne() {
  uint16_t lastVal = wf[(wfIndex + WAVE_SAMPLES - 1) % WAVE_SAMPLES];
  for (int i = 0; i < WAVE_SAMPLES - 1; i++) wf[i] = wf[i + 1];
  wf[WAVE_SAMPLES - 1] = lastVal;
  wfIndex = (wfIndex + 1) % WAVE_SAMPLES;
  if (!wfFilled && wfIndex == 0) wfFilled = true;
}

void tryRecoverSensor() {
  Serial.println("Attempting sensor re-init...");
  bool ok2 = sensor.begin(CustomWire);
  Serial.print("Recovery begin returned: "); Serial.println(ok2 ? "OK" : "FAIL");
  if (ok2) {
    // clear results history after re-init
    resetSessionStats();
  }
}