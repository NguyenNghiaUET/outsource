#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

// Pins
#define BTN_INC        5
#define BTN_SELECT     6
#define BTN_CHANGE     7
#define BUZZER_PIN     8
#define SENSOR_PIN     9

#define OLED_SDA       10
#define OLED_SCL       11
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  32
unsigned long lastEventMs = 0;
// Biến quản lý Gap kéo dài
unsigned long lastEventTimestamp = 0;       // thời điểm xảy ra event cuối cùng
float averageGap = 0.0f;                    // gap trung bình của 3 event gần nhất
const int CANDIDATE_THRESHOLD = 3; // Số lần cần lặp lại trước khi chấp nhận
const float CHANGE_THRESHOLD = 0.5f; // Ngưỡng khác biệt (50%) để xem là lớn
unsigned long gapCount = 0;
bool rateLock = true;
float lastGap = 0.0f; // Biến toàn cục
float Rn = 0.0f;
float Rn_trong_pause = 0.0f;  
unsigned long runningStartTime = 0;
  static unsigned long lastEventProcessed = 0;
  static unsigned long prevGap = 0;      // gap trước đó
  static float smoothRo = 0.0f;
  static bool readyForRo = false;
  const float alpha = 0.7f;

// Các biến kiểm soát trạng thái beep và thứ tự beep
bool isBeepingErr = false;
bool isBeepingDone = false;
unsigned long beepTimer = 0;
int beepCount = 0;
bool errorCode = false;
bool doneFlag = false;  
// Blink mask hiển thị ERR và DONE
unsigned long blinkTimer = 0;
bool blinkOn = false;
const int REQUIRED_STABLE_SAMPLES = 3;  // cần 3 mẫu ổn định trước khi unlock
unsigned long lastRnUpdate = 0;
const unsigned long RN_UPDATE_INTERVAL = 1000; // 1 giây cập nhật Rn một lần
const int STABLE_REQUIRED_COUNT = 2;
const float DEAD_BAND = 0.15f;
const unsigned long BTN_DEBOUNCE_MS = 50;
const unsigned long EVENT_DEBOUNCE_MS = 100;  // ms
const float NOISE_THRESHOLD = 25.0f;
const float FALL_THRESHOLD_RATIO = 0.3f;
const int SAMPLE_DELAY_MS = 2;  // lấy mẫu 500Hz (2ms/lần)
const unsigned long MIN_RAW_DT_US = 1000UL;      // Loại nhiễu cao tần <1ms
const unsigned long MIN_DROP_DT_US = 100000UL;   // Tối đa 10 event/s
volatile int rateNowEventCounter = 0;  
volatile unsigned long rateNowLastEventMs = 0;
volatile unsigned long rateNowCurrentEventMs = 0; 
const int DROPS_PER_CC = 20;
int A = 0, B = 0, C = 0; 
volatile bool rateReadyToAccept = false; 
volatile float rateNowRo = 0.0f;
volatile float Ro_stable = 0.0f;
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);
enum State {
  STATE_BOOT,
  STATE_MENU,
  STATE_SET_VOL,
  STATE_SET_RATIO,
  STATE_SET_CUSTOM,
  STATE_SET_TIME,
  STATE_SET_SENSITIVITY,
  STATE_RATE,
  STATE_RUNNING,
  STATE_PAUSE,
};
struct Button {
  uint8_t pin;
  bool stableState;
  bool lastReading;
  unsigned long lastChange;
};
State currentState = STATE_BOOT;
unsigned long stateStartTime = 0;
  // đổi sang ms để hiển thị chuẩn

Button btnInc = {BTN_INC, HIGH, HIGH, 0};
Button btnSelect = {BTN_SELECT, HIGH, HIGH, 0};
Button btnChange = {BTN_CHANGE, HIGH, HIGH, 0};
bool highlightVol = true;
bool highlightRatio = true;
int volDigits[3] = {0, 0, 0};
int volCursor = 0;
int ratioVal = 20;
int ratioDigits[2] = {2, 0};
int ratioCursor = 0;
int timeDigits[6] = {0, 0, 0, 0, 0, 0};
int timeCursor = 0;
int sensitivityIdx = 0;
bool sensitivityOffMode = false; // "-Off" mode
const float sensitivities[3] = {1.05f, 1.10f, 1.30f};
bool useVolumeSet = false;
unsigned long targetTimeSec = 0;
unsigned long setValueVol = 0;
unsigned long setValueTime = 0;
unsigned long rateStartTime = 0;
unsigned long pausedElapsedMs = 0;
unsigned long lastEventUs = 0;
unsigned long lastDt = 0;
float currentSmoothedRate = 0.0f;
volatile bool flagBtnInc = false;
volatile bool flagBtnSelect = false;
volatile bool flagBtnChange = false;
unsigned long beepCooldown = 0;  // Biến quản lý cooldown của loa beep
volatile unsigned long lastEventGap = 0;
// Event counter (state running)
unsigned long eventCount = 0;   // 1=Rate ERR, 2=Blur ERR
unsigned long lastDraw = 0;
unsigned long selectHoldTimer = 0;
// Event detection variables (Core 0)
volatile float smoothedValue = 0;
volatile float peakValue = 0;
volatile float riseSum = 0;
volatile float fallSum = 0;
volatile float prevValue = 0;
volatile bool isRising = false;
volatile bool peakDetected = false;
volatile unsigned long lastPeakTime = 0;
volatile bool eventFlag = false;
volatile unsigned long eventTimestamps[10] = {0};
volatile int eventTimestampIndex = 0;
unsigned long detectedTime = 0;
unsigned long gap = 0;
unsigned long displayCount = 0;
unsigned long toneTimer = 0;
bool thresholdUseHz = false;
int thresholdRateInt = 0;
float thresholdSecPerDrop = 0.0f;
unsigned long lastEventTimeCheck = 0;
const float MAX_ALLOWED_GAP_FACTOR = 2.0f; // gap lớn gấp đôi là ERR
bool isStable = false;
float adaptiveSmoothRate(float currentRate, float newRate) {
    static float candidateRate = 0.0f;
    static int candidateCount = 0;

    if (currentRate <= 0) {
        isStable = false;
        return newRate;
    }

    float diffRatio = fabs(newRate - currentRate) / currentRate;

    // Nếu chênh lệch nhỏ
    if (diffRatio < DEAD_BAND) {
        candidateCount++;
        if (candidateCount >= STABLE_REQUIRED_COUNT) {
            isStable = true; // đặt isStable khi đã ổn định
            candidateCount = STABLE_REQUIRED_COUNT; // giữ không vượt quá
        }
        return currentRate * 0.6f + newRate * 0.4f;
    } else {
        // Chênh lệch lớn -> chưa ổn định
        candidateCount = 0;
        isStable = false; // reset isStable ở đây!
        return currentRate * 0.8f + newRate * 0.2f;  // di chuyển nhẹ sang giá trị mới
    }
}
void doReset() {
  currentState = STATE_BOOT;
  stateStartTime = millis();
  // Reset trạng thái chung
  highlightVol = true;
  highlightRatio = true;
  memset(volDigits, 0, sizeof(volDigits));
  volCursor = 0;
  ratioVal = 20;
  ratioDigits[0] = 2;
  ratioDigits[1] = 0;
  ratioCursor = 0;
  memset(timeDigits, 0, sizeof(timeDigits));
  timeCursor = 0;
  sensitivityIdx = 0;
  sensitivityOffMode = false;
  // Reset trạng thái beep
  beepCooldown = 0;
  isBeepingErr = false;
  isBeepingDone = false;
  beepCount = 0;
  beepTimer = 0;
  // Reset trạng thái running và lỗi
  eventCount = 0;
  pausedElapsedMs = 0;
  errorCode = false;
  doneFlag = false;
  blinkTimer = millis();
  blinkOn = false;
  // Reset ratenow hoàn chỉnh (quan trọng nhất)
  Ro_stable = 0.0f;
  rateNowRo = 0.0f;
  rateNowEventCounter = 0;
  rateNowLastEventMs = 0;
  rateNowCurrentEventMs = 0;
  rateReadyToAccept = false;
  currentSmoothedRate = 0.0f;
  rateStartTime = 0;
  lastEventMs = millis();
  lastEventUs = micros();
  // Reset detectEvent variables
  noInterrupts();
  smoothedValue = peakValue = riseSum = fallSum = prevValue = 0;
  isRising = peakDetected = eventFlag = false;
  eventTimestampIndex = 0;
  memset((void*)eventTimestamps, 0, sizeof(eventTimestamps));
  interrupts();

  lastDraw = millis();
   pausedElapsedMs = 0;
    eventCount = 0;
}
void detectEventTask(void *pvParameters) {
    const int ADC_SAMPLE_DELAY_MS = 2; // lấy mẫu mỗi 2ms (500Hz)
    const float SMOOTH_FACTOR = 0.3f;  // hệ số làm mềm ADC
    const float RISE_THRESHOLD = 25.0f;
    const float FALL_RATIO = 0.3f;
    unsigned long currentTime;

    for (;;) {
        currentTime = millis();
        int rawADC = analogRead(SENSOR_PIN);

        noInterrupts(); // Đảm bảo an toàn khi sửa đổi giá trị dùng chung
        smoothedValue = SMOOTH_FACTOR * rawADC + (1 - SMOOTH_FACTOR) * smoothedValue;

        bool debounce = (currentTime - lastPeakTime) < EVENT_DEBOUNCE_MS;

        if (!debounce) {
            if (smoothedValue > prevValue + RISE_THRESHOLD) {
                if (!isRising) {
                    // Bắt đầu một lần tăng mới
                    isRising = true;
                    peakValue = smoothedValue;
                    riseSum = 0;
                    fallSum = 0;
                    peakDetected = false;
                }
                riseSum += (smoothedValue - prevValue);
                peakValue = max(smoothedValue, peakValue);
            }
            else if (smoothedValue < prevValue - RISE_THRESHOLD && isRising) {
                // Phát hiện đỉnh xong và bắt đầu giảm xuống
                isRising = false;
                peakDetected = true;
            }

            if (peakDetected) {
                fallSum += (prevValue - smoothedValue);
                if (fallSum >= (FALL_RATIO * riseSum)) {
                    // Phát hiện biến cố thành công
                    unsigned long detectedTime = millis();
                    unsigned long gap = detectedTime - lastEventTimestamp;

                    noInterrupts();
                    eventFlag = true;
                    lastEventTimestamp = detectedTime;
                    lastEventGap = gap;
                    interrupts();


                    // Cập nhật gap trung bình, chỉ nếu gap nằm trong khoảng hợp lý (50ms - 5000ms)
                    if (gap > 50 && gap < 5000) {
                        if (averageGap == 0.0f)
                            averageGap = gap;
                        else
                            averageGap = 0.7f * averageGap + 0.3f * gap;
                    }

                    // Cập nhật thời gian sự kiện cuối cùng
                    lastEventTimestamp = detectedTime;

                    // Cập nhật timestamps (nếu cần dùng)
                    eventTimestamps[eventTimestampIndex] = detectedTime;
                    eventTimestampIndex = (eventTimestampIndex + 1) % 10;

                    // Reset biến kiểm soát peak
                    peakDetected = false;
                    riseSum = 0;
                    fallSum = 0;
                    lastPeakTime = detectedTime;
                }
            }
        }

        prevValue = smoothedValue;
        interrupts();

        delay(ADC_SAMPLE_DELAY_MS); // lấy mẫu ở tần số 500Hz
    }
}

void playBeep(int duration_ms, int freq) {
  if (millis() > beepCooldown) {
    tone(BUZZER_PIN, freq, duration_ms);
    beepCooldown = millis() + duration_ms + 100;  // cooldown sau mỗi beep
  }
}

void handleBeeps() {
  static unsigned long beepTimer = 0;
  static int beepCount = 0;
  unsigned long now = millis();

  // DONE có ưu tiên tuyệt đối cao nhất
  if (isBeepingDone) {
    if (now - beepTimer >= 2000 || beepTimer == 0) {
      playBeep(600, 1000);
      beepTimer = now;
      beepCooldown = now + 800; 
    }
    return;
  }

  // ERR: 4 beep nhanh, nghỉ 500ms
  if (isBeepingErr && !sensitivityOffMode) {
    if (beepCount < 4 && (now - beepTimer >= 150 || beepTimer == 0)) { // nhịp nhanh hơn
      playBeep(100, 1500);
      beepTimer = now;
      beepCooldown = now + 150; 
      beepCount++;
    } else if (beepCount >= 4 && now - beepTimer >= 500) { // nghỉ 500ms
      beepCount = 0;
      beepTimer = now;
    }
    return;
  }
}

// beep mỗi biến cố (âm cao 2500Hz rất ngắn 30ms)
void beepEvent() {
    if (millis() > beepCooldown) {
        tone(BUZZER_PIN, 4000, 30); // beep nhẹ 10ms
        beepCooldown = millis() + 30;  
    }
}

void drawBoot() {
  const char* text = "MINI LAB";
  u8g2.setFont(u8g2_font_ncenB14_tr);

  // Tính toán vị trí canh giữa ngang
  int16_t textWidth = u8g2.getStrWidth(text);
  int16_t x = (SCREEN_WIDTH - textWidth) / 2 - 2;  // canh trái 2px
  
  // Tính toán vị trí canh giữa dọc
  int16_t fontHeight = u8g2.getMaxCharHeight();
  int16_t y = (SCREEN_HEIGHT - fontHeight) / 2 + fontHeight - 3; // nhích lên 3px

  u8g2.clearBuffer();
  u8g2.drawStr(x, y, text);
  u8g2.sendBuffer();
}
void drawMenu(bool factor) {
  // Nội dung
  const char* vol = "VOLSET";
  const char* slash = " / ";
  const char* time = "TIMESET";

  // Cài font
  u8g2.setFont(u8g2_font_ncenB08_tr);
  int fontH = u8g2.getMaxCharHeight();

  // Tính chiều rộng của từng phần
  int wVol = u8g2.getStrWidth(vol);
  int wSlash = u8g2.getStrWidth(slash);
  int wTime = u8g2.getStrWidth(time);
  int totalW = wVol + wSlash + wTime;

  // Tọa độ căn giữa
  int x0 = (SCREEN_WIDTH - totalW) / 2;
  int y = (SCREEN_HEIGHT - fontH) / 2 + fontH;

  // Padding và bo góc
  const int padX = 2;
  const int padY = 1;
  const int radius = 3;

  // Bắt đầu vẽ
  u8g2.clearBuffer();

  if (factor) {
    // Highlight VOLSET
    int bx = x0 - padX;
    int by = y - fontH - padY / 2 + 1;
    int bw = wVol + padX * 2;
    int bh = fontH + padY;

    u8g2.setDrawColor(1);
    u8g2.drawRBox(bx, by, bw, bh, radius);
    u8g2.setDrawColor(0);
    u8g2.drawStr(x0, y, vol);

    // Slash và TIMESET
    u8g2.setDrawColor(1);
    u8g2.drawStr(x0 + wVol, y, slash);
    u8g2.drawStr(x0 + wVol + wSlash, y, time);
  } else {
    // VOLSET và slash thường
    u8g2.setDrawColor(1);
    u8g2.drawStr(x0, y, vol);
    u8g2.drawStr(x0 + wVol, y, slash);

    // Highlight TIMESET
    int bx = x0 + wVol + wSlash - padX;
    int by = y - fontH - padY / 2;
    int bw = wTime + padX * 2;
    int bh = fontH + padY;

    u8g2.drawRBox(bx, by, bw, bh, radius);
    u8g2.setDrawColor(0);
    u8g2.drawStr(x0 + wVol + wSlash, y, time);
  }

  u8g2.sendBuffer();
}
void drawSetVol(int A, int B, int C, int cursorPos) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%d%d%dcc", A, B, C);
  u8g2.setFont(u8g2_font_ncenB08_tr);
  int fontH = u8g2.getMaxCharHeight();

  int textW = u8g2.getStrWidth(buf);
  int x0 = (SCREEN_WIDTH - textW) / 2;
  int y = (SCREEN_HEIGHT - fontH) / 2 + fontH;

  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.drawStr(x0, y, buf);

  // Chỉnh lại gạch chân theo vị trí cursor chính xác:
  int charW = u8g2.getStrWidth("0");
  int lineX = x0 + (charW * cursorPos);
  if (cursorPos == 1) lineX += 1;  // dịch sang phải 1px ở ký tự 2
  if (cursorPos == 2) lineX += 2;  // dịch sang phải 2px ở ký tự 3

  int lineY = y + 2;
  int lineH = 2;
  u8g2.drawBox(lineX, lineY, charW, lineH);

  u8g2.sendBuffer();
}

void drawRatioSet(bool factor2) {
  const char* leftText = "20d/cc";
  const char* sep = " - ";
  const char* rightText = "Custom";

  u8g2.setFont(u8g2_font_ncenB08_tr);
  int fontH = u8g2.getMaxCharHeight();

  int wLeft = u8g2.getStrWidth(leftText);
  int wSep = u8g2.getStrWidth(sep);
  int wRight = u8g2.getStrWidth(rightText);
  int totalW = wLeft + wSep + wRight;

  int x0 = (SCREEN_WIDTH - totalW) / 2;
  int y = (SCREEN_HEIGHT - fontH) / 2 + fontH;

  u8g2.clearBuffer();

  const int padX = 2;
  const int padY_top = 0;   // giảm trên 1px
  const int padY_bottom = 3; // mở rộng xuống dưới 2px
  const int radius = 3;

  if (factor2) {
    int bx = x0 - padX;
    int by = y - fontH - padY_top;
    int bw = wLeft + padX * 2;
    int bh = fontH + padY_top + padY_bottom;
    u8g2.setDrawColor(1);
    u8g2.drawRBox(bx, by, bw, bh, radius);
    u8g2.setDrawColor(0);
    u8g2.drawStr(x0, y, leftText);
    u8g2.setDrawColor(1);
    u8g2.drawStr(x0 + wLeft, y, sep);
    u8g2.drawStr(x0 + wLeft + wSep, y, rightText);
  } else {
    u8g2.setDrawColor(1);
    u8g2.drawStr(x0, y, leftText);
    u8g2.drawStr(x0 + wLeft, y, sep);
    int bx = x0 + wLeft + wSep - padX;
    int by = y - fontH - padY_top;
    int bw = wRight + padX * 2;
    int bh = fontH + padY_top + padY_bottom;
    u8g2.drawRBox(bx, by, bw, bh, radius);
    u8g2.setDrawColor(0);
    u8g2.drawStr(x0 + wLeft + wSep, y, rightText);
  }

  u8g2.sendBuffer();
}
void drawSetCustom(int D, int E, int cursorPosition) {
  int value = D * 10 + E;
  value = constrain(value, 5, 99);
  D = value / 10;
  E = value % 10;

  char buf[8];
  snprintf(buf, sizeof(buf), "%02dd/cc", value);

  u8g2.setFont(u8g2_font_ncenB08_tr);
  int fontH = u8g2.getMaxCharHeight();
  int textW = u8g2.getStrWidth(buf);
  int x0 = (SCREEN_WIDTH - textW) / 2;
  int y = (SCREEN_HEIGHT - fontH) / 2 + fontH;

  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.drawStr(x0, y, buf);

  char digit[2] = { buf[cursorPosition], '\0' };

  // Sửa tại đây: thêm offset 1px khi cursorPosition = 1
  int underlineOffsetX = u8g2.getStrWidth(cursorPosition == 0 ? "" : "0") + (cursorPosition == 1 ? 1 : 0);

  int charW = u8g2.getStrWidth(digit);
  int lineY = y + 2;
  u8g2.drawBox(x0 + underlineOffsetX, lineY, charW, 2);
  u8g2.sendBuffer();
}

void drawSetTime(int F, int G, int H, int I, int J, int K, int cursorPosition) {
  int hours = F * 10 + G;
  int minutes = H * 10 + I;
  int seconds = J * 10 + K;

  char buf[12];
  snprintf(buf, sizeof(buf), "%02dh %02dm %02ds", hours, minutes, seconds);

  u8g2.setFont(u8g2_font_ncenB08_tr);
  int fontH = u8g2.getMaxCharHeight();
  int textW = u8g2.getStrWidth(buf);
  int x0 = (SCREEN_WIDTH - textW) / 2;
  int y = (SCREEN_HEIGHT - fontH) / 2 + fontH;

  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.drawStr(x0, y, buf);

  // Xác định lại vị trí chính xác cho từng digit, thêm offset 1px vào digit thứ 2 của mỗi cặp (h, m, s)
  int digitPositions[] = {
    0,                                // 0: giờ hàng chục (F)
    u8g2.getStrWidth("0") + 1,        // 1: giờ hàng đơn vị (G) ← thêm 1px
    u8g2.getStrWidth("00h "),         // 2: phút hàng chục (H)
    u8g2.getStrWidth("00h 0") + 1,    // 3: phút hàng đơn vị (I) ← thêm 1px
    u8g2.getStrWidth("00h 00m "),     // 4: giây hàng chục (J)
    u8g2.getStrWidth("00h 00m 0") + 1 // 5: giây hàng đơn vị (K) ← thêm 1px
  };

  int underlineX = x0 + digitPositions[cursorPosition];
  int lineY = y + 2;
  int charW = u8g2.getStrWidth("0");

  u8g2.drawBox(underlineX, lineY, charW, 2);
  u8g2.sendBuffer();
}
void enterRunning(float recentRn) {
    Rn = recentRn;
    runningStartTime = millis();

    errorCode = false;
    doneFlag = false;
    isBeepingErr = false;
    isBeepingDone = false;
    beepCooldown = millis();
    beepTimer = 0;
    beepCount = 0;

    gapCount = 0;
    averageGap = 0.0f;

    lastEventTimestamp = millis();
    lastEventMs = millis();
    lastEventUs = micros();

    blinkTimer = millis();
    blinkOn = false;

    noInterrupts();
    eventFlag = false;
    eventTimestampIndex = 0;
    memset((void*)eventTimestamps, 0, sizeof(eventTimestamps));
    interrupts();

    // Reset logic tính Rn trong RUNNING
    lastEventProcessed = millis();
    prevGap = 0;
    smoothRo = recentRn;
    readyForRo = false;

    // Lưu ý: KHÔNG reset pausedElapsedMs và eventCount tại đây nữa
    // (Hai biến này chỉ reset trong hàm doReset() ban đầu)
}

void enterPause(unsigned long elapsedMsAtPause, unsigned long eventCountAtPause) {
    errorCode = false;
    doneFlag = false;
    isBeepingDone = false;
    isBeepingErr = false;
    beepCooldown = millis();

    pausedElapsedMs = elapsedMsAtPause;  // Bảo lưu thời gian đã trôi qua
    eventCount = eventCountAtPause;      // Bảo lưu số lượng sự kiện đã đếm
}

bool readButton(Button &btn) {
  bool reading = digitalRead(btn.pin);
  
  if (reading != btn.lastReading) {
    btn.lastChange = millis();
    btn.lastReading = reading;
  }

  if ((millis() - btn.lastChange) > BTN_DEBOUNCE_MS) {
    if (reading != btn.stableState) {
      btn.stableState = reading;
      if (reading == LOW) { // Chỉ trả về true khi nút được nhấn (tín hiệu LOW)
        return true;
      }
    }
  }

  return false;
}
float calculateAverageGap() {
  noInterrupts();
  unsigned long timestampsCopy[10];
  memcpy((void*)timestampsCopy, (const void*)eventTimestamps, sizeof(eventTimestamps));
  int currentIndex = eventTimestampIndex;
  interrupts();

  int validEvents = 0;
  unsigned long totalGap = 0;

  for (int i = 0; i < 9; i++) {
    int idx1 = (currentIndex - 1 - i + 10) % 10;
    int idx2 = (currentIndex - 2 - i + 10) % 10;

    if (timestampsCopy[idx1] > timestampsCopy[idx2]) {
      totalGap += timestampsCopy[idx1] - timestampsCopy[idx2];
      validEvents++;
    }
  }

  if (validEvents < 2) return 0.0f; // Yêu cầu ít nhất 3 điểm để có 2 khoảng cách hợp lệ
  return float(totalGap) / float(validEvents);
}

float getSmoothedRate() {
  float avgGapMs = calculateAverageGap();
  if (avgGapMs < 0.001f) return 0.0f;
  return 1000.0f / avgGapMs;
}
void setRunningParameters() {
  rateStartTime = millis(); 
  pausedElapsedMs = 0;
  eventCount = 0;
  displayCount = 0;
  errorCode = 0;
  doneFlag = false;
  blinkTimer = millis();
  blinkOn = false;
  toneTimer = millis();

  if (Ro_stable >= 1.0f) {
    thresholdUseHz = true;
    thresholdRateInt = int(Ro_stable + 0.5f);
    thresholdSecPerDrop = 0;
  } else {
    thresholdUseHz = false;
    thresholdRateInt = 0;
    thresholdSecPerDrop = (rateNowCurrentEventMs - rateNowLastEventMs) / 1000.0f;
  }

  if (targetTimeSec > 0) {
    useVolumeSet = false;
    setValueTime = targetTimeSec;   // ✅ đảm bảo gán đúng
  } else {
    useVolumeSet = true;
    setValueVol = volDigits[0]*100 + volDigits[1]*10 + volDigits[2];  // ✅ đảm bảo gán đúng
  }

  // Ro_stable đã lấy từ trước, nên không cần sửa ở đây.
}
void drawRateNow(float rate, bool ready, bool isStable) {
    char buf[16];
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    if (ready && rate > 0.0f) {
        if (rate >= 1.0f) {
            snprintf(buf, sizeof(buf), "%s%.1fd/s", (isStable ? "" : "?"), rate);
        } else {
            snprintf(buf, sizeof(buf), "%s%.1fs/d", (isStable ? "" : "?"), (1.0f / rate));
        }
    } else {
        strcpy(buf, "Measuring...");
    }

    int textW = u8g2.getStrWidth(buf);
    int fontH = u8g2.getMaxCharHeight();
    int x = (SCREEN_WIDTH - textW) / 2;
    int y = (SCREEN_HEIGHT + fontH) / 2;
    u8g2.drawStr(x, y, buf);
    u8g2.sendBuffer();
}

void drawRunning(
    bool blinkOn, bool useVolumeSet, 
    unsigned long elapsedSec, unsigned long setValueTime, 
    unsigned long volNow, unsigned long setValueVol,
    float Ro_stable, float currentSmoothedRate
) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    char buf[20];

    //--- Góc trên trái (Vset hoặc Tset) ---
    if (useVolumeSet) {
        snprintf(buf, sizeof(buf), "Vset: %03dcc", setValueVol);
    } else {
        unsigned long hh = setValueTime / 3600;
        unsigned long mm = (setValueTime % 3600) / 60;
        unsigned long ss = setValueTime % 60;
        snprintf(buf, sizeof(buf), "Tset: %02dh%02dm%02ds", hh, mm, ss);
    }
    u8g2.drawStr(0, 10, buf);

    //--- Góc trên phải (Ro_stable), dịch sang phải thêm 3 ký tự ---
    if (Ro_stable >= 1.0f) {
        snprintf(buf, sizeof(buf), "%.1fd/s", Ro_stable);
    } else {
        snprintf(buf, sizeof(buf), "%.1fs/d", (1.0f / Ro_stable));
    }
    int xRo = 80 + u8g2.getStrWidth("000");  // dịch phải 3 ký tự "0"
    u8g2.drawStr(xRo, 10, buf);

    //--- Góc dưới trái (Vnow hoặc Tnow) ---
    u8g2.setDrawColor(0);
    u8g2.drawBox(0, 20, 79, 12);
    u8g2.setDrawColor(1);

    if (useVolumeSet) {
        snprintf(buf, sizeof(buf), "Vn: %03dcc", volNow);
    } else {
        unsigned long hh = elapsedSec / 3600;
        unsigned long mm = (elapsedSec % 3600) / 60;
        unsigned long ss = elapsedSec % 60;

        // Chỉ hiển thị phần thời gian đã trôi qua
        if (hh > 0) {
            snprintf(buf, sizeof(buf), "Tn: %dh%dm%ds", hh, mm, ss);
        } else if (mm > 0) {
            snprintf(buf, sizeof(buf), "Tn: %dm%ds", mm, ss);
        } else {
            snprintf(buf, sizeof(buf), "Tn: %ds", ss);
        }
    }

    if (doneFlag && blinkOn) {
        u8g2.drawBox(0, 20, u8g2.getStrWidth(buf), 10);
    } else {
        u8g2.drawStr(0, 30, buf);
    }

    //--- Góc dưới phải (Rn), dịch sang phải thêm 3 ký tự ---
    if (currentSmoothedRate >= 1.0f) {
        snprintf(buf, sizeof(buf), "%.1fd/s", currentSmoothedRate);
    } else {
        snprintf(buf, sizeof(buf), "%.1fs/d", (1.0f / currentSmoothedRate));
    }
    int xRn = 80 + u8g2.getStrWidth("000");  // dịch phải 3 ký tự "0"

    if (errorCode && blinkOn) {
        u8g2.drawBox(xRn, 20, u8g2.getStrWidth(buf), 10);
    } else {
        u8g2.drawStr(xRn, 30, buf);
    }

    u8g2.sendBuffer();
}

void enterRate() {
  noInterrupts();
  rateNowEventCounter = 0;
  rateReadyToAccept = false;
  rateNowRo = 0.0f;
  rateNowLastEventMs = 0;
  rateNowCurrentEventMs = 0;
  eventFlag = false;
  memset((void*)eventTimestamps, 0, sizeof(eventTimestamps)); // reset kỹ bộ đệm event
  interrupts();
}
void drawPause(float Ro_stable, float currentSmoothedRate, bool readyPauseRn) {
    u8g2.clearBuffer();

    // Chữ PAUSE to, đậm, căn dọc giữa, lệch trái
    u8g2.setFont(u8g2_font_ncenB14_tr);
    const char *pauseStr = "PAUSE";
    int pauseY = (SCREEN_HEIGHT + u8g2.getMaxCharHeight()) / 2 - 2;
    u8g2.drawStr(0, pauseY, pauseStr);

    // Ro_stable (góc trên phải, chỉ hiển thị giá trị, bỏ "Ro:")
    u8g2.setFont(u8g2_font_ncenB08_tr);
    char roStr[12];
    if (Ro_stable >= 1.0f)
        snprintf(roStr, sizeof(roStr), "%.1fd/s", Ro_stable);
    else
        snprintf(roStr, sizeof(roStr), "%.1fs/d", (1.0f / Ro_stable));

    int roWidth = u8g2.getStrWidth(roStr);
    u8g2.drawStr(SCREEN_WIDTH - roWidth, 10, roStr);

    // Rn (góc dưới phải, kèm dấu "?")
    char rnStr[12];
    if (currentSmoothedRate >= 1.0f)
        snprintf(rnStr, sizeof(rnStr), "%s%.1fd/s", readyPauseRn ? "" : "?", currentSmoothedRate);
    else
        snprintf(rnStr, sizeof(rnStr), "%s%.1fs/d", readyPauseRn ? "" : "?", (1.0f / currentSmoothedRate));

    int rnWidth = u8g2.getStrWidth(rnStr);
    u8g2.drawStr(SCREEN_WIDTH - rnWidth, SCREEN_HEIGHT - 1, rnStr);

    u8g2.sendBuffer();
}

void setup() {
  // Khởi động Serial để debug (nếu cần)
  Serial.begin(115200);

  pinMode(BTN_INC, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BTN_CHANGE, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT);

  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);

  doReset();

  xTaskCreatePinnedToCore(
    detectEventTask,    // Hàm detect event riêng
    "detectEventTask",  // Tên task
    4096,               // Stack size (4KB đủ cho task này)
    NULL,               // Tham số (không dùng)
    1,                  // Độ ưu tiên
    NULL,               // Không cần giữ handle
    0                   // Chạy trên core 0
  );
}

void loop() {
  // Xử lý giữ nút SELECT để reset
  if (digitalRead(BTN_SELECT) == LOW) {
    if (selectHoldTimer == 0) {
      selectHoldTimer = millis();
    } else if (millis() - selectHoldTimer > 2000) { 
      doReset(); 
      selectHoldTimer = 0; 
      return; 
    }
  } else {
    selectHoldTimer = 0;
  }

  bool incPressed = readButton(btnInc);
  bool selPressed = readButton(btnSelect);
  bool chgPressed = readButton(btnChange);
handleBeeps();
  switch(currentState) {

    case STATE_BOOT:
      drawBoot();
      if (millis() - stateStartTime >= 1000) {
        currentState = STATE_MENU;
      }
      break;

    case STATE_MENU:
      if (incPressed || chgPressed) highlightVol = !highlightVol;
      if (selPressed) {
        if (highlightVol) {
          volCursor = 0; targetTimeSec = 0; eventCount = 0;
          currentState = STATE_SET_VOL;
        } else {
          timeCursor = 0;
          currentState = STATE_SET_TIME;
        }
      }
      drawMenu(highlightVol);
      break;

    case STATE_SET_VOL:
      if (incPressed) {
        if (volCursor == 0) {
          volDigits[0] = (volDigits[0] + 1) % 6;
          if (volDigits[0] == 5) volDigits[1] = volDigits[2] = 0;
        } else if (volDigits[0] < 5) {
          volDigits[volCursor] = (volDigits[volCursor] + 1) % 10;
        }
      }
      if (chgPressed) volCursor = (volCursor + 1) % 3;
      if (selPressed) {
        int v = volDigits[0]*100 + volDigits[1]*10 + volDigits[2];
        if (v > 0 && v <= 500) {
          highlightRatio = true;
          currentState = STATE_SET_RATIO;
        }
      }
      drawSetVol(volDigits[0], volDigits[1], volDigits[2], volCursor);
      break;

    case STATE_SET_RATIO:
      if (incPressed || chgPressed) highlightRatio = !highlightRatio;
      if (selPressed) {
        if (highlightRatio) {
          ratioVal = 20;
          currentState = STATE_SET_SENSITIVITY;
        } else {
          ratioCursor = 0;
          ratioDigits[0] = ratioVal / 10;
          ratioDigits[1] = ratioVal % 10;
          currentState = STATE_SET_CUSTOM;
        }
      }
      drawRatioSet(highlightRatio);
      break;

    case STATE_SET_CUSTOM:
      if (incPressed) {
        if (ratioCursor == 0) {
          ratioDigits[0] = (ratioDigits[0] + 1) % 10;
          if (ratioDigits[0] == 0 && ratioDigits[1] < 5) ratioDigits[1] = 5;
        } else {
          ratioDigits[1] = (ratioDigits[1] + 1) % 10;
        }
      }
      if (chgPressed) ratioCursor = (ratioCursor + 1) % 2;
      if (selPressed) {
        ratioVal = ratioDigits[0]*10 + ratioDigits[1];
        if (ratioVal >= 5 && ratioVal <= 99) {
          currentState = STATE_SET_SENSITIVITY;
        }
      }
      drawSetCustom(ratioDigits[0], ratioDigits[1], ratioCursor);
      break;

case STATE_SET_TIME:
  if (incPressed) {
    switch (timeCursor) {
      case 0:  // giờ hàng chục (0-2)
        timeDigits[0] = (timeDigits[0] + 1) % 3;
        if (timeDigits[0] == 2 && timeDigits[1] > 3) 
          timeDigits[1] = 3;  // giới hạn max 23h
        break;

      case 1:  // giờ hàng đơn vị
        if (timeDigits[0] == 2)
          timeDigits[1] = (timeDigits[1] + 1) % 4;  // 20-23h
        else
          timeDigits[1] = (timeDigits[1] + 1) % 10; // 00-19h
        break;

      case 2:  // phút hàng chục (0-5)
        timeDigits[2] = (timeDigits[2] + 1) % 6;
        break;

      case 3:  // phút hàng đơn vị (0-9)
        timeDigits[3] = (timeDigits[3] + 1) % 10;
        break;

      case 4:  // giây hàng chục (0-5)
        timeDigits[4] = (timeDigits[4] + 1) % 6;
        break;

      case 5:  // giây hàng đơn vị (0-9)
        timeDigits[5] = (timeDigits[5] + 1) % 10;
        break;
    }
  }

  if (chgPressed) {
    timeCursor = (timeCursor + 1) % 6;
  }

  if (selPressed) {
    int hh = timeDigits[0]*10 + timeDigits[1];
    int mm = timeDigits[2]*10 + timeDigits[3];
    int ss = timeDigits[4]*10 + timeDigits[5];

    targetTimeSec = hh * 3600 + mm * 60 + ss;

    // Chỉ cho phép chuyển state nếu tổng >=10 giây
    if (targetTimeSec >= 10) {
      currentState = STATE_SET_SENSITIVITY;
    }
  }

  // Luôn hiển thị đúng theo giá trị cài đặt, không ép min 10s
  drawSetTime(
    timeDigits[0], timeDigits[1], 
    timeDigits[2], timeDigits[3], 
    timeDigits[4], timeDigits[5], 
    timeCursor
  );
  break;

case STATE_SET_SENSITIVITY: {
  const char* opts[] = {"5%", "10%", "30%", "-Off"};
  const int numOpts = 4;
  const char* spaces = "  ";

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  char buf[32];
  snprintf(buf, sizeof(buf), "%s%s%s%s%s%s%s",
           opts[0], spaces, opts[1], spaces, opts[2], spaces, opts[3]);
  int totalWidth = u8g2.getStrWidth(buf);
  int x0 = (SCREEN_WIDTH - totalWidth) / 2;

  int fontH = u8g2.getMaxCharHeight();
  int y_text = (SCREEN_HEIGHT + fontH) / 2 - 2;
  int currentX = x0;

  for (int i = 0; i < numOpts; i++) {
    int optWidth = u8g2.getStrWidth(opts[i]);
    if (i == sensitivityIdx) {
      u8g2.drawRBox(currentX - 2, y_text - fontH - 1, optWidth + 4, fontH + 3, 2);
      u8g2.setDrawColor(0);
      u8g2.drawStr(currentX, y_text, opts[i]);
      u8g2.setDrawColor(1);
    } else {
      u8g2.drawStr(currentX, y_text, opts[i]);
    }
    currentX += optWidth + u8g2.getStrWidth(spaces);
  }

  u8g2.sendBuffer();

  // Thay đổi ở đây: dùng incPressed, chgPressed, selPressed như các state khác
  if (incPressed) {
    sensitivityIdx = (sensitivityIdx + 1) % numOpts;
  }

  if (chgPressed) {
    sensitivityIdx = (sensitivityIdx + numOpts - 1) % numOpts;
  }

  if (selPressed) {
    sensitivityOffMode = (sensitivityIdx == 3);
    thresholdUseHz = false;
    thresholdRateInt = 0;
    thresholdSecPerDrop = 0;
    enterRate();
    currentState = STATE_RATE;
  }
  break;
}
case STATE_RATE: {

  unsigned long now = millis();

  if (eventFlag) {
    eventFlag = false;

    if (lastEventProcessed != 0) {
      unsigned long currentGap = now - lastEventProcessed;

      if (currentGap > 50 && currentGap < 5000) {
        if (prevGap > 0) {
          float gapDiffRatio = abs((float)currentGap - (float)prevGap) / (float)prevGap;
          readyForRo = (gapDiffRatio <= 0.07f);
        } else {
          readyForRo = false;
        }

        prevGap = currentGap;

        float currentRo = 1000.0f / currentGap;
        if (smoothRo == 0.0f) smoothRo = currentRo;
        else smoothRo = alpha * smoothRo + (1 - alpha) * currentRo;
      }
    }
    lastEventProcessed = now;
    beepEvent();
  }

  // So sánh ngay cả khi chưa có event mới (gap dài bất thường)
  if (prevGap > 0 && (now - lastEventProcessed) > (prevGap * 1.07f)) {
    readyForRo = false;  // Gap mới đang kéo dài quá 7%
  }

  if (selPressed && readyForRo) {
    Ro_stable = smoothRo;
    setRunningParameters();
    Rn = Ro_stable;
    enterRunning(Rn); 
    currentState = STATE_RUNNING;
}
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  char buf[20];
  if (prevGap == 0) {
    strcpy(buf, "Waiting...");
  } else {
    if (smoothRo >= 1.0f) {
      snprintf(buf, sizeof(buf), "%s%.1fd/s", readyForRo ? "" : "?", smoothRo);
    } else {
      snprintf(buf, sizeof(buf), "%s%.1fs/d", readyForRo ? "" : "?", (1.0f / smoothRo));
    }
  }

  int textW = u8g2.getStrWidth(buf);
  int fontH = u8g2.getMaxCharHeight();
  int x = (SCREEN_WIDTH - textW) / 2;
  int y = (SCREEN_HEIGHT + fontH) / 2;

  u8g2.drawStr(x, y, buf);
  u8g2.sendBuffer();
}
break;
case STATE_RUNNING: {
    unsigned long now = millis();

    if (eventFlag) {
        eventFlag = false;
        eventCount++;

        unsigned long currentGap = (lastEventProcessed != 0) ? now - lastEventProcessed : 0;

        if (currentGap >= 50 && currentGap <= 5000) {
            if (prevGap != 0) {
                float gapDiffRatio = abs((float)currentGap - (float)prevGap) / prevGap;
                readyForRo = (gapDiffRatio <= 0.07f);
            } else {
                readyForRo = false;
            }

            prevGap = currentGap;

            float currentRn = 1000.0f / currentGap;
            smoothRo = alpha * smoothRo + (1 - alpha) * currentRn;

            if (readyForRo) {
                gapCount++; 
            } else {
                gapCount = 0;
            }

            if (gapCount >= 2) {
                Rn = smoothRo;
                gapCount = 0;
            }
        }

        lastEventProcessed = now;
        lastEventTimestamp = now;
        beepEvent();
    }

    // Chỉ kiểm tra ERR nếu KHÔNG ở chế độ Off (sensitivityOffMode=false)
    if (!sensitivityOffMode) {
        // ERR khi mất tín hiệu quá lâu
        if ((millis() - lastEventProcessed) > (3 * prevGap) && prevGap > 0 && (millis() - runningStartTime) > 1000) {
            errorCode = true;
            isBeepingErr = true;
        }

        // ERR khi lệch Ro_stable theo sensitivities
        if ((now - runningStartTime) > 1000) {
            if ((Rn < Ro_stable / sensitivities[sensitivityIdx]) || 
                (Rn > Ro_stable * sensitivities[sensitivityIdx])) {
                errorCode = true;
                isBeepingErr = true;
            }
        }
    } else {
        errorCode = false;
        isBeepingErr = false;
    }

    unsigned long elapsedMs = pausedElapsedMs + (now - rateStartTime);
    unsigned long elapsedSec = elapsedMs / 1000;
    unsigned long volNow = eventCount / ratioVal;

    if (!doneFlag) {
        if ((useVolumeSet && volNow >= setValueVol) ||
            (!useVolumeSet && elapsedSec >= setValueTime)) {
            doneFlag = true;
            isBeepingDone = true;
        }
    }

    if (now - blinkTimer >= 400) {
        blinkOn = !blinkOn;
        blinkTimer = now;
    }

    if (chgPressed) {
        unsigned long elapsedMs = pausedElapsedMs + (now - rateStartTime);
        enterPause(elapsedMs, eventCount);
        currentState = STATE_PAUSE;
    }

    handleBeeps();

    drawRunning(
        blinkOn, useVolumeSet,
        elapsedSec, setValueTime,
        volNow, setValueVol,
        Ro_stable, Rn
    );
}
break;

case STATE_PAUSE: {
    static unsigned long lastPauseGap = 0, prevPauseGap = 0, lastPauseEventMs = 0;
    static float smoothRnPause = 0.0f;
    static bool readyPauseRn = false;
    const float alphaPause = 0.7f;

    if (eventFlag) {
        eventFlag = false;
        unsigned long now = millis();
        unsigned long currentGap = now - lastPauseEventMs;

        if (currentGap > 50 && currentGap < 5000) {
            if (prevPauseGap > 0) {
                float gapDiffRatio = abs((float)currentGap - (float)prevPauseGap) / (float)prevPauseGap;
                readyPauseRn = (gapDiffRatio <= 0.07f);
            } else {
                readyPauseRn = false;
            }

            prevPauseGap = currentGap;
            float currentRnPause = 1000.0f / currentGap;
            if (smoothRnPause == 0.0f) smoothRnPause = currentRnPause;
            else smoothRnPause = alphaPause * smoothRnPause + (1 - alphaPause) * currentRnPause;
        }

        lastPauseEventMs = now;
        beepEvent();
    }

    if (prevPauseGap > 0 && (millis() - lastPauseEventMs) > (prevPauseGap * 1.07f))
        readyPauseRn = false;

    drawPause(Ro_stable, smoothRnPause, readyPauseRn);

   if (selPressed && readyPauseRn) {
    Rn_trong_pause = smoothRnPause;
    rateStartTime = millis();   
    enterRunning(Rn_trong_pause);
    currentState = STATE_RUNNING;
}
}
break;

  }
}