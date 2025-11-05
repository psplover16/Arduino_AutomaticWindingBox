// 自動上鍊盒 Arduino 程式碼
// 目標：一天轉動約 1086 圈

// 定義常數
const float SECONDS_PER_ROTATION = 6.2;  // 馬達6.2秒轉1圈
const unsigned int HALF_ROTATION_PAUSE = 800; // 半圈間休息 (毫秒)
const unsigned int SHORT_REST_CYCLE = 10;     // 每 10 次循環休息
const unsigned int SHORT_REST_TIME = 780000;  // 短暫休息 780 秒 (13分鐘)

void setup() {
  Serial.begin(9600);
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A4, INPUT_PULLUP); // 使用內建上拉電阻，預設接收為 HIGH

  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A5, LOW);
  
  Serial.println("手錶盒系統啟動");
  Serial.println("目標: 一天約1086圈");
  Serial.println("開關邏輯已對調");
}

void loop() {
  int a4State = digitalRead(A4);
  Serial.print("A4狀態: ");
  Serial.println(a4State);

  if (a4State == HIGH) {
    // 混合模式：半圈+短暫休息，週期性短暫休息
    Serial.println("混合模式：半圈+短暫休息，週期性短暫休息");
  static int cycleCount = 0;
    // 順時針半圈
    rotateHalf(true);
    if (digitalRead(A4) == LOW) { stopMotor(); return; }
    delayWithCheck(HALF_ROTATION_PAUSE);
    // 逆時針半圈
    rotateHalf(false);
    if (digitalRead(A4) == LOW) { stopMotor(); return; }
    delayWithCheck(HALF_ROTATION_PAUSE);
    cycleCount++;
    // 每 SHORT_REST_CYCLE 次循環休息 SHORT_REST_TIME 毫秒
    if (cycleCount >= SHORT_REST_CYCLE) {
      Serial.print("短暫休息 ");
      Serial.print(SHORT_REST_TIME/1000);
      Serial.println(" 秒");
      stopMotor();
      if (!waitWithCheck(SHORT_REST_TIME)) return;
      cycleCount = 0;
    }
  } else {
    // 高強度模式：不停交替半圈+0.5秒休息
    Serial.println("高速模式：不停交替半圈+0.5秒休息");
    rotateHalf(true);
    if (digitalRead(A4) == HIGH) { stopMotor(); return; }
    delayWithCheck(HALF_ROTATION_PAUSE);
    rotateHalf(false);
    if (digitalRead(A4) == HIGH) { stopMotor(); return; }
    delayWithCheck(HALF_ROTATION_PAUSE);
  }
  delay(10);
}


// 新增：半圈旋轉函式
void rotateHalf(bool clockwise) {
  unsigned long halfDuration = (SECONDS_PER_ROTATION * 1000) / 2;
  unsigned long startTime = millis();
  int currentState = digitalRead(A4);
  if (clockwise) {
    Serial.println("順時針半圈");
    while (millis() - startTime < halfDuration && digitalRead(A4) == currentState) {
      digitalWrite(A0, LOW);
      digitalWrite(A1, HIGH);
      delay(50);
    }
  } else {
    Serial.println("逆時針半圈");
    while (millis() - startTime < halfDuration && digitalRead(A4) == currentState) {
      digitalWrite(A0, HIGH);
      digitalWrite(A1, LOW);
      delay(50);
    }
  }
  stopMotor();
}

void counterclockwiseRotations(int rotations) {
  unsigned long totalDuration = rotations * SECONDS_PER_ROTATION * 1000;
  unsigned long startTime = millis();
  int currentState = digitalRead(A4); // 記錄進入函數時的開關狀態
  
  Serial.print("逆時針轉動 ");
  Serial.print(rotations);
  Serial.println(" 圈");
  
  // 轉動期間監控開關狀態變化
  while (millis() - startTime < totalDuration && digitalRead(A4) == currentState) {
    digitalWrite(A0, HIGH);  // 逆時針
    digitalWrite(A1, LOW);
    delay(200);
  }
  stopMotor();
  Serial.println("逆時針轉動完成");
}


// 休息期間持續檢查 A4 狀態
bool waitWithCheck(unsigned long duration) {
  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0;
  while (millis() - startTime < duration) {
    unsigned long remaining = (duration - (millis() - startTime)) / 1000;
    if (millis() - lastPrintTime >= 1000) {
      Serial.print("剩餘時間: ");
      Serial.print(remaining);
      Serial.println(" 秒");
      lastPrintTime = millis();
    }
    digitalWrite(A0, LOW);
    digitalWrite(A1, LOW);
    if (digitalRead(A4) == LOW) {
      Serial.println("切換到高速模式");
      stopMotor();
      return false;
    }
    delay(100);
  }
  Serial.println("休息完成");
  return true;
}

// 0.5秒休息期間持續檢查 A4 狀態變化
void delayWithCheck(unsigned long ms) {
  unsigned long startTime = millis();
  int initialState = digitalRead(A4); // 記錄初始狀態
  while (millis() - startTime < ms) {
    digitalWrite(A0, LOW);
    digitalWrite(A1, LOW);
    // 如果狀態改變,立即返回
    if (digitalRead(A4) != initialState) return;
    delay(10);
  }
}

void stopMotor() {
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
}

/*
程式說明：
- 目標：一天轉動約 1000 圈
- 單次循環：1圈轉動(6.2秒) + 0.8秒休息 + 0.8秒休息 = 7.8秒/圈
- 每 10 圈休息 780 秒 (13分鐘)
- 一天循環次數：100 次 (1000圈 ÷ 10圈)
- 總時間：(10圈 × 7.8秒 + 780秒) × 100 = 86400秒 = 24小時
- 總圈數：100 × 10 = 1000 圈

控制邏輯：
- 開關關閉(A4=HIGH)：正常模式，一天1000圈
- 開關打開(A4=LOW)：高速模式，連續轉動無休息

適用機芯：NH35A (建議800-1200圈/天)
*/