// ESP32-WROVER + DRV8833 機械錶自動上鍊盒
// v2：預設速度修正、開機/設定後 IDLE 緩衝、統一 analogWrite、Wi-Fi 路由只註冊一次。
// 注意：軟體無法保證 ESP32 進入 setup() 前的 GPIO15 啟動電位不造成瞬間輸入。

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ==========================================
// 硬體腳位定義
// ========================================== 
#define MOTOR_IN1    15
#define MOTOR_IN2    14
#define WIFI_SWITCH  13   // 實體開關（最高控制權）

// ==========================================
// 系統時間與穩定參數設定
// ==========================================
#define WIFI_STABILIZE_DELAY 100  // Wi-Fi 啟動後的電源回血緩衝時間（毫秒），可依實際狀況調整

// ==========================================
// 系統常數設定
// ==========================================
#define PWM_MIN      147  
#define PWM_MAX      232  
#define DEBOUNCE_TIME 50       // 實體開關軟體防彈跳時間 (毫秒)
#define IDLE_BUFFER_TIME 500UL // 馬達每次啟動前的靜止緩衝時間 (毫秒)

WebServer server(80);
Preferences preferences;

// ==========================================
// 系統設定參數結構
// ==========================================
struct Config {
  int mode = 1;             
  int speed = 40;           // 轉速百分比 0~100；預設採用 NH36 預設組合的 40%
  float runTime = 10.0;     
  float restTime = 30.0;    
  int wifiDuration = 5;     
  int autoWakeEnable = 0;   
  float autoWakeValue = 1.0;
  String autoWakeUnit = "min"; 
};

Config cfg;

// ==========================================
// 運行狀態變數
// ==========================================
unsigned long wifiStartTime = 0;
unsigned long wifiOffTime = 0;   
bool wifiActive = false;
bool wifiShutdownPending = false;
unsigned long wifiShutdownRequestTime = 0;

// 防彈跳與開關狀態
bool currentSwitchState = HIGH; 
bool lastFlickerState = HIGH;
unsigned long lastDebounceTime = 0;

// 開機延遲控制
bool isBooting = true;
unsigned long bootTimer = 0;

// 馬達狀態機變數
enum MotorState { STATE_IDLE, STATE_RUNNING, STATE_COASTING, STATE_RESTING };
MotorState motorState = STATE_IDLE;
int currentDir = 1; 
unsigned long motorTimer = 0;
unsigned long activeDuration = 0;
unsigned long restDuration = 0;
unsigned long coastTimer = 0; // 滑行緩衝計時器
unsigned long idleStartTime = 0; // IDLE 靜止緩衝起始時間

// 函式宣告
void loadConfig();
void saveConfigToFlash();
void setMotor(int speed, int dir);
void runWinderLogic();
void startAPMode();
void registerWebServerRoutes();
bool updateDebouncedSwitch();
void handlePhysicalSwitchChange();
void turnOffWiFi();
void handleRoot();
void handleSave();
void handleReset();
void handleTurnOffWifi();
void handleStatus(); // 新增：提供 JSON 狀態的端點
String getHTML();

void setup() {
  // 盡可能在 setup() 一開始就設定馬達輸出為停止。
  // 注意：這仍無法控制晶片進入 setup() 之前的硬體啟動階段。
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  setMotor(0, 0);

  pinMode(WIFI_SWITCH, INPUT_PULLUP);

  Serial.begin(115200);
  btStop();

  loadConfig();
  registerWebServerRoutes(); // 網頁路由只註冊一次，Wi-Fi 重啟時不再重複新增

  // 開機先強制關閉 Wi-Fi，確保啟動期間不開啟熱點
  WiFi.mode(WIFI_OFF);

  // 紀錄實體開關初始狀態，並啟動 2 秒開機保護計時器
  currentSwitchState = digitalRead(WIFI_SWITCH);
  lastFlickerState = currentSwitchState;
  lastDebounceTime = millis();
  bootTimer = millis();
  idleStartTime = bootTimer;
  motorTimer = bootTimer;
  motorState = STATE_IDLE;
  isBooting = true;

  Serial.println("【系統啟動】進入 2 秒電壓平穩保護期...");
}

void loop() {
  // ==========================================
  // 0. 開機保護：最少 2 秒不啟動 Wi-Fi 排程或馬達。
  //    期間仍持續讀取實體開關並完成防彈跳，避免沿用過期狀態。
  // ==========================================
  if (isBooting) {
    updateDebouncedSwitch();

    const unsigned long now = millis();
    const bool switchStable = (now - lastDebounceTime) >= DEBOUNCE_TIME;
    if ((now - bootTimer) >= 2000UL && switchStable) {
      isBooting = false;

      // 開機保護解除後，額外維持 500ms 停止狀態。
      idleStartTime = now;
      motorTimer = now;
      motorState = STATE_IDLE;

      Serial.println("【系統自動】開機保護解除，檢查總開關狀態...");
      if (currentSwitchState == LOW) {
        Serial.println(" -> 總開關為 ON，安全啟動 Wi-Fi！");
        startAPMode();
      } else {
        // 開機時開關為 OFF：清除排程起點，避免意外自動喚醒。
        wifiOffTime = 0;
        Serial.println(" -> 總開關為 OFF，維持 Wi-Fi 關閉與排程鎖定。");
      }
    }

    // 保護期的最後一輪也不執行馬達狀態機。
    return;
  }

  // ==========================================
  // 1. 實體開關優先控制權（50ms 防彈跳）
  // ==========================================
  if (updateDebouncedSwitch()) {
    handlePhysicalSwitchChange();
  }

  // ==========================================
  // 2. Wi-Fi 電源管理與自動排程喚醒邏輯
  // ==========================================
  if (wifiActive) {
    server.handleClient();

    // 網頁「關閉 Wi-Fi」回應送出後，使用非阻塞計時等待 500ms。
    // 不用 delay()，避免馬達在等待期間持續輸出而超過設定運轉時間。
    if (wifiActive && wifiShutdownPending &&
        (millis() - wifiShutdownRequestTime) >= 500UL) {
      turnOffWiFi();
    }

    if (wifiActive) {
      unsigned long elapsedWifiTime = (millis() - wifiStartTime) / 1000UL;
      if (elapsedWifiTime >= (unsigned long)(cfg.wifiDuration * 60)) {
        Serial.println("【系統自動】Wi-Fi 保持開啟的倒數時間已到！執行關閉以省電。");
        turnOffWiFi();
      }
    }
  } else {
    // 只有在開關是 ON，且不是被 OFF 鎖死的狀態下，才允許自動喚醒
    if (cfg.autoWakeEnable == 1 && wifiOffTime > 0 && currentSwitchState == LOW) {
      
      unsigned long offDuration = millis() - wifiOffTime;
      unsigned long wakeTargetMs = 0;
      
      if (cfg.autoWakeUnit == "min") {
        wakeTargetMs = (unsigned long)(cfg.autoWakeValue * 60000.0);
      } else {
        wakeTargetMs = (unsigned long)(cfg.autoWakeValue * 3600000.0);
      }

      if (offDuration >= wakeTargetMs) {
        Serial.println("【系統自動】休眠排程時間已滿！自動喚醒 Wi-Fi。");
        startAPMode();
      }
    }
  }

  // ==========================================
  // 3. 執行自動上鍊盒核心馬達邏輯
  // ==========================================
  runWinderLogic();
}

// ==========================================
// Wi-Fi 與伺服器控制區塊
// ==========================================
void registerWebServerRoutes() {
  // 路由只在 setup() 註冊一次；server.stop() 不會移除這些路由。
  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/reset", HTTP_POST, handleReset);
  server.on("/off", HTTP_GET, handleTurnOffWifi);
  server.on("/status", HTTP_GET, handleStatus);
}

bool updateDebouncedSwitch() {
  const unsigned long now = millis();
  const bool reading = digitalRead(WIFI_SWITCH);

  if (reading != lastFlickerState) {
    lastFlickerState = reading;
    lastDebounceTime = now;
  }

  if ((now - lastDebounceTime) >= DEBOUNCE_TIME &&
      reading != currentSwitchState) {
    currentSwitchState = reading;
    return true;
  }
  return false;
}

void handlePhysicalSwitchChange() {
  if (currentSwitchState == LOW) {
    // ON：允許 Wi-Fi；若已開啟則重置維持時間。
    Serial.println("【實體開關】切換至 ON！(允許 Wi-Fi)");
    wifiOffTime = 0;
    if (!wifiActive) {
      Serial.println("  -> 執行強制啟動 Wi-Fi...");
      startAPMode();
    } else {
      wifiStartTime = millis();
      Serial.println("  -> Wi-Fi 已在運作，重置倒數計時器。");
    }
  } else {
    // OFF：絕對鎖住自動喚醒。必須在 turnOffWiFi() 後清零，
    // 因為 turnOffWiFi() 本身會記錄自動排程的關閉起點。
    Serial.println("【實體開關】切換至 OFF！(鎖死排程)");
    if (wifiActive) {
      Serial.println(" -> 執行強制關閉 Wi-Fi");
      turnOffWiFi();
    }
    wifiOffTime = 0;
  }
}

void startAPMode() {
  if (wifiActive) {
    // 防止未來新增呼叫路徑時，重複啟動 AP 或重設伺服器。
    wifiStartTime = millis();
    return;
  }

  // 只有在馬達處於靜止狀態時，才強制壓制腳位，避免乾擾轉動中的馬達
  if (motorState == STATE_IDLE || motorState == STATE_RESTING || motorState == STATE_COASTING) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
  }

  // 以較低功率啟動 Wi-Fi，躲避射頻天線冷啟動的湧浪電流
  WiFi.setTxPower(WIFI_POWER_8_5dBm); 

  const bool modeStarted = WiFi.mode(WIFI_AP);
  if (!modeStarted || !WiFi.softAP("Watch_Winder_AP")) {
    WiFi.mode(WIFI_OFF);
    wifiActive = false;
    wifiOffTime = millis();
    Serial.println("【錯誤】Wi-Fi 熱點啟動失敗，已保持關閉狀態。");
    return;
  }

  // 【改用參數】：讓電源稍微平穩，時間長度可由上方 #define 自行調控
  delay(WIFI_STABILIZE_DELAY);

  // 天線穩定後，將發射功率拉回最高 (19.5 dBm)，確保最強穿牆收訊
  WiFi.setTxPower(WIFI_POWER_19_5dBm); 

  // 路由已在 setup() 註冊，這裡只啟動 HTTP 伺服器。
  server.begin();
  wifiActive = true;
  wifiStartTime = millis();
  wifiOffTime = 0;

  Serial.println("  => [狀態] Wi-Fi 熱點已上線: Watch_Winder_AP (已恢復最高功率)");
}

void turnOffWiFi() {
  if (!wifiActive) {
    wifiShutdownPending = false;
    return;
  }

  server.stop();
  WiFi.mode(WIFI_OFF);
  wifiActive = false;
  wifiShutdownPending = false;
  // 記錄關閉時間，供自動喚醒排程使用；實體 OFF 會再清除此值。
  wifiOffTime = millis();
  Serial.println("  => [狀態] Wi-Fi 模組已關閉。");
}

// ==========================================
// 快閃記憶體 (Preferences) 存取區塊
// ==========================================
void loadConfig() {
  preferences.begin("winder", true); 
  cfg.mode = preferences.getInt("mode", 1);
  cfg.speed = preferences.getInt("speed", 40);
  // 舊版預設值可能是 200；若 Flash 值超出新介面的 0~100 範圍，安全回到 40%。
  if (cfg.speed < 0 || cfg.speed > 100) {
    cfg.speed = 40;
    Serial.println("【系統提醒】偵測到無效轉速設定，已在本次執行改用 40%。");
  }
  cfg.runTime = preferences.getFloat("runTF", 10.0);
  cfg.restTime = preferences.getFloat("restTF", 30.0);
  cfg.wifiDuration = preferences.getInt("wifiDur", 5); 
  cfg.autoWakeEnable = preferences.getInt("wakeEn", 0);
  cfg.autoWakeValue = preferences.getFloat("wakeVal", 1.0);
  cfg.autoWakeUnit = preferences.getString("wakeUnit", "min");
  preferences.end();
  Serial.println("【系統記錄】已從 Flash 讀取上次儲存的設定。");
}

void saveConfigToFlash() {
  preferences.begin("winder", false); 
  preferences.putInt("mode", cfg.mode);
  preferences.putInt("speed", cfg.speed);
  preferences.putFloat("runTF", cfg.runTime);
  preferences.putFloat("restTF", cfg.restTime);
  preferences.putInt("wifiDur", cfg.wifiDuration); 
  preferences.putInt("wakeEn", cfg.autoWakeEnable);
  preferences.putFloat("wakeVal", cfg.autoWakeValue);
  preferences.putString("wakeUnit", cfg.autoWakeUnit);
  preferences.end();
  Serial.println("【系統記錄】設定已成功存入 Flash！");
}

// ==========================================
// 馬達驅動與保護邏輯區塊
// ==========================================
// ==========================================
// 馬達驅動與保護邏輯區塊
// ==========================================
void setMotor(int pwmVal, int dir) {
  // 1. 【新增狀態鎖】：記憶上次的指令
  static int lastPwm = -1;
  static int lastDir = -1;

  // 如果數值沒變，直接結束，絕不讓 loop() 瘋狂重置底層 PWM 計時器
  if (pwmVal == lastPwm && dir == lastDir) {
    return; 
  }

  // 更新記憶體狀態
  lastPwm = pwmVal;
  lastDir = dir;

  pwmVal = constrain(pwmVal, 0, 255);

  // 2. 【強制壓制 0V】：切換狀態時，用 pinMode 與 digitalWrite 強行破解 GPIO 15 的上拉干擾
  if (pwmVal == 0 || dir == 0) {
    // 雙腳位都強制解除 PWM 綁定，死死釘在 0V
    pinMode(MOTOR_IN1, OUTPUT);
    digitalWrite(MOTOR_IN1, LOW);
    pinMode(MOTOR_IN2, OUTPUT);
    digitalWrite(MOTOR_IN2, LOW);
  } else if (dir > 0) {
    // 順時針：IN2 強制壓在 0V (防止電壓逆流)，IN1 正常發送 PWM
    pinMode(MOTOR_IN2, OUTPUT);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_IN1, pwmVal);
  } else {
    // 逆時針：IN1 (GPIO 15) 強制壓在 0V (破解 3.3V 漂移)，IN2 正常發送 PWM
    pinMode(MOTOR_IN1, OUTPUT);
    digitalWrite(MOTOR_IN1, LOW);
    analogWrite(MOTOR_IN2, pwmVal);
  }
}

void runWinderLogic() {
  unsigned long currentMillis = millis();
  int safeSpeed = constrain(cfg.speed, 0, 100);
  int mappedPWM = map(safeSpeed, 0, 100, PWM_MIN, PWM_MAX);

  switch (motorState) {
    case STATE_IDLE:
      setMotor(0, 0);

      // IDLE 狀態至少維持 500ms，不會在進入 IDLE 的同一輪立刻啟動馬達。
      if (currentMillis - idleStartTime < IDLE_BUFFER_TIME) {
        break;
      }

      if (cfg.mode == 3) {
        activeDuration = (unsigned long)((cfg.runTime * 1000.0) / 2.0);
        currentDir = 1;
      } else {
        activeDuration = (unsigned long)(cfg.runTime * 1000.0);
        currentDir = (cfg.mode == 1) ? 1 : -1;
      }
      motorTimer = currentMillis;
      motorState = STATE_RUNNING;
      break;

    case STATE_RUNNING:
      setMotor(mappedPWM, currentDir);
      if (currentMillis - motorTimer >= activeDuration) {
        // DRV8833 的 00 狀態是 Coast；先滑行 1.5 秒，再換向或休息。
        setMotor(0, 0);
        coastTimer = currentMillis;
        motorState = STATE_COASTING;
      }
      break;

    case STATE_COASTING:
      // 等待機構自然減速；此狀態不是 DRV8833 的主動電氣煞車。
      if (currentMillis - coastTimer >= 1500UL) {
        if (cfg.mode == 3 && currentDir == 1) {
          currentDir = -1;
          motorTimer = currentMillis;
          motorState = STATE_RUNNING;
        } else {
          motorTimer = currentMillis;
          restDuration = (unsigned long)(cfg.restTime * 1000.0);
          motorState = STATE_RESTING;
        }
      }
      break;

    case STATE_RESTING:
      setMotor(0, 0);
      if (currentMillis - motorTimer >= restDuration) {
        motorState = STATE_IDLE;
        // 已經完整經過設定的休止時間，因此不再額外增加 500ms，避免改變 TPD 計算。
        idleStartTime = currentMillis - IDLE_BUFFER_TIME;
        motorTimer = currentMillis;
      }
      break;
  }
}

// ==========================================
// 網頁 API 端點處理區塊
// ==========================================
void handleRoot() {
  server.send(200, "text/html", getHTML());
}

// 新增：吐出馬達即時狀態的 JSON
void handleStatus() {
  unsigned long elapsed = millis() - motorTimer;
  unsigned long remain = 0;
  String st = "";
  
  if (motorState == STATE_RUNNING) {
    st = "RUNNING";
    remain = (activeDuration > elapsed) ? (activeDuration - elapsed) / 1000 : 0;
  } else if (motorState == STATE_COASTING) {
    st = "COASTING";
    // 滑行緩衝期間不倒數
  } else if (motorState == STATE_RESTING) {
    st = "RESTING";
    remain = (restDuration > elapsed) ? (restDuration - elapsed) / 1000 : 0;
  } else {
    st = "IDLE";
  }

  // 組裝極簡 JSON：{"s":"RUNNING","d":1,"r":14,"m":3}
  // s=狀態, d=方向(1左/-1右), r=剩餘秒數, m=模式(1順/2逆/3雙向)
  String json = "{\"s\":\"" + st + "\",\"d\":" + String(currentDir) + ",\"r\":" + String(remain) + ",\"m\":" + String(cfg.mode) + "}";
  server.send(200, "application/json", json);
}

void handleSave() {
  Serial.println("【網頁指令】收到使用者的 [儲存設定] 請求");
  int oldWifiDuration = cfg.wifiDuration;

  if (server.hasArg("mode")) cfg.mode = server.arg("mode").toInt();
  if (server.hasArg("speed")) cfg.speed = server.arg("speed").toInt();
  if (server.hasArg("runTime")) cfg.runTime = server.arg("runTime").toFloat();
  if (server.hasArg("restTime")) cfg.restTime = server.arg("restTime").toFloat();
  if (server.hasArg("wifiDuration")) cfg.wifiDuration = server.arg("wifiDuration").toInt();
  if (server.hasArg("autoWakeEnable")) cfg.autoWakeEnable = server.arg("autoWakeEnable").toInt();
  if (server.hasArg("autoWakeValue")) cfg.autoWakeValue = server.arg("autoWakeValue").toFloat();
  if (server.hasArg("autoWakeUnit")) cfg.autoWakeUnit = server.arg("autoWakeUnit");

  saveConfigToFlash();

  // 將馬達拉回 IDLE，套用新設定前先維持 500ms 停止緩衝。
  setMotor(0, 0);
  motorState = STATE_IDLE;
  idleStartTime = millis();
  motorTimer = idleStartTime;

  if (oldWifiDuration != cfg.wifiDuration) {
    wifiStartTime = millis();
    Serial.println("  -> Wi-Fi 保持時間已變更，重新計算倒數！");
  }

  server.send(200, "text/plain", "OK");
}

void handleReset() {
  Serial.println("【網頁指令】收到使用者的 [重置設定] 請求");
  loadConfig();
  
  // 【優化】：重置設定後，順便幫使用者把 Wi-Fi 倒數計時也重置，避免網頁重整時斷線
  wifiStartTime = millis(); 
  Serial.println("  -> 設定已重置，並重新給予完整的 Wi-Fi 連線時間。");
  
  // 同樣拉回 IDLE，並重設 500ms 靜止緩衝計時。
  setMotor(0, 0);
  motorState = STATE_IDLE;
  idleStartTime = millis();
  motorTimer = idleStartTime; 

  server.send(200, "text/plain", "OK");
}

void handleTurnOffWifi() {
  Serial.println("【網頁指令】收到使用者的 [手動關閉 Wi-Fi] 請求！");
  String exitHtml = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  exitHtml += "<title>Wi-Fi 關閉</title><style>body{text-align:center;font-family:Arial,sans-serif;padding-top:50px;background:#f4f4f9;color:#333;} h2{color:#dc3545;}</style></head>";
  exitHtml += "<body><h2>Wi-Fi 已成功關閉</h2><p>系統已進入零干擾省電模式。<br><br>您可以直接關閉此瀏覽器分頁。</p></body></html>";
  
  server.send(200, "text/html", exitHtml);
  // 等待 HTTP 回應送出後再關閉熱點；採非阻塞方式，不凍結馬達狀態機。
  wifiShutdownPending = true;
  wifiShutdownRequestTime = millis();
}

// ==========================================
// HTML 前端介面生成
// ==========================================
String getHTML() {
  // 此處以相同的有號型別計算，避免秒數比較的 signed/unsigned 警告。
  long elapsedSec = (long)((millis() - wifiStartTime) / 1000UL);
  long totalSecAllowed = (long)cfg.wifiDuration * 60L;
  long remainSec = (totalSecAllowed > elapsedSec) ? (totalSecAllowed - elapsedSec) : 0L;
  int remMin = (int)(remainSec / 60L);
  int remS = (int)(remainSec % 60L);

  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>自動上鍊盒控制系統</title>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; background: #f4f4f9; margin: 0; padding: 20px; color: #333; }";
  html += ".container { max-width: 450px; background: #fff; padding: 20px; margin: auto; border-radius: 10px; box-shadow: 0 4px 10px rgba(0,0,0,0.1); }";
  html += "h2 { text-align: center; color: #007BFF; margin-bottom: 5px; }";
  html += ".timer-box { text-align: center; background: #e9ecef; padding: 8px; border-radius: 5px; font-weight: bold; margin-bottom: 10px; color: #d9534f; }";
  html += ".status-box { text-align: center; background: #e9ecef; padding: 8px; border-radius: 5px; font-weight: bold; margin-bottom: 20px; color: #333; font-size: 15px; }";
  html += ".section { margin-bottom: 18px; padding-bottom: 12px; border-bottom: 1px solid #eee; }";
  html += "label { display: block; font-weight: bold; margin-bottom: 6px; }";
  html += "input[type='number'] { width: 90px; padding: 6px; font-size: 16px; }";
  html += "input[type='range'] { width: 100%; cursor: pointer; }";
  html += ".btn-group { display: flex; gap: 10px; margin-top: 20px; }";
  html += "button { flex: 1; padding: 10px; font-size: 16px; font-weight: bold; border: none; border-radius: 5px; cursor: pointer; color: #fff; }";
  html += ".btn-save { background: #28a745; }";
  html += ".btn-reset { background: #ffc107; color: #333; }";
  html += ".btn-off { background: #dc3545; }";
  html += ".sub-row { margin-top: 10px; padding-left: 15px; border-left: 3px solid #007BFF; display: none; }";
  html += "</style></head><body>";
  
  html += "<div class='container'>";
  html += "<h2>自動上鍊盒控制系統</h2>";
  
  // 原有的 Wi-Fi 倒數計時框
  html += "<div class='timer-box'>Wi-Fi 將於 <span id='m'>" + String(remMin) + "</span> 分 <span id='s'>" + String(remS) + "</span> 秒後關閉</div>";
  
  // 新增：AJAX 動態狀態儀表板 (極致單行對稱版)
  html += "<div class='status-box' id='motorStatus'>連線中...</div>";

  html += "<form id='winderForm'>";

  // 機芯一鍵設定
  html += "<div class='btn-group' style='margin-top:0; margin-bottom:20px;'>";
  html += "<button type='button' style='background:#17a2b8;' onclick='preset(\"nh36\")'>Seiko NH36</button>";
  html += "<button type='button' style='background:#6c757d;' onclick='preset(\"eta2824\")'>ETA 2824</button>";
  html += "</div>";
  
  html += "<div class='section'><label>馬達運轉模式：</label>";
  html += "<label><input type='radio' name='mode' value='1' " + String(cfg.mode == 1 ? "checked" : "") + "> 僅順時針</label>";
  html += "<label><input type='radio' name='mode' value='2' " + String(cfg.mode == 2 ? "checked" : "") + "> 僅逆時針</label>";
  html += "<label><input type='radio' name='mode' value='3' " + String(cfg.mode == 3 ? "checked" : "") + "> 雙向交替 (滑行緩衝後換向)</label></div>";

  int displaySpeed = cfg.speed > 100 ? 100 : cfg.speed; 
  html += "<div class='section'><label>馬達轉速調整 (<span id='speedVal'>" + String(displaySpeed) + "</span>%)：</label>";
  html += "<input type='range' name='speed' min='0' max='100' value='" + String(displaySpeed) + "' oninput='document.getElementById(\"speedVal\").innerText=this.value'></div>";

  html += "<div class='section'><label>單次運轉時間 (秒)：</label>";
  html += "<input type='number' name='runTime' min='0.1' max='300' step='0.01' value='" + String(cfg.runTime) + "'></div>";

  html += "<div class='section'><label>循環休止時間 (秒)：</label>";
  html += "<input type='number' name='restTime' min='0.1' max='3600' step='0.01' value='" + String(cfg.restTime) + "'></div>";

  html += "<div class='section'><label>Wi-Fi 保持開啟時間 (<span id='wifiDurVal'>" + String(cfg.wifiDuration) + "</span> 分鐘)：</label>";
  html += "<input type='range' name='wifiDuration' min='1' max='10' value='" + String(cfg.wifiDuration) + "' oninput='document.getElementById(\"wifiDurVal\").innerText=this.value'></div>";

  html += "<div class='section'><label>Wi-Fi 自動排程喚醒：</label>";
  html += "<label><input type='radio' name='autoWakeEnable' value='0' " + String(cfg.autoWakeEnable == 0 ? "checked" : "") + " onclick='toggleWake(0)'> 否 (僅靠實體開關)</label>";
  html += "<label><input type='radio' name='autoWakeEnable' value='1' " + String(cfg.autoWakeEnable == 1 ? "checked" : "") + " onclick='toggleWake(1)'> 是 (定時自動開啟)</label>";
  
  html += "<div id='wakeSubRow' class='sub-row' style='display:" + String(cfg.autoWakeEnable == 1 ? "block" : "none") + ";'>";
  html += "<input type='number' step='0.01' name='autoWakeValue' value='" + String(cfg.autoWakeValue) + "' style='width:80px;'> ";
  html += "<label style='display:inline;'><input type='radio' name='autoWakeUnit' value='min' " + String(cfg.autoWakeUnit == "min" ? "checked" : "") + "> 分</label> ";
  html += "<label style='display:inline;'><input type='radio' name='autoWakeUnit' value='hr' " + String(cfg.autoWakeUnit == "hr" ? "checked" : "") + "> 時</label> </div></div>";

  html += "<div class='btn-group'>";
  html += "<button type='button' class='btn-save' onclick='sendAction(\"/save\")'>儲存</button>";
  html += "<button type='button' class='btn-reset' onclick='sendAction(\"/reset\")'>重置</button>";
  html += "<button type='button' class='btn-off' onclick='if(confirm(\"確定要關閉 Wi-Fi 嗎？\")) location.href=\"/off\";'>關閉 Wi-Fi</button>";
  html += "</div>";

  html += "</form></div>";

  html += "<script>";
  html += "function toggleWake(en) { document.getElementById('wakeSubRow').style.display = en === 1 ? 'block' : 'none'; }";
  
  html += "function preset(type) {";
  html += "  if(type === 'nh36') {";
  html += "    document.querySelector('input[name=\"mode\"][value=\"3\"]').checked = true;";
  html += "    document.querySelector('input[name=\"speed\"]').value = 40;";
  html += "    document.getElementById('speedVal').innerText = 40;";
  html += "    document.querySelector('input[name=\"runTime\"]').value = '27.54';";
  html += "    document.querySelector('input[name=\"restTime\"]').value = '618.96';"; 
  html += "    alert('【Seiko NH36 (約 800 TPD 預設)】\\n建議轉速 40%\\n單次運轉 27.54 秒 (每方向約 3 圈)\\n休止時間 618.96 秒；實際 TPD 受實測轉速影響');";
  html += "  } else if(type === 'eta2824') {";
  html += "    document.querySelector('input[name=\"mode\"][value=\"3\"]').checked = true;";
  html += "    document.querySelector('input[name=\"speed\"]').value = 70;";
  html += "    document.getElementById('speedVal').innerText = 70;";
  html += "    document.querySelector('input[name=\"runTime\"]').value = '24.54';";
  html += "    document.querySelector('input[name=\"restTime\"]').value = '621.96';"; 
  html += "    alert('【ETA 2824 (約 800 TPD 預設)】\\n建議轉速 70%\\n單次運轉 24.54 秒 (每方向約 3 圈)\\n休止時間 621.96 秒；實際 TPD 受實測轉速影響');";
  html += "  }";
  html += "}";

  html += "function sendAction(url) {";
  html += "  let formData = new URLSearchParams(new FormData(document.getElementById('winderForm'))).toString();";
  html += "  fetch(url, {";
  html += "    method: 'POST',";
  html += "    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },"; 
  html += "    body: formData";
  html += "  }).then(res => {";
  html += "    if(res.ok) { alert('操作成功！'); location.reload(); } else { alert('操作失敗'); }";
  html += "  });";
  html += "}";

  // 原有 Wi-Fi 倒數計時邏輯
  html += "let rSec = " + String(remainSec) + ";";
  html += "setInterval(() => {";
  html += "  if (rSec > 0) {";
  html += "    rSec--;";
  html += "    let m = Math.floor(rSec / 60);";
  html += "    let s = rSec % 60;";
  html += "    let mEl = document.getElementById('m');";
  html += "    let sEl = document.getElementById('s');";
  html += "    if(mEl) mEl.innerText = m;";
  html += "    if(sEl) sEl.innerText = s;";
  html += "  }";
  html += "}, 1000);";

  // 新增：AJAX 每秒抓取馬達狀態並更新 UI
  html += "function formatTime(secs) {";
  html += "  if(secs < 60) return secs + '秒';";
  html += "  let m = Math.floor(secs / 60); let s = secs % 60;";
  html += "  return (m < 10 ? '0' + m : m) + ':' + (s < 10 ? '0' + s : s);";
  html += "}";
  
  html += "setInterval(() => {";
  html += "  fetch('/status').then(r => r.json()).then(data => {";
  html += "    let box = document.getElementById('motorStatus');";
  html += "    let modeText = (data.m == 1) ? '[順向]' : (data.m == 2) ? '[逆向]' : '[雙向]';";
  html += "    let dirText = (data.d == 1) ? '右旋' : '左旋';"; // d=1/-1 為程式定義的方向，實際左右旋以機構校正為準
  html += "    if(data.s === 'RUNNING') {";
  html += "      box.innerHTML = '🟢 ' + modeText + ' ' + dirText + ' ⏳ ' + data.r + '秒';";
  html += "      box.style.color = '#155724';"; // 暗綠色
  html += "    } else if(data.s === 'COASTING') {";
  html += "      box.innerHTML = '↘️ [滑行減速] 緩衝中...';";
  html += "      box.style.color = '#856404';"; // 暗黃色
  html += "    } else if(data.s === 'RESTING') {";
  html += "      box.innerHTML = '🌙 [休止] ⏳ ' + formatTime(data.r);";
  html += "      box.style.color = '#004085';"; // 暗藍色
  html += "    } else {";
  html += "      box.innerHTML = '⚪ [待命] 準備啟動';";
  html += "      box.style.color = '#333';";
  html += "    }";
  html += "  }).catch(e => {";
  html += "    document.getElementById('motorStatus').innerText = '🔴 連線中斷...';";
  html += "  });";
  html += "}, 1000);"; // 每秒更新一次

  html += "</script>";

  html += "</body></html>";
  return html;
}