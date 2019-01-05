#include <ArduinoJson.h>
#include <M5Stack.h>
#include <SparkFunMPU9250-DMP.h>
#include <ThingsBoard.h>
#include <Ticker.h>
#include <vector>
#include <WiFi.h>

#include <drawPulse.h>
#include <MyPulseSensorPlayground.h>

#include "myconfig.h"

const int PIN_INPUT = 36;
const int THRESHOLD = 2200;   // Adjust this number to avoid noise when idle

const int PIN_GSR = 35;
const int MAX_GSR = 2047;
const int FS_GSR = 1;

const int SEND_INTERVAL_MS = 60000;


PulseSensorPlayground pulseSensor;
MPU9250_DMP imu;

WiFiClient espClient;
ThingsBoard tb(espClient);
DrawPulse drawPulse;

Ticker tickerGSR; // GSRセンサの値を読む
Ticker tickerSend; //送信のタイマ



std::vector<int> ibiBuff;
std::vector<int> gsrBuff;

volatile bool sendFlg = false;
volatile int steps = 0; // 歩数


void setup() {
    M5.begin();
    dacWrite(25, 0); // Speaker OFF

    initWiFi();
    initPulseSensor();
    initImu();

    // 可変バッファの初期容量を確保
    gsrBuff.reserve(64);
    ibiBuff.reserve(256);


    drawPulse.init();

    // 位置情報用タスクのセット
    xTaskCreatePinnedToCore(
                    taskGeo,     /* Function to implement the task */
                    "taskGeo",   /* Name of the task */
                    4096,      /* Stack size in words */
                    NULL,      /* Task input parameter */
                    1,         /* Priority of the task */
                    NULL,      /* Task handle. */
                    0);        /* Core where the task should run */
    
    // タイマーセット
    tickerGSR.attach_ms(FS_GSR * 1000, _readGSR);
    tickerSend.attach_ms(SEND_INTERVAL_MS, _sendFlgUp);
}

unsigned long pedLastStepCount = 0;

// メインループ
// 重い処理をしたら早めにreturnしておく
void loop() {
    if (pulseSensor.sawStartOfBeat()) {  // 心拍発見時
        int ibi = pulseSensor.getInterBeatIntervalMs();
        ibiBuff.push_back(ibi);
        
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.setTextSize(3);
        M5.Lcd.printf("BPM: %03d", 60000 /ibi);
    }

    keepTbConn();

    // タイマーによってフラグがあげられているとき
    // ブロック内の計測時間: 5m秒
    if (sendFlg) {
        //unsigned long startt = millis();
        sendFlg = false;
        int n = ibiBuff.size();
        
        //10拍も取れていないー＞計算をあきらめる
        if(n < 10){
            gsrBuff.clear();
            ibiBuff.clear();
            return;
        }
        
        //RMSSD(RR間隔の差の自乗平均平方根)の計算
        //と同時に平均心拍数も計算

        int diffCount = 0;
        int sum = 0;
        int ibiSum = 0;
        for(int i = 1;i < n; i++) {
            int a = ibiBuff[i-1];
            int b = ibiBuff[i];

            //差が20%以下のみ使用　https://www.hrv4training.com/blog/issues-in-heart-rate-variability-hrv-analysis-motion-artifacts-ectopic-beats
            if(a * 0.8 < b && a * 1.2 > b) {
                int diff = b - a;
                sum += diff * diff;
                ibiSum += a;
                diffCount++;
            }
        }
        double rmssd = sqrt((double)sum / diffCount);
        double bpm = 60000.0 / (ibiSum / diffCount);

        //RMSSDをvalenceに変換 (https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5624990/)
        //平均42,最大値75,最小値19を1～9に正規化
        double valence = 0;
        if(rmssd > 42) {
            valence = (rmssd-42) / (75-42) / 2 + 0.5;
        } else {
            valence = (rmssd-19) / (42-19) / 2;
        }
        valence = valence * 8 + 1;

        //平均GSRの計算
        sum = 0;
        for(int i = 0; i < gsrBuff.size(); i++) {
            sum += gsrBuff[i];
        }
        double avg_gsr = (double)sum / n;
        double cond = ((2048-avg_gsr) * 100)/(4096+2*avg_gsr);//校正 uS

        double arousal = cond / 50 * 8 + 1;
        
        // 歩数の取得
        unsigned long pedStepCount = imu.dmpGetPedometerSteps();
        steps = (int)(pedStepCount - pedLastStepCount);
        pedLastStepCount = pedStepCount;

        keepTbConn();
        tb.sendTelemetryFloat("bpm", bpm);
        tb.sendTelemetryFloat("arousal", arousal);
        tb.sendTelemetryFloat("valence", valence);
        tb.sendTelemetryFloat("steps", steps);
        
        gsrBuff.clear();
        ibiBuff.clear();
        // unsigned long endt = millis();
        // Serial.println(endt - startt);
    }
    
    
    int y = pulseSensor.getLatestSample();
    drawPulse.addValue(y);

    
    delay(20);
}

void initWiFi() {
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void initPulseSensor() {
    pulseSensor.analogInput(PIN_INPUT);
    pulseSensor.setThreshold(THRESHOLD);

    while (!pulseSensor.begin()) {
        Serial.println("PulseSensor.begin: failed");
        delay(500);
    }
}

void initImu() {
    while(imu.begin() != INV_SUCCESS) {
        Serial.println("Unable to communicate with MPU-9250");
        delay(500);
    }
    imu.dmpBegin(DMP_FEATURE_PEDOMETER);
    imu.dmpSetPedometerSteps(0); // バッファを0で初期化
    imu.dmpSetPedometerTime(0);
}

// サーバーとのコネクション維持
bool keepTbConn() {
    if (!tb.connected()) {
        // Connect to the ThingsBoard
        Serial.print("Connecting...");
        if (!tb.connect(address, key)) {
            Serial.println("Failed to connect");
            return false;
        }
        Serial.print("Connecting done");
    }
    tb.loop();
    return true;
}

// AP情報を取得しサーバーへ送信
bool postAP() {
    //int n = WiFi.scanNetworks(false, false, false, 200);
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    if (n == 0) {
        Serial.println("no networks found");
        return false;
    } else {
        DynamicJsonBuffer jsonBuffer;
        JsonObject &root = jsonBuffer.createObject();
        root["type"] = "wifi";
        
        int total = 0;
        for (int i = 0; i < n; ++i) {
            // 8個ぐらいあれば十分な気がする
            if(total > 8) {
                break;
            }
            String ssid = WiFi.SSID(i);
            if(isAvoidSSID(ssid)) {
                Serial.print("not use: ");
                Serial.println(ssid);
                continue;
            }
            char buff [20];

            uint8_t* macAddress = WiFi.BSSID(i);
            sprintf(buff, "%02X%02X%02X%02X%02X%02X", macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
            root[buff] =  WiFi.RSSI(i);
            total++;
        }
        if(total == 0) {
            Serial.println("no usable networks found");
            return false;
        }

        char output[300];
        root.printTo(output, sizeof(output));
        Serial.println(output);
        for(int i = 0; i < 10 ; i++) {
            if(keepTbConn()) {
                break;
            }
            delay(1000);
        }
        tb.sendTelemetryJson(output);
        return true;
    }
}

const int blacklistNum = 6;
char blacklist[blacklistNum][8] = {
    // WiMAX
    "SPWN", 
    "W0",
    "wx0",
    // ワイモバイル
    "HWa",
    // b-mobile他
    "mobile",
    // ～のiPhone他
    "Phone"
};

// 使わないAPだったら true
bool isAvoidSSID(const String &ssid) {
    // _nomapのオプトアウトに対応
    if(ssid.endsWith("_nomap")) {
        return true;
    }
    // モバイルWi-Fiかチェック
    for(int i = 0; i < blacklistNum; i++) {
        // indexOf:  一致したら場所を返す
        if(ssid.indexOf(blacklist[i]) != -1) {
            return true;
        }
    }

    return false;
}


//位置情報を定期的に更新するタスク
void taskGeo(void * pvParameters) {
    delay(30000); // 他のタイマとタイミングをちょっとずらす
    // APが見つかるまで探す
    while(!postAP()){
        delay(10000);
    }
    delay(60000);
    for(;;) {
        // 10歩以上歩いていたら更新
        //if(steps > 10) {
        if(true) {
           postAP();
        }
        delay(60000);
    }
}

// ハンドラ１：センサーリード
void _readGSR() {
    int gsr = analogRead(PIN_GSR);
    gsr = min(gsr, MAX_GSR); //センサーの最大値を超えたら最大値に固定
    gsrBuff.push_back(gsr);
}

// ハンドラ２：データ送信フラグの管理
void _sendFlgUp() {
    sendFlg = true;
}
