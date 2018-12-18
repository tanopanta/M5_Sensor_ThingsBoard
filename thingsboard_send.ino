#include <ArduinoJson.h>
#include <M5Stack.h>
#include <WiFi.h>
#include <MyPulseSensorPlayground.h>
#include <SparkFunMPU9250-DMP.h>
#include <Ambient.h>
#include <ThingsBoard.h>
#include <drawPulse.h>

#include "myconfig.h"

const int PIN_INPUT = 36;
const int THRESHOLD = 2200;   // Adjust this number to avoid noise when idle

const int PIN_GSR = 35;
const int MAX_GSR = 2047;

PulseSensorPlayground pulseSensor;
MPU9250_DMP imu;

WiFiClient espClient;
ThingsBoard tb(espClient);
DrawPulse drawPulse;

int steps = 0;


bool getAP() {
    int n = WiFi.scanNetworks(false, false, false, 101);
    // Serial.println("scan done");
    if (n == 0) {
        Serial.println("no networks found");
        return false;
    } else {
        DynamicJsonBuffer jsonBuffer;
        JsonObject &root = jsonBuffer.createObject();
        root["type"] = "wifi";
        // 8個ぐらいあれば十分な気がする
        if(n > 8) {
          n = 8;
        }

        for (int i = 0; i < n; i++) {
            char buff [20];

            uint8_t* macAddress = WiFi.BSSID(i);
            sprintf(buff, "%02X%02X%02X%02X%02X%02X", macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
            root[buff] =  WiFi.RSSI(i);
        }

        char output[300];
        root.printTo(output, sizeof(output));
        Serial.println(output);
        tb.sendTelemetryJson(output);
        return true;
    }
}

//位置情報を定期的に更新するタスク
void taskGeo(void * pvParameters) {
    
    // APが見つかるまで探す
    while(!getAP()){
        delay(10000);
    }
    for(;;) {
        // 10歩以上歩いていたら更新
        if(steps > 10) {
           while(!getAP()) {
               delay(10000);
           }

        }
        delay(60000);
    }
}


void setup() {
    M5.begin();
    dacWrite(25, 0); // Speaker OFF

    initWiFi();

    Serial.print("WiFi connected\r\nIP address: ");
    Serial.println(WiFi.localIP());

    initPulseSensor();
    initImu();

    drawPulse.init();

    // 位置情報用タスクのセット
    xTaskCreatePinnedToCore(
                    taskGeo,     /* Function to implement the task */
                    "taskGeo",   /* Name of the task */
                    8192,      /* Stack size in words */
                    NULL,      /* Task input parameter */
                    1,         /* Priority of the task */
                    NULL,      /* Task handle. */
                    0);        /* Core where the task should run */
}


#define REDRAW 20 // msec
#define PERIOD 60 // sec


int loopcount = 0;

int ibis[256];
int gsrs[256];
int pointer = 0;


unsigned long pedLastStepCount = 0;

void loop() {
    delay(REDRAW);
    if (pulseSensor.sawStartOfBeat()) {            // Constantly test to see if "a beat happened". 
        int ibi = pulseSensor.getInterBeatIntervalMs();
        int gsr = analogRead(PIN_GSR);

        ibis[pointer] = ibi;
        gsr = min(gsr, MAX_GSR); //センサーの最大値を超えたら最大値に固定
        gsrs[pointer] = gsr;
        pointer++;
        
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.setTextSize(3);
        M5.Lcd.printf("BPM: %03d", 60000 /ibi);
    }

    int y = pulseSensor.getLatestSample();
    drawPulse.addValue(y);      
    
    if (++loopcount > PERIOD * 1000 / REDRAW) {
        loopcount = 0;
        int n = pointer;
        pointer = 0;
        //10拍も取れていないー＞計算をあきらめる
        if(n < 10){
            return;
        }
        
        
        //RMSSD(RR間隔の差の自乗平均平方根)の計算
        int diffCount = 0;
        int sum = 0;
        for(int i = 1;i < n; i++) {
            int a = ibis[i-1];
            int b = ibis[i];

            //差が20%以下のみ使用　https://www.hrv4training.com/blog/issues-in-heart-rate-variability-hrv-analysis-motion-artifacts-ectopic-beats
            if(a * 0.8 < b && a * 1.2 > b) {
                int diff = b - a;
                sum += diff * diff;
                diffCount++;
            }
        }
        double rmssd = sqrt((double)sum / diffCount);

        //RMSSDをvalenceに変換 (https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5624990/)
        //平均42,最大値75,最小値19を1～9に正規化
        double valence = 0;
        if(valence > 42) {
            valence = (rmssd-42) / (75-42) / 2 + 0.5;
        } else {
            valence = (rmssd-19) / (42-19) / 2;
        }
        valence = valence * 8 + 1;

        //平均心拍数の計算
        sum = 0;
        for(int i = 0; i < n; i++) {
            sum += ibis[i];
        }
        int avg = sum / n;

        double bpm = 60000.0 / avg;

        //平均GSRの計算
        sum = 0;
        for(int i = 0; i < n; i++) {
            sum += gsrs[i];
        }
        double avg_gsr = (double)sum / n;
        double cond = ((2048-avg_gsr) * 100)/(4096+2*avg_gsr);//校正 uS

        double arousal = cond / 50 * 8 + 1;
        
        unsigned long pedStepCount = imu.dmpGetPedometerSteps();
        steps = (int)(pedStepCount - pedLastStepCount);
        pedLastStepCount = pedStepCount;

        // コネクション維持
        if (!tb.connected()) {
          // Connect to the ThingsBoard
          Serial.print("Connecting...");
          if (!tb.connect(address, key)) {
            Serial.println("Failed to connect");
          }
          Serial.print("Connecting done");
        }
        

        tb.sendTelemetryFloat("bpm", bpm);
        tb.sendTelemetryFloat("arousal", arousal);
        tb.sendTelemetryFloat("valence", valence);
        tb.sendTelemetryFloat("steps", steps);
    }
    tb.loop();
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