# M5_Sensor_ThingsBoard  
ThingsBoardで可視化をする   

## 機能   
(現在のところ)   
1分毎に
- 心拍数
- 覚醒度または眠気
- 快不快
- 歩数
- WiFiAP(サーバー側でAPI問い合わせ)  

を送信しグラフ化
## 使い方    
1. ライブラリを追加   
[GitHubにある ZIP形式ライブラリ のインストール方法 ( Arduino IDE )](https://www.mgo-tec.com/arduino-ide-lib-zip-install)
    - ThingsBoard.h  
    https://github.com/thingsboard/ThingsBoard-Arduino-MQTT-SDK
    - PulseSensorPlayground.h   
    心拍数用ライブラリ
    https://github.com/tanopanta/MyPulseSensorPlayground  
    - SparkFunMPU9250-DMP.h   
    歩数計用ライブラリ   
    https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library   
    **＊インストール時に数か所変更するー＞** https://qiita.com/tanopanta/items/7ec96bf4801eddedac39   
    - Wi-Fi位置測位用ライブラリ   
    https://github.com/tanopanta/M5Stack_WiFi_Geolocation
    - Jsonライブラリ(ArduinoJson)   
    Arduino IDEのライブラリマネージャーを利用しバージョン５系の最新版をインストール
    - MQTTライブラリ(pubsubclient)   
    https://github.com/knolleary/pubsubclient   
    **＊PubSubClient.hのMQTT_MAX_PACKET_SIZEを4096ぐらいに変更**
