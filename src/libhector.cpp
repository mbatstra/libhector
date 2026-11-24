#include "libhector.h"
#include "brownian/Brownian.h"
#include "omni/OmniBase.h"
#include "omni/OmniTypes.h"

#include "Arduino.h"
#include "ArduinoMqttClient.h"
#include "MqttClient.h"
#include "WiFi.h"
#include "WiFiClient.h"
#include "WiFiType.h"

static const char *ssid = "tcsensors", *pass = "wijvanTCeend";
static const char *broker = "192.168.68.185";
static const int port = 1883;

static OmniBase base;
static Brownian brownian;
static WiFiClient wifiClient;
static MqttClient mqttClient(wifiClient);

static void onMqttMsg(int msg_len) {
    (void)msg_len;
}

void hector_init() {
    base.init();
    base.enable();

    Serial.begin(115200);
    Serial.printf("connecting to WPA SSID: %s\n", ssid);
    for (int attempts = 10; attempts > 0; attempts--) {
        WiFi.begin(ssid, pass);
        uint8_t wifi_status = WiFi.waitForConnectResult();
        if (wifi_status != WL_CONNECTED) {
            Serial.printf("failed to connect (status: %d), retrying...\n", wifi_status);
        } else {
            break;
        }
        delay(2500);
    }
    Serial.println("wifi connect success");

    Serial.printf("connecting to mqtt broker: %s:%d\n", broker, port);
    if (!mqttClient.connect(broker, port)) {
        Serial.println("failed to connect to broker");
    } else {
        Serial.println("connected to broker");
    }

    mqttClient.onMessage(onMqttMsg)r;
}

void hector_service() {
    mqttClient.poll();
}

void hector_move(float x_mm, float y_mm, float v_mms) {
    base.move(x_mm / 1000.0f, y_mm / 1000.0f, 0.0f, v_mms / 1000.0f, 0.0f);
}

bool hector_is_idle() {
    return base.telemetry().state != OmniCommandState::RUNNING;
}

void brownian_init(float mean, float stddev, uint32_t seed) {
    brownian = Brownian(mean, stddev, seed);
}

float brownian_draw() {
    return brownian.draw();
}
