// === transmitter_proposed_no_tflm_auto_channel.ino ===
// Board A: XIAO ESP32S3 Sense + Grove sensors + SD + Wio-E5 LoRa + analog mic + Camera
//
// GitHub-safe version:
// - No real LoRa AppKey
// - No real ESP-NOW receiver MAC address
//
// Before flashing:
// 1. Replace RECEIVER_MAC with your receiver board MAC address
// 2. Replace LORA_APPKEY with your own LoRaWAN AppKey

#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "FS.h"
#include <SensirionI2cScd4x.h>
#include <esp_camera.h>
#include <esp_wifi.h>

// -------------------- USER CONFIG --------------------
// Replace these before flashing.
// Do not commit real credentials to a public repo.

uint8_t RECEIVER_MAC[6] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const char* LORA_APPKEY = "REPLACE_WITH_YOUR_LORA_APPKEY";

// -----------------------------------------------------

#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

#define LOGI(tag, fmt, ...)  do { Serial.printf("[%s] " fmt "\n", tag, ##__VA_ARGS__); } while(0)
#define LOGOK(tag, fmt, ...) do { Serial.printf("[%s][OK] " fmt "\n", tag, ##__VA_ARGS__); } while(0)
#define LOGF(tag, fmt, ...)  do { Serial.printf("[%s][FAIL] " fmt "\n", tag, ##__VA_ARGS__); } while(0)

#define ULTRASONIC_PIN D0
#define MIC_ANALOG_PIN A1
#define LORA_RX_PIN    D7
#define LORA_TX_PIN    D6
#define SD_CS_PIN      D3
#define SPEAKER_PIN    D2

static const int SAMPLE_RATE = 16000;
static const int FRAME_SAMPLES = 16000;
static const int CAPTURE_SEC = 2;
static const int CAPTURE_SAMPLES = SAMPLE_RATE * CAPTURE_SEC;
static const uint32_t STATUS_PERIOD_MS = 500;
static const uint32_t AUDIO_FRAME_PERIOD_MS = 1200;
static const uint32_t LORA_ENV_INTERVAL_MS = 120000UL;
static const float HUMAN_DIST_M = 1.5f;

enum MsgType : uint8_t {
  MSG_STATUS = 1,
  MSG_PING   = 2,
  MSG_ACK    = 3
};

struct StatusPacket {
  int16_t  temp_x100;
  uint16_t co2_ppm;
  uint16_t rh_x100;
  uint8_t  anomaly;
  uint8_t  human_near;
  uint16_t last_max_amp;
} __attribute__((packed));

static uint8_t receiver_channel = 1;
static volatile bool ack_received = false;
static volatile bool receiver_found = false;

SensirionI2cScd4x scd4x;
static uint16_t scd_co2 = 0;
static uint16_t scd_rh_x100 = 0;
static int16_t scd_temp_x100 = 0;
static uint32_t last_status_ms = 0;
static uint32_t last_audio_ms = 0;
static uint32_t last_lora_ms = 0;
static bool sd_ok = false;

#if defined(BOARD_HAS_PSRAM)
#include "esp_heap_caps.h"
static int16_t* g_frame = nullptr;
static int16_t* g_capture = nullptr;
#else
static int16_t g_frame[FRAME_SAMPLES];
static int16_t g_capture[CAPTURE_SAMPLES];
#endif

static volatile int16_t last_max_amp = 0;
static int micBaseline = 2048;
static uint8_t current_anomaly_state = 0;

// -------------------- AUDIO / SENSOR HELPERS --------------------
static float readDistanceMeters() {
  pinMode(ULTRASONIC_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_PIN, LOW);

  pinMode(ULTRASONIC_PIN, INPUT);
  uint32_t duration = pulseIn(ULTRASONIC_PIN, HIGH, 30000);
  if (duration == 0) return 999.0f;
  return (duration / 58.2f) / 100.0f;
}

static void calibrateMicBaseline() {
  int32_t sum = 0;
  for (int i = 0; i < 200; i++) {
    sum += analogRead(MIC_ANALOG_PIN);
    delay(2);
  }
  micBaseline = (int)(sum / 200);
}

static void captureAudioAnalog(int16_t* buffer, int samples, bool trackMax) {
  const uint32_t us_per_sample = 1000000UL / SAMPLE_RATE;
  int16_t maxamp = 0;
  uint32_t t = micros();

  for (int i = 0; i < samples; i++) {
    while ((uint32_t)(micros() - t) < us_per_sample) {}
    t += us_per_sample;

    int raw = analogRead(MIC_ANALOG_PIN);
    int16_t s = (int16_t)((raw - micBaseline) * 16);
    buffer[i] = s;

    if (trackMax) {
      int16_t a = abs(s);
      if (a > maxamp) maxamp = a;
    }
  }

  if (trackMax) last_max_amp = maxamp;
}

static void speakerTone(int freq, int dur_ms) {
  if (freq <= 0) {
    ledcWriteTone(SPEAKER_PIN, 0);
    delay(dur_ms);
    return;
  }

  ledcWriteTone(SPEAKER_PIN, freq);
  delay(dur_ms);
  ledcWriteTone(SPEAKER_PIN, 0);
}

static void playBirdSound() {
  for (int i = 0; i < 3; i++) {
    speakerTone(3200, 60);
    speakerTone(3800, 50);
    speakerTone(4500, 40);
    speakerTone(0, 30);

    speakerTone(4200, 50);
    speakerTone(3600, 70);
    speakerTone(0, 40);
  }

  speakerTone(5000, 35);
  speakerTone(4200, 45);
  speakerTone(3500, 60);
}

static void initSD() {
  sd_ok = SD.begin(SD_CS_PIN);
  if (sd_ok) LOGOK("SD", "card mounted");
  else LOGF("SD", "card mount failed");
}

static String makeAudioFilename2s() {
  static uint32_t idx = 0;
  idx++;
  char buf[48];
  snprintf(buf, sizeof(buf), "/anom_%lu.raw", (unsigned long)idx);
  return String(buf);
}

static bool saveAudioRawToSD(const String& path, const int16_t* samples, int n) {
  if (!sd_ok) return false;
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;
  size_t wrote = f.write((const uint8_t*)samples, (size_t)n * sizeof(int16_t));
  f.close();
  return wrote == (size_t)n * sizeof(int16_t);
}

// -------------------- LORA --------------------
static char lora_buf[256];
static bool lora_present = false;
static bool lora_joined = false;

static bool loraWaitFor(const char* needle, uint32_t timeout_ms) {
  uint32_t t0 = millis();
  size_t n = 0;
  lora_buf[0] = 0;

  while (millis() - t0 < timeout_ms) {
    while (Serial1.available()) {
      char c = (char)Serial1.read();
      if (n < sizeof(lora_buf) - 1) {
        lora_buf[n++] = c;
        lora_buf[n] = 0;
      }
      if (strstr(lora_buf, needle)) return true;
    }
    delay(1);
  }
  return false;
}

static bool loraCmd(const char* expect, uint32_t timeout_ms, const char* cmd) {
  Serial1.print(cmd);
  return loraWaitFor(expect, timeout_ms);
}

static void initLoRa() {
  Serial1.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  delay(200);

  if (strcmp(LORA_APPKEY, "REPLACE_WITH_YOUR_LORA_APPKEY") == 0) {
    LOGF("LORA", "LORA_APPKEY is not configured");
    return;
  }

  if (loraCmd("+AT: OK", 300, "AT\r\n")) {
    lora_present = true;

    loraCmd("+MODE: LWOTAA", 1000, "AT+MODE=LWOTAA\r\n");
    loraCmd("+DR: EU868", 1000, "AT+DR=EU868\r\n");
    loraCmd("+CH: NUM", 1000, "AT+CH=NUM,0-2\r\n");

    char keyCmd[96];
    snprintf(keyCmd, sizeof(keyCmd), "AT+KEY=APPKEY,\"%s\"\r\n", LORA_APPKEY);
    loraCmd("+KEY: APPKEY", 1000, keyCmd);

    loraCmd("+CLASS: A", 1000, "AT+CLASS=A\r\n");
    loraCmd("+PORT: 8", 1000, "AT+PORT=8\r\n");

    if (loraCmd("+JOIN: Network joined", 12000, "AT+JOIN\r\n")) {
      lora_joined = true;
    }
  }
}

static String toHex2(uint8_t v) {
  const char* h = "0123456789ABCDEF";
  return String(h[(v >> 4) & 0xF]) + String(h[v & 0xF]);
}

static String toHexU16(uint16_t v) {
  return toHex2((v >> 8) & 0xFF) + toHex2(v & 0xFF);
}

static String toHexI16(int16_t v) {
  return toHexU16((uint16_t)v);
}

static void sendLoRaEnv(bool is_event, float dist_m) {
  if (!lora_present || !lora_joined) return;

  uint16_t dist_cm = (dist_m > 650.0f) ? 65535 : (uint16_t)(dist_m * 100.0f);
  uint8_t flags = 0;
  if (dist_m < HUMAN_DIST_M) flags |= 0x01;
  if (is_event) flags |= 0x02;

  String hex = toHexI16(scd_temp_x100) +
               toHexU16(scd_co2) +
               toHexU16(scd_rh_x100) +
               toHexU16(dist_cm) +
               toHex2(flags) +
               toHexI16(last_max_amp);

  Serial1.print("AT+MSGHEX=\"" + hex + "\"\r\n");
  loraWaitFor("+MSGHEX: Done", 8000);
}

// -------------------- ESP-NOW --------------------
static bool receiverMacConfigured() {
  for (int i = 0; i < 6; i++) {
    if (RECEIVER_MAC[i] != 0x00) return true;
  }
  return false;
}

static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len < 1) return;
  if (memcmp(info->src_addr, RECEIVER_MAC, 6) != 0) return;

  if (data[0] == MSG_ACK) {
    ack_received = true;
    receiver_found = true;
  }
}

static void sendPing() {
  uint8_t msg = MSG_PING;
  esp_now_send(RECEIVER_MAC, &msg, 1);
}

static bool addReceiverPeerOnCurrentChannel() {
  esp_now_del_peer(RECEIVER_MAC);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, RECEIVER_MAC, 6);
  peer.channel = receiver_channel;
  peer.encrypt = false;

  return esp_now_add_peer(&peer) == ESP_OK;
}

static bool findReceiverChannel() {
  if (!receiverMacConfigured()) {
    LOGF("ESP-NOW", "RECEIVER_MAC is not configured");
    return false;
  }

  for (uint8_t ch = 1; ch <= 13; ch++) {
    receiver_channel = ch;
    ack_received = false;

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(receiver_channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (!addReceiverPeerOnCurrentChannel()) {
      continue;
    }

    for (int i = 0; i < 4; i++) {
      sendPing();
      delay(150);
      if (ack_received) {
        Serial.printf("Receiver found on channel %u\n", receiver_channel);
        return true;
      }
    }
  }

  return false;
}

static void sendStatus(float dist_m, uint8_t anomaly_flag) {
  StatusPacket pkt{
    scd_temp_x100,
    scd_co2,
    scd_rh_x100,
    anomaly_flag,
    (uint8_t)(dist_m < HUMAN_DIST_M ? 1 : 0),
    (uint16_t)last_max_amp
  };

  uint8_t buf[1 + sizeof(StatusPacket)];
  buf[0] = MSG_STATUS;
  memcpy(buf + 1, &pkt, sizeof(pkt));

  esp_err_t err = esp_now_send(RECEIVER_MAC, buf, sizeof(buf));
  if (err != ESP_OK) {
    Serial.printf("ESP-NOW send failed: %d\n", err);
  }
}

// -------------------- OTHER HELPERS --------------------
static void readSCD41() {
  uint16_t co2;
  float temperature;
  float humidity;

  if (scd4x.readMeasurement(co2, temperature, humidity) == 0 && co2 != 0) {
    scd_co2 = co2;
    scd_temp_x100 = (int16_t)lroundf(temperature * 100.0f);
    scd_rh_x100 = (uint16_t)lroundf(humidity * 100.0f);
  }
}

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  return esp_camera_init(&config) == ESP_OK;
}

void capturePhoto() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) {
    File file = SD.begin(SD_CS_PIN) ? SD.open("/photo.jpg", FILE_WRITE) : File();
    if (file) {
      file.write(fb->buf, fb->len);
      file.close();
    }
    esp_camera_fb_return(fb);
  }
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n=================================");
  Serial.println(" TRANSMITTER BOOT");
  Serial.println("=================================");

  analogReadResolution(12);
  calibrateMicBaseline();

#if defined(BOARD_HAS_PSRAM)
  g_frame = (int16_t*)heap_caps_malloc(
    FRAME_SAMPLES * sizeof(int16_t),
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );

  g_capture = (int16_t*)heap_caps_malloc(
    CAPTURE_SAMPLES * sizeof(int16_t),
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );
#endif

  Wire.begin(D4, D5);
  scd4x.begin(Wire, 0x62);
  scd4x.startPeriodicMeasurement();

  initSD();
  initLoRa();
  initCamera();

  if (!ledcAttach(SPEAKER_PIN, 2000, 8)) {
    Serial.println("Speaker LEDC attach failed");
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onRecv);

  if (!findReceiverChannel()) {
    Serial.println("Receiver channel not found or RECEIVER_MAC not configured");
    return;
  }

  last_status_ms = last_audio_ms = last_lora_ms = millis();
}

// -------------------- LOOP --------------------
void loop() {
  if (!receiver_found) {
    delay(100);
    return;
  }

  uint32_t now = millis();
  static uint32_t last_display_ms = 0;

  float current_dist = readDistanceMeters();
  bool human_present = current_dist < HUMAN_DIST_M;

  Serial.printf("dist=%.2f human=%d\n", current_dist, human_present);

  static uint32_t lastBirdMs = 0;

  if (human_present && millis() - lastBirdMs >= 3000) {
    LOGI("SPEAKER", "Human present -> chirping...");
    playBirdSound();
    lastBirdMs = millis();
  }

  if (now - last_display_ms > 2000) {
    last_display_ms = now;
    readSCD41();

    Serial.printf(
      "\n--- Dashboard ---\n"
      "CO2: %u ppm | Temp: %.2f C | Hum: %.2f %%\n"
      "Ultrasonic Distance: %.2f m | Human Near: %s | Ch: %u\n",
      scd_co2,
      scd_temp_x100 / 100.0f,
      scd_rh_x100 / 100.0f,
      current_dist,
      human_present ? "YES" : "NO",
      receiver_channel
    );
  }

  if (now - last_audio_ms >= AUDIO_FRAME_PERIOD_MS) {
    last_audio_ms = now;

    if (g_frame) {
      captureAudioAnalog(g_frame, FRAME_SAMPLES, true);

      current_anomaly_state = last_max_amp > 22500 ? 1 : 0;

      Serial.printf(
        "[AUDIO] Max amplitude: %d | Anomaly: %s\n",
        last_max_amp,
        current_anomaly_state ? "YES" : "NO"
      );

      static uint8_t consec = 0;

      if (current_anomaly_state) {
        consec++;
      } else {
        consec = 0;
      }

      if (consec == 2) {
        LOGI("ANOMALY", "Triggering SD Capture & LoRa...");

        if (g_capture) {
          captureAudioAnalog(g_capture, CAPTURE_SAMPLES, true);
          saveAudioRawToSD(makeAudioFilename2s(), g_capture, CAPTURE_SAMPLES);
        }

        sendLoRaEnv(true, current_dist);
        last_lora_ms = millis();
      }
    }
  }

  if (now - last_status_ms >= STATUS_PERIOD_MS) {
    last_status_ms = now;
    sendStatus(current_dist, current_anomaly_state);
  }

  if (now - last_lora_ms >= LORA_ENV_INTERVAL_MS) {
    last_lora_ms = now;
    sendLoRaEnv(false, current_dist);
  }

  delay(5);
}
