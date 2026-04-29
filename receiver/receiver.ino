#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <time.h>
#include <TFT_eSPI.h>
#include <esp_wifi.h>
#define BOARD_SCREEN_COMBO 501

// -------------------- USER CONFIG --------------------
// Replace these before flashing.
// Do not commit real credentials to a public repo.

uint8_t TRANSMITTER_MAC[6] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const char* WIFI_SSID = "REPLACE_WITH_YOUR_WIFI_SSID";
const char* WIFI_PASS = "REPLACE_WITH_YOUR_WIFI_PASSWORD";

// Lithuania / EET-EEST example timezone.
// Change if your deployment is elsewhere.
const char* NTP_SERVER = "pool.ntp.org";
const char* TZ_INFO    = "EET-2EEST,M3.5.0/3,M10.5.0/4";

// -----------------------------------------------------

// -------------------- TYPE DEFINITIONS --------------------
enum MsgType : uint8_t {
  MSG_STATUS = 1,
  MSG_PING   = 2,
  MSG_ACK    = 3
};

enum UiMode  : uint8_t { MODE_IDLE = 0, MODE_HUMAN = 1, MODE_ANOM = 2 };
enum HumanPage : uint8_t {
  PAGE_BIRD = 0,
  PAGE_TIME = 1,
  PAGE_RH   = 2,
  PAGE_CO2  = 3,
  PAGE_KTU  = 4,
  PAGE_TEMP = 5,
  PAGE_COUNT
};

struct StatusPacket {
  int16_t  temp_x100;
  uint16_t co2_ppm;
  uint16_t rh_x100;
  uint8_t  anomaly;
  uint8_t  human_near;
  uint16_t last_max_amp;
} __attribute__((packed));

struct HumanPageRule {
  HumanPage id;
  uint8_t weight;
  uint16_t min_ms;
  uint16_t max_ms;
};

// -------------------- EXPLICIT PROTOTYPES --------------------
static void schedulePage(HumanPage p);
static void setMode(UiMode m);
static HumanPage pickNext(HumanPage prev, UiMode mode);

// -------------------- LOGGING --------------------
#define LOGI(tag, fmt, ...)  do { Serial.printf("[%s] " fmt "\n", tag, ##__VA_ARGS__); } while(0)
#define LOGOK(tag, fmt, ...) do { Serial.printf("[%s][OK] " fmt "\n", tag, ##__VA_ARGS__); } while(0)
#define LOGF(tag, fmt, ...)  do { Serial.printf("[%s][FAIL] " fmt "\n", tag, ##__VA_ARGS__); } while(0)

// -------------------- WIFI / TIME --------------------
static const uint8_t ESPNOW_FALLBACK_CHANNEL = 6;

static bool    g_ntp_ok = false;
static uint8_t g_wifi_channel = ESPNOW_FALLBACK_CHANNEL;

// -------------------- DISPLAY & UI STATE --------------------
TFT_eSPI tft;
static StatusPacket g_status{};
static volatile bool g_have_status = false;

static uint32_t g_last_human_seen_ms = 0;
static uint32_t g_last_anom_seen_ms  = 0;
static const uint32_t HUMAN_HOLD_MS  = 5000;
static const uint32_t ANOM_HOLD_MS   = 10000;

static UiMode g_mode = MODE_IDLE;
static UiMode g_last_drawn_mode = (UiMode)255;

static const HumanPageRule HUMAN_RULES[] = {
  { PAGE_BIRD, 1, 2000, 3000 },
  { PAGE_KTU,  1, 2000, 3000 },
  { PAGE_TIME, 1, 2000, 3000 },
  { PAGE_RH,   1, 2000, 3000 },
  { PAGE_CO2,  1, 2000, 3000 },
  { PAGE_TEMP, 1, 2000, 3000 }
};

static HumanPage g_page = PAGE_BIRD;
static uint32_t  g_page_start_ms = 0;
static uint32_t  g_page_dwell_ms = 7000;

// -------------------- CONFIG CHECKS --------------------
static bool transmitterMacConfigured() {
  for (int i = 0; i < 6; i++) {
    if (TRANSMITTER_MAC[i] != 0x00) return true;
  }
  return false;
}

static bool wifiConfigured() {
  return strcmp(WIFI_SSID, "REPLACE_WITH_YOUR_WIFI_SSID") != 0 &&
         strcmp(WIFI_PASS, "REPLACE_WITH_YOUR_WIFI_PASSWORD") != 0;
}

// -------------------- WIFI / NTP INIT --------------------
static void initTimeAndWifiForNtp(const char* tag) {
  if (!wifiConfigured()) {
    g_wifi_channel = ESPNOW_FALLBACK_CHANNEL;
    g_ntp_ok = false;

    LOGF(tag, "WiFi credentials not configured; skipping NTP");
    LOGI(tag, "using fallback ESP-NOW channel=%u", g_wifi_channel);

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(g_wifi_channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    return;
  }

  LOGI(tag, "connecting WiFi...");
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 8000) {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    g_wifi_channel = WiFi.channel();
    LOGOK(tag, "WiFi connected, channel=%u", g_wifi_channel);

    configTime(0, 0, NTP_SERVER);
    setenv("TZ", TZ_INFO, 1);
    tzset();

    struct tm ti;
    if (getLocalTime(&ti, 2000)) {
      g_ntp_ok = true;
      LOGOK(tag, "time synced: %02d:%02d:%02d", ti.tm_hour, ti.tm_min, ti.tm_sec);
    } else {
      LOGF(tag, "NTP sync failed");
    }

    WiFi.disconnect(false, false);
    delay(100);
  } else {
    g_wifi_channel = ESPNOW_FALLBACK_CHANNEL;
    LOGF(tag, "WiFi connect failed, fallback channel=%u", g_wifi_channel);
  }

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(g_wifi_channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  LOGOK(tag, "ESP-NOW radio set to channel %u", g_wifi_channel);
}

// -------------------- TIME --------------------
static String timeHHMM() {
  if (!g_ntp_ok) return String("--:--");

  struct tm ti;
  if (!getLocalTime(&ti, 10)) return String("--:--");

  char b[6];
  strftime(b, sizeof(b), "%H:%M", &ti);
  return String(b);
}

static String getTimeString() {
  return timeHHMM();
}

// -------------------- DRAW HELPERS --------------------
static void clearScreen() {
  tft.fillScreen(TFT_BLACK);
}

static void drawCentered(const String& txt, int y, int font = 4) {
  tft.setTextDatum(MC_DATUM);
  tft.setTextFont(font);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(txt, tft.width() / 2, y);
}

static void drawBird(bool red, const String& bubble) {
  clearScreen();

  int cx = tft.width() / 2;
  int cy = tft.height() / 2 + 15;
  uint16_t body = red ? TFT_RED : TFT_BLUE;
  uint16_t wing = red ? TFT_MAROON : TFT_NAVY;

  tft.fillCircle(cx, cy, 45, body);
  tft.fillCircle(cx + 10, cy + 10, 25, TFT_WHITE);
  tft.fillCircle(cx - 25, cy + 5, 22, wing);
  tft.fillCircle(cx + 15, cy - 15, 6, TFT_WHITE);
  tft.fillCircle(cx + 17, cy - 15, 3, TFT_BLACK);

  int bx = cx + 45;
  int by = cy - 2;
  tft.fillTriangle(bx, by, bx + 18, by + 8, bx, by + 16, TFT_YELLOW);

  int w = tft.width() - 100;
  int x = (tft.width() - w) / 2;

  tft.drawRoundRect(x, 20, w, 60, 12, TFT_WHITE);

  if (bubble == "Anomaly Detected!") {
    drawCentered("Anomaly", 40, 4);
    drawCentered("Detected!", 65, 4);
  } else {
    drawCentered(bubble, 50, 4);
  }

  tft.setTextDatum(BL_DATUM);
  tft.setTextFont(2);
  tft.drawString(getTimeString(), 6, tft.height() - 4);
}

static void drawTimeOnly() {
  clearScreen();
  drawCentered(getTimeString(), tft.height() / 2, 7);
}

static float rhPct() {
  return g_status.rh_x100 / 100.0f;
}

static float tempC() {
  return g_status.temp_x100 / 100.0f;
}

static void drawRH() {
  clearScreen();
  drawCentered("Hum(%):", 40, 4);

  tft.setTextFont(7);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(String(rhPct(), 1), tft.width() / 2, tft.height() / 2 + 10);

  tft.setTextFont(2);
  tft.setTextDatum(BC_DATUM);
  tft.drawString(getTimeString(), tft.width() / 2, tft.height() - 8);
}

static void drawCO2() {
  clearScreen();
  drawCentered("CO2(ppm):", 40, 4);

  tft.setTextFont(7);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(String(g_status.co2_ppm), tft.width() / 2, tft.height() / 2 + 10);

  tft.setTextFont(2);
  tft.setTextDatum(BC_DATUM);
  tft.drawString(getTimeString(), tft.width() / 2, tft.height() - 8);
}

static void drawTemp() {
  clearScreen();
  drawCentered("Temp(C):", 40, 4);

  tft.setTextFont(7);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(String(tempC(), 1), tft.width() / 2, tft.height() / 2 + 10);

  tft.setTextFont(2);
  tft.setTextDatum(BC_DATUM);
  tft.drawString(getTimeString(), tft.width() / 2, tft.height() - 8);
}

static void drawKTU() {
  clearScreen();

  int side = 140;
  int x = (tft.width() - side) / 2;
  int y = (tft.height() - side) / 2;
  int thickness = 8;

  for (int i = 0; i < thickness; i++) {
    tft.drawRect(x + i, y + i, side - (i * 2), side - (i * 2), TFT_WHITE);
  }

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextFont(4);
  tft.drawString("ktu", tft.width() / 2, tft.height() / 2);

  tft.setTextFont(2);
  tft.setTextDatum(BL_DATUM);
  tft.drawString("1922", x + thickness + 4, y + side - thickness - 4);

  tft.setTextFont(2);
  tft.setTextDatum(BC_DATUM);
  tft.drawString(getTimeString(), tft.width() / 2, tft.height() - 8);
}

static uint32_t randRange(uint32_t a, uint32_t b) {
  return (b <= a) ? a : a + (esp_random() % (b - a + 1));
}

static HumanPage pickNext(HumanPage prev, UiMode mode) {
  if (mode == MODE_IDLE) {
    return (prev == PAGE_TIME) ? PAGE_KTU : PAGE_TIME;
  }

  switch (prev) {
    case PAGE_BIRD: return PAGE_TIME;
    case PAGE_TIME: return PAGE_RH;
    case PAGE_RH:   return PAGE_CO2;
    case PAGE_CO2:  return PAGE_TEMP;
    case PAGE_TEMP: return PAGE_KTU;
    case PAGE_KTU:  return PAGE_BIRD;
    default:        return PAGE_BIRD;
  }
}

static void schedulePage(HumanPage p) {
  g_page = p;

  if (g_mode == MODE_IDLE) {
    g_page_dwell_ms = randRange(5000, 9000);
    g_page_start_ms = millis();
    return;
  }

  for (auto &ru : HUMAN_RULES) {
    if (ru.id == p) {
      g_page_dwell_ms = randRange(ru.min_ms, ru.max_ms);
      break;
    }
  }

  g_page_start_ms = millis();
}

static void drawHumanPage() {
  switch (g_page) {
    case PAGE_BIRD: drawBird(false, "Hello There!"); break;
    case PAGE_TIME: drawTimeOnly(); break;
    case PAGE_RH:   drawRH(); break;
    case PAGE_CO2:  drawCO2(); break;
    case PAGE_TEMP: drawTemp(); break;
    case PAGE_KTU:  drawKTU(); break;
    default:        drawTimeOnly(); break;
  }
}

static void setMode(UiMode m) {
  if (m == g_mode) return;

  UiMode old_mode = g_mode;
  g_mode = m;

  if (g_mode == MODE_HUMAN) {
    if (old_mode == MODE_IDLE) {
      schedulePage(PAGE_BIRD);
    } else {
      schedulePage(pickNext(g_page, g_mode));
    }
  } else if (g_mode == MODE_IDLE) {
    schedulePage(PAGE_TIME);
  }
}

// -------------------- ESP-NOW RECEIVE --------------------
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len < 1) return;

  if (memcmp(info->src_addr, TRANSMITTER_MAC, 6) != 0) {
    return;
  }

  if (data[0] == MSG_PING) {
    uint8_t ack = MSG_ACK;
    esp_now_send(info->src_addr, &ack, 1);
    return;
  }

  if (data[0] == MSG_STATUS && len == (int)(1 + sizeof(StatusPacket))) {
    memcpy(&g_status, data + 1, sizeof(StatusPacket));
    g_have_status = true;

    uint32_t now = millis();
    if (g_status.human_near) g_last_human_seen_ms = now;
    if (g_status.anomaly)    g_last_anom_seen_ms  = now;

    Serial.printf(
      "[RX] T=%.2fC CO2=%u RH=%.2f A=%u H=%u AMP=%u\n",
      g_status.temp_x100 / 100.0f,
      g_status.co2_ppm,
      g_status.rh_x100 / 100.0f,
      g_status.anomaly,
      g_status.human_near,
      g_status.last_max_amp
    );
  }
}

// -------------------- SETUP / LOOP --------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n=================================");
  Serial.println(" RECEIVER BOOT (LIGHTWEIGHT UI)");
  Serial.println("=================================");

  tft.init();
  tft.setRotation(3);
  clearScreen();

  initTimeAndWifiForNtp("NTP");

  if (esp_now_init() != ESP_OK) {
    clearScreen();
    drawCentered("ESP-NOW init FAIL", tft.height() / 2, 2);
    while (1) delay(1000);
  }

  if (!transmitterMacConfigured()) {
    clearScreen();
    drawCentered("SET TRANSMITTER_MAC", tft.height() / 2, 2);
    LOGF("BOOT", "TRANSMITTER_MAC is not configured");
    while (1) delay(1000);
  }

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, TRANSMITTER_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add transmitter peer");
  }

  if (esp_now_register_recv_cb(onRecv) != ESP_OK) {
    clearScreen();
    drawCentered("RX CB FAIL", tft.height() / 2, 2);
    while (1) delay(1000);
  }

  schedulePage(PAGE_TIME);
  g_page_start_ms = millis();

  LOGOK("BOOT", "receiver ready");
  LOGI("BOOT", "display time = %s", getTimeString().c_str());
  LOGI("BOOT", "ESP-NOW channel = %u", g_wifi_channel);
}

void loop() {
  uint32_t now = millis();

  bool human_recent = (now - g_last_human_seen_ms) < HUMAN_HOLD_MS;
  bool anom_recent  = (now - g_last_anom_seen_ms)  < ANOM_HOLD_MS;

  if (anom_recent)       setMode(MODE_ANOM);
  else if (human_recent) setMode(MODE_HUMAN);
  else                   setMode(MODE_IDLE);

  if (!g_have_status) setMode(MODE_IDLE);

  if (g_mode != g_last_drawn_mode) {
    if (g_mode == MODE_IDLE || g_mode == MODE_HUMAN) {
      drawHumanPage();
    } else {
      drawBird(true, "Anomaly Detected!");
    }
    g_last_drawn_mode = g_mode;
  }

  if (g_mode == MODE_IDLE || g_mode == MODE_HUMAN) {
    static uint32_t last_refresh = 0;

    if (now - g_page_start_ms >= g_page_dwell_ms) {
      schedulePage(pickNext(g_page, g_mode));
      drawHumanPage();
      last_refresh = now;
    } else if (now - last_refresh >= 1000) {
      drawHumanPage();
      last_refresh = now;
    }
  } else {
    static uint32_t lastTick = 0;
    if (now - lastTick >= 1000) {
      lastTick = now;
      drawBird(true, "Anomaly Detected!");
    }
  }

  delay(20);
}
