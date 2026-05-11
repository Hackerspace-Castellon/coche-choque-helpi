#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_NeoPixel.h>

static const int melody[] = {
  392, 440, 494, 440,
  392, 440, 494, 440,
  392, 440, 494, 440,
  392, 440,
  494, 440, 392, 440,
  494, 440, 392, 440,
  494, 440, 392, 440,
  494, 440, 392, 440,
  494
};

static const int durations[] = {
  63, 63, 63, 63,
  63, 63, 63, 63,
  63, 63, 63, 63,
  63, 63,
  63, 63, 63, 63,
  63, 63, 63, 63,
  63, 63, 63, 63,
  63, 63, 63, 63,
  63
};

static const int melodyLength = sizeof(melody) / sizeof(melody[0]);

// ===================== Ajustes =====================
static const uint8_t ESPNOW_CHANNEL = 6;   // Debe coincidir con el mando
static const uint32_t FAILSAFE_MS = 300;   // Si no llegan controles, frena
static const uint32_t LINK_LOST_MS = 1200; // Si se pierde enlace, volver a emparejar
static const uint32_t SEARCH_BLINK_MS = 250;

// DRV8833
static const int PIN_IN1 = 33;  // Motor A
static const int PIN_IN2 = 25;  // Motor A
static const int PIN_IN3 = 26;  // Motor B
static const int PIN_IN4 = 27;  // Motor B

// WS2812B coche
static const int PIN_LED = 21;
static const int LED_COUNT = 2;
Adafruit_NeoPixel strip(LED_COUNT, PIN_LED, NEO_GRB + NEO_KHZ800);

// Buzzer
static const int PIN_BUZZ_S = 22;

// PWM
static const int PWM_FREQ = 20000;
static const int PWM_BITS = 8;

// ===================== Protocolo =====================
enum MsgType : uint8_t {
  MSG_DISCOVER = 1,
  MSG_OFFER    = 2,
  MSG_PAIR_REQ = 3,
  MSG_PAIR_ACK = 4,
  MSG_CONTROL  = 5
};

#pragma pack(push, 1)
struct Msg {
  uint8_t  type;
  uint8_t  version;
  uint16_t device_id;
  uint32_t seq;
  int16_t  x;       // -1000..1000
  int16_t  y;       // -1000..1000
  uint16_t flags;
  uint8_t  r, g, b;
};
#pragma pack(pop)

// ===================== Estado =====================
uint8_t gCtrlMac[6] = {0};
bool gPaired = false;

uint8_t gR = 0, gG = 0, gB = 0;
unsigned long tLastControl = 0;
unsigned long tSearchBlink = 0;
bool gSearchLedOn = false;

volatile bool gMusicEnabled = false;
bool gJoyPressedPrev = false;
TaskHandle_t gMusicTask = nullptr;

static void buzzerOff();
static void musicTaskMain(void* parameter);

uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ===================== Helpers =====================
static bool macEquals(const uint8_t* a, const uint8_t* b) {
  for (int i = 0; i < 6; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

static int clampi(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static void addPeer(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) return;

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;

  esp_err_t result = esp_now_add_peer(&peer);
  if (result != ESP_OK) {
    Serial.print("Error al añadir peer: ");
    Serial.println(result);
  }
}

static void printMac(const uint8_t* mac) {
  for (int i = 0; i < 6; i++) {
    if (i > 0) Serial.print(":");
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
  }
  Serial.println();
}

// ===================== LEDs =====================
static void setAllLeds(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

static void ledsSearching() {
  unsigned long now = millis();
  if (now - tSearchBlink >= SEARCH_BLINK_MS) {
    tSearchBlink = now;
    gSearchLedOn = !gSearchLedOn;
  }

  if (gSearchLedOn) {
    setAllLeds(40, 40, 40);
  } else {
    setAllLeds(0, 0, 0);
  }
}

static void ledsPaired() {
  setAllLeds(gR, gG, gB);
}

static void ledsFailsafe() {
  setAllLeds(40, 0, 0);
}

// ===================== PWM =====================
static void pwmInitPin(int pin, int freq, int bits) {
  ledcAttach(pin, freq, bits);
  ledcWrite(pin, 0);
}

static void pwmWritePin(int pin, int duty) {
  const int maxDuty = (1 << PWM_BITS) - 1;
  duty = clampi(duty, 0, maxDuty);
  ledcWrite(pin, duty);
}

// ===================== Motores =====================
static void setupMotorPwm() {
  pwmInitPin(PIN_IN1, PWM_FREQ, PWM_BITS);
  pwmInitPin(PIN_IN2, PWM_FREQ, PWM_BITS);
  pwmInitPin(PIN_IN3, PWM_FREQ, PWM_BITS);
  pwmInitPin(PIN_IN4, PWM_FREQ, PWM_BITS);
}

static void motorWritePins(int pinFwd, int pinRev, int speed) {
  speed = clampi(speed, -255, 255);

  if (speed > 0) {
    pwmWritePin(pinFwd, speed);
    pwmWritePin(pinRev, 0);
  } else if (speed < 0) {
    pwmWritePin(pinFwd, 0);
    pwmWritePin(pinRev, -speed);
  } else {
    pwmWritePin(pinFwd, 0);
    pwmWritePin(pinRev, 0);
  }
}

static void stopMotors() {
  pwmWritePin(PIN_IN1, 0);
  pwmWritePin(PIN_IN2, 0);
  pwmWritePin(PIN_IN3, 0);
  pwmWritePin(PIN_IN4, 0);
}

static void resetToPairingMode() {
  gPaired = false;
  memset(gCtrlMac, 0, sizeof(gCtrlMac));
  gMusicEnabled = false;
  gJoyPressedPrev = false;
  buzzerOff();
  gSearchLedOn = false;
  tSearchBlink = millis();
  stopMotors();
  ledsSearching();
  Serial.println("Conexion con mando perdida. Volviendo a emparejamiento...");
}

// Mezcla de joystick
static void driveFromXY(int16_t x, int16_t y) {
  int throttle = (int)y;
  int steer = (int)x;

  int left = throttle + steer;
  int right = throttle - steer;

  left = (left * 255) / 1000;
  right = (right * 255) / 1000;

  left = clampi(left, -255, 255);
  right = clampi(right, -255, 255);

  // Motor A
  motorWritePins(PIN_IN1, PIN_IN2, left);
  // Motor B
  motorWritePins(PIN_IN3, PIN_IN4, right);
}

// ===================== Buzzer =====================
static void setupBuzzer() {
  pinMode(PIN_BUZZ_S, OUTPUT);
  noTone(PIN_BUZZ_S);
}

static void buzzerOff() {
  noTone(PIN_BUZZ_S);
}

static void buzzerTone(uint16_t freq) {
  if (freq == 0) {
    buzzerOff();
    return;
  }

  tone(PIN_BUZZ_S, freq);
}

static void beep(int ms = 80) {
  tone(PIN_BUZZ_S, 2000);
  delay(ms);
  buzzerOff();
}

static void updateMusic() {
  if (!gMusicEnabled) {
    buzzerOff();
    return;
  }

  if (gMusicTask == nullptr) {
    xTaskCreatePinnedToCore(musicTaskMain, "musicTask", 4096, nullptr, 1, &gMusicTask, 1);
  }
}

static void musicTaskMain(void* parameter) {
  (void)parameter;

  while (gMusicEnabled) {
    for (int i = 0; i < melodyLength && gMusicEnabled; i++) {
      tone(PIN_BUZZ_S, melody[i], durations[i]);
      vTaskDelay(pdMS_TO_TICKS((int)(durations[i] * 1.15)));
      noTone(PIN_BUZZ_S);
    }
  }

  buzzerOff();
  gMusicTask = nullptr;
  vTaskDelete(nullptr);
}

// ===================== Emparejado simple =====================
static void pairWithController(const uint8_t* mac, const Msg& msg) {
  memcpy(gCtrlMac, mac, 6);
  gPaired = true;
  tLastControl = millis();

  gR = msg.r;
  gG = msg.g;
  gB = msg.b;

  addPeer(gCtrlMac);

  Serial.print("Mando emparejado con MAC: ");
  printMac(gCtrlMac);

  ledsPaired();
  beep(120);
}

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != (int)sizeof(Msg)) return;

  Msg msg;
  memcpy(&msg, data, sizeof(Msg));

  const uint8_t* src = info->src_addr;

  // =====================
  // NO EMPAREJADO
  // =====================
  if (!gPaired) {

    // 1. DISCOVER → responder OFFER
    if (msg.type == MSG_DISCOVER) {

      addPeer(src);

      Msg offer{};
      offer.type = MSG_OFFER;
      offer.version = 1;
      offer.device_id = 0xCAFE;
      offer.r = msg.r;
      offer.g = msg.g;
      offer.b = msg.b;

      esp_now_send((uint8_t*)src, (uint8_t*)&offer, sizeof(offer));

      return;
    }

    // 2. PAIR_REQ → emparejar
    if (msg.type == MSG_PAIR_REQ) {

      memcpy(gCtrlMac, src, 6);
      gPaired = true;
      tLastControl = millis();

      gR = msg.r;
      gG = msg.g;
      gB = msg.b;

      addPeer(gCtrlMac);

      Serial.print("Emparejado con: ");
      for (int i = 0; i < 6; i++) {
        if (i) Serial.print(":");
        Serial.print(gCtrlMac[i], HEX);
      }
      Serial.println();

      setAllLeds(gR, gG, gB);
      beep(120);

      Msg ack{};
      ack.type = MSG_PAIR_ACK;
      ack.version = 1;
      ack.device_id = 0xCAFE;
      ack.r = gR;
      ack.g = gG;
      ack.b = gB;

      esp_now_send(gCtrlMac, (uint8_t*)&ack, sizeof(ack));

      return;
    }

    return;
  }

  // =====================
  // EMPAREJADO
  // =====================
  if (!macEquals(src, gCtrlMac)) return;

  if (msg.type == MSG_CONTROL) {
    tLastControl = millis();

    driveFromXY(msg.x, msg.y);

    bool joyPressed = (msg.flags & 0x0001) != 0;
    if (joyPressed && !gJoyPressedPrev) {
      gMusicEnabled = !gMusicEnabled;

      if (!gMusicEnabled) {
        buzzerOff();
      } else {
        updateMusic();
      }

      Serial.print("Musica ");
      Serial.println(gMusicEnabled ? "ON" : "OFF");
    }
    gJoyPressedPrev = joyPressed;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // LED
  strip.begin();
  strip.setBrightness(80);
  strip.show();

  // Motores y buzzer
  setupMotorPwm();
  setupBuzzer();
  stopMotors();
  ledsSearching();

  // WiFi / ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: esp_now_init()");
    while (true) {
      delay(1000);
    }
  }

  esp_now_register_recv_cb(onRecv);

  addPeer(BROADCAST_MAC);

  gPaired = false;
  memset(gCtrlMac, 0, sizeof(gCtrlMac));
  gMusicEnabled = false;
  gJoyPressedPrev = false;
  buzzerOff();
  tSearchBlink = millis();
  gSearchLedOn = false;

  Serial.print("MAC coche: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Esperando mando...");
}

void loop() {
  if (!gPaired) {
    stopMotors();
    ledsSearching();
    delay(10);
    return;
  }

  unsigned long idleMs = millis() - tLastControl;

  if (idleMs > LINK_LOST_MS) {
    resetToPairingMode();
    delay(10);
    return;
  }

  if (idleMs > FAILSAFE_MS) {
    stopMotors();
    ledsFailsafe();
  } else {
    ledsPaired();
  }

  updateMusic();

  delay(10);
}