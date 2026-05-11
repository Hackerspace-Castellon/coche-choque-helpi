#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_NeoPixel.h>

// ====== CONFIG ======
static const uint8_t ESPNOW_CHANNEL = 6;
static const uint32_t DISCOVER_INTERVAL_MS = 150;
static const uint32_t SEND_INTERVAL_MS = 20;
static const uint32_t BLINK_INTERVAL_MS = 250;
static const uint32_t LINK_LOST_MS = 1200;

// Joystick
static const int PIN_VRX = 34;
static const int PIN_VRY = 35;
static const int PIN_SW  = 32;

// LED / tira del mando
static const int PIN_LED = 23;
static const int NUM_LEDS = 2;

Adafruit_NeoPixel strip(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);

// ====== PROTOCOLO ======
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
  int16_t  x;
  int16_t  y;
  uint16_t flags;
  uint8_t  r, g, b;
};
#pragma pack(pop)

static const uint16_t CONTROLLER_ID = 0x1234;

uint32_t gSeq = 0;
uint8_t gCarMac[6] = {0};
bool gPaired = false;

uint8_t gR = 0, gG = 0, gB = 0;

unsigned long tLastDiscover = 0;
unsigned long tLastSend = 0;
unsigned long tLastLinkOk = 0;

// Blink búsqueda
unsigned long tLastBlink = 0;
bool blinkOn = false;

uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static void resetToPairingMode() {
  gPaired = false;
  memset(gCarMac, 0, sizeof(gCarMac));
  tLastLinkOk = 0;
  tLastBlink = millis();
  blinkOn = false;
  setLed(0, 0, 0);
  Serial.println("Conexion con coche perdida. Volviendo a emparejamiento...");
}

// ====== FUNCIONES ======
static void setLed(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

static void pickRandomColor() {
  uint16_t hue = (uint16_t)(esp_random() & 0xFFFF);
  uint32_t color = Adafruit_NeoPixel::ColorHSV(hue, 255, 180);
  gR = (color >> 16) & 0xFF;
  gG = (color >> 8) & 0xFF;
  gB = color & 0xFF;
}

static int16_t readAxis(int pin) {
  int v = analogRead(pin);
  int centered = v - 2048;

  if (abs(centered) < 40) centered = 0;

  int32_t scaled = ((int32_t)centered * 1000) / 2048;

  if (scaled > 1000) scaled = 1000;
  if (scaled < -1000) scaled = -1000;

  return (int16_t)scaled;
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

// ====== RECEPCIÓN ======
void onSent(const esp_now_send_info_t* tx_info, esp_now_send_status_t status) {
  if (!gPaired) return;
  if (!tx_info || !tx_info->des_addr) return;
  if (memcmp(tx_info->des_addr, gCarMac, 6) != 0) return;

  if (status == ESP_NOW_SEND_SUCCESS) {
    tLastLinkOk = millis();
  }
}

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != (int)sizeof(Msg)) return;

  Msg msg;
  memcpy(&msg, data, sizeof(Msg));

  const uint8_t* src = info->src_addr;

  // Mientras no esté emparejado:
  if (!gPaired) {
    if (msg.type == MSG_OFFER) {
      addPeer(src);

      Msg req{};
      req.type = MSG_PAIR_REQ;
      req.version = 1;
      req.device_id = CONTROLLER_ID;
      req.seq = gSeq++;
      req.r = gR;
      req.g = gG;
      req.b = gB;

      esp_now_send((uint8_t*)src, (uint8_t*)&req, sizeof(req));
      return;
    }

    if (msg.type == MSG_PAIR_ACK) {
      memcpy(gCarMac, src, 6);
      gPaired = true;
      tLastLinkOk = millis();
      addPeer(gCarMac);

      Serial.print("Coche emparejado con MAC: ");
      printMac(gCarMac);

      setLed(gR, gG, gB);
      return;
    }

    return;
  }

  // Ya emparejado: ignoramos otros coches
  if (memcmp(src, gCarMac, 6) != 0) return;

  // No hace falta hacer nada más aquí
}

// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_SW, INPUT_PULLUP);

  strip.begin();
  strip.setBrightness(80);
  strip.show();

  // Al arrancar, empieza buscando
  tLastBlink = millis();
  blinkOn = false;
  setLed(0, 0, 0);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error iniciando ESP-NOW");
    while (true) {
      delay(1000);
    }
  }

  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  addPeer(BROADCAST_MAC);

  pickRandomColor();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Serial.print("MAC mando: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Buscando coche...");
}

// ====== LOOP ======
void loop() {
  unsigned long now = millis();

  // 1) LED: blanco parpadeando mientras busca
  if (!gPaired) {
    if (now - tLastBlink >= BLINK_INTERVAL_MS) {
      tLastBlink = now;
      blinkOn = !blinkOn;

      if (blinkOn) setLed(255, 255, 255);
      else         setLed(0, 0, 0);
    }
  }

  // 2) Descubrimiento mientras no esté emparejado
  if (!gPaired && (now - tLastDiscover >= DISCOVER_INTERVAL_MS)) {
    tLastDiscover = now;

    Msg d{};
    d.type = MSG_DISCOVER;
    d.version = 1;
    d.device_id = CONTROLLER_ID;
    d.seq = gSeq++;
    d.r = gR;
    d.g = gG;
    d.b = gB;

    esp_now_send(BROADCAST_MAC, (uint8_t*)&d, sizeof(d));
  }

  if (gPaired && (now - tLastLinkOk > LINK_LOST_MS)) {
    resetToPairingMode();
  }

  // 3) Envío de controles cuando ya está emparejado
  if (gPaired && (now - tLastSend >= SEND_INTERVAL_MS)) {
    tLastSend = now;

    Msg c{};
    c.type = MSG_CONTROL;
    c.version = 1;
    c.device_id = CONTROLLER_ID;
    c.seq = gSeq++;
    c.x = readAxis(PIN_VRX);
    c.y = readAxis(PIN_VRY);
    c.flags = (digitalRead(PIN_SW) == LOW) ? 1 : 0;
    c.r = gR;
    c.g = gG;
    c.b = gB;

    esp_now_send(gCarMac, (uint8_t*)&c, sizeof(c));
  }

  // 4) Mantener color fijo cuando ya está emparejado
  if (gPaired) {
    setLed(gR, gG, gB);
  }

  delay(5);
}