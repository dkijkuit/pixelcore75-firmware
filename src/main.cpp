#include <Arduino.h>

// **************************************
// FLASH SETTINGS for HUIDU WF2
//
// BOARD: WiFiduino32S3
// USB CDC on boot: ENABLED
// Erase all flash before sketch upload: ENABLE (only the first time, afterwards disable)
// Flash mode: DIO 80 Mhz
// Flash size: 8MB (64Mb)
// Partition scheme: Huge App (3MB no OTA/1MB SPIFFS)
// PSRSAM: Disabled
// **************************************

#include <WiFi.h>
#include <WiFiMulti.h>

#include <PubSubClient.h>

#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <Fonts/Org_01.h>

#include <FS.h>
#include <LittleFS.h>

#include <Preferences.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include "images.h"
#include "image_utils.h"

#define PANEL_RES_X 64 // Number of pixels wide of each INDIVIDUAL panel module.
#define PANEL_RES_Y 32 // Number of pixels tall of each INDIVIDUAL panel module.
#define PANEL_CHAIN 1  // Total number of panels chained one to another

#define WF2_X1_R1_PIN 2
#define WF2_X1_R2_PIN 3
#define WF2_X1_G1_PIN 6
#define WF2_X1_G2_PIN 7
#define WF2_X1_B1_PIN 10
#define WF2_X1_B2_PIN 11
#define WF2_X1_E_PIN 21

#define WF2_X2_R1_PIN 4
#define WF2_X2_R2_PIN 5
#define WF2_X2_G1_PIN 8
#define WF2_X2_G2_PIN 9
#define WF2_X2_B1_PIN 12
#define WF2_X2_B2_PIN 13
#define WF2_X2_E_PIN -1 // Currently unknown, so X2 port will not work (yet) with 1/32 scan panels

#define WF2_A_PIN 39
#define WF2_B_PIN 38
#define WF2_C_PIN 37
#define WF2_D_PIN 36
#define WF2_OE_PIN 35
#define WF2_CLK_PIN 34
#define WF2_LAT_PIN 33

#define WF2_BUTTON_TEST 17    // Test key button on PCB, 1=normal, 0=pressed
#define WF2_LED_RUN_PIN 40    // Status LED on PCB
#define WF2_BM8563_I2C_SDA 41 // RTC BM8563 I2C port
#define WF2_BM8563_I2C_SCL 42
#define WF2_USB_DM_PIN 19
#define WF2_USB_DP_PIN 20

#define MAX_PAYLOAD_SIZE 16384
#define MAX_VALUES 8192

#define FORMAT_LITTLE_FS_IF_FAILED true

#define SERVICE_UUID "975a3183-e5f1-448a-acab-2016d89c1fe7"
#define CHARACTERISTIC_UUID_SERVER "38487a5b-f731-4118-bf66-4ee253d5f664"
#define CHARACTERISTIC_UUID_SERVER_PORT "e67e6360-99f3-4c6b-8e60-2e9266100718"
#define CHARACTERISTIC_UUID_WIFI_SSID "7a034f21-a679-4d51-a284-e6b4b69ceea9"
#define CHARACTERISTIC_UUID_WIFI_PASSWORD "3f007796-2fd1-42d2-b122-458f1f0b90bf"
#define CHARACTERISTIC_UUID_BRIGHTNESS "a7423ece-dced-4fb2-ac67-ddf97323726b"
#define CHARACTERISTIC_UUID_RESTART "81ed8290-f167-47b9-b183-2f248c543889"

#define BUILD_NAME "PixelCore75"
#define BUILD_VERSION "V.0.0.1"

HUB75_I2S_CFG::i2s_pins _pins_x1 = {WF2_X1_R1_PIN, WF2_X1_G1_PIN, WF2_X1_B1_PIN, WF2_X1_R2_PIN, WF2_X1_G2_PIN, WF2_X1_B2_PIN, WF2_A_PIN, WF2_B_PIN, WF2_C_PIN, WF2_D_PIN, WF2_X1_E_PIN, WF2_LAT_PIN, WF2_OE_PIN, WF2_CLK_PIN};
HUB75_I2S_CFG::i2s_pins _pins_x2 = {WF2_X2_R1_PIN, WF2_X2_G1_PIN, WF2_X2_B1_PIN, WF2_X2_R2_PIN, WF2_X2_G2_PIN, WF2_X2_B2_PIN, WF2_A_PIN, WF2_B_PIN, WF2_C_PIN, WF2_D_PIN, WF2_X2_E_PIN, WF2_LAT_PIN, WF2_OE_PIN, WF2_CLK_PIN};

MatrixPanel_I2S_DMA *dma_display = nullptr;

WiFiMulti wiFiMulti;
Preferences preferences;
WiFiClient espClient;
PubSubClient client(espClient);
ScreenImage currentScreenImage = ScreenImage::None;

bool hasWifi = false;
bool updateScreen = false;
bool deviceConnected = false;
bool bluetoothInitCompleted = false;
bool buttonPressed = false;

unsigned long lastReconnectAttempt = 0;

static constexpr int W = 64;
static constexpr int H = 32;
static constexpr size_t FRAME_BYTES = W * H * 2;

static constexpr uint32_t ANIM_MAGIC = 0x4D494E41;       // "ANIM" little-endian
static constexpr uint32_t ANIM_FRAME_MAGIC = 0x46494E41; // "ANIF" little-endian
static constexpr uint32_t ANIM_PLAY_MAGIC = 0x50494E41;  // "ANIP" little-endian
static constexpr const char *TOPIC_ANIM_START = "/anim/start";
static constexpr const char *TOPIC_ANIM_FRAME = "/anim/frame";
static constexpr const char *TOPIC_ANIM_PLAY = "/anim/play";
static constexpr const char *TOPIC_ANIM_LOADED = "/anim/loaded";
static constexpr const char *ANIM_UPLOAD_PATH = "/anim_up.bin"; // staging file while an upload is in flight
static constexpr uint8_t ANIM_MAX_SLOTS = 32; // persistent animation slots; capacity-bound, eviction self-heals via re-upload
static constexpr size_t ANIM_FILE_HEADER_BYTES = 4; // frameCount(u16) + delayMs(u16)
static constexpr size_t ANIM_START_PAYLOAD = 14;    // magic + frameCount + delayMs + uploadId + flags + slot
static constexpr size_t ANIM_FRAME_PAYLOAD = 6 + FRAME_BYTES; // magic + frameIdx + pixels
static constexpr size_t ANIM_PLAY_PAYLOAD = 9;      // magic + slot + uploadId
static constexpr size_t ANIM_LOADED_PAYLOAD = 9;    // magic + slot + uploadId
static constexpr uint8_t ANIM_FLAG_STAGE_ONLY = 0x01; // stage to flash but wait for /anim/play
static constexpr uint16_t MAX_ANIM_FRAMES = 200;    // sanity cap; ~800KB fits the ~4.9MB LittleFS many times over
static constexpr uint16_t ANIM_MIN_DELAY_MS = 10;
static constexpr size_t ANIM_RING_FRAMES = 8;  // RAM frame ring (8*4KB): decouples MQTT from flash writes
static constexpr size_t ANIM_FLUSH_CHUNK = 256; // one flash page per loop pass while an animation plays

uint8_t rxBuf[FRAME_BYTES];       // raw bytes from MQTT
uint16_t *px = (uint16_t *)rxBuf; // view as RGB565 pixels (little-endian)

bool animActive = false;
bool animUploading = false;
bool animLoadedAckPending = false;
uint32_t animUploadId = 0;
bool animStageOnly = false;     // current upload stages but doesn't play on completion
uint8_t animSlot = 0;           // slot the current upload writes to
uint8_t animPlayingSlot = 0;    // slot currently playing (auto-resume after reconnect/reboot)
uint8_t animAckSlot = 0;        // ack payload while animLoadedAckPending
uint32_t animAckUploadId = 0;   // ack payload while animLoadedAckPending
uint32_t animPlayRequestId = 0; // /anim/play that arrived while the upload was still running
uint16_t animUploadCount = 0;
uint16_t animExpectedIdx = 0;
uint16_t animFrameCount = 0;
uint16_t animFrameIdx = 0;
uint16_t animDelayMs = 0;
unsigned long animLastFrameMs = 0;
uint8_t animBuf[FRAME_BYTES];
File animFile;
File animUploadFile; // staging file, held open for the whole upload (per-frame open/close starves the animation loop)

// Upload ring: the MQTT callback only copies frames into RAM; loop() flushes them to
// flash in small budgeted chunks. Flash writes stall both cores (page programs ~0.5ms,
// block erases tens of ms), so doing them inline in the callback froze the animation
// tick for the whole 4KB frame. Backpressure: loop() stops calling client.loop() when
// the ring is nearly full, so TCP/MQTT flow control throttles the server instead.
uint8_t animRing[ANIM_RING_FRAMES][FRAME_BYTES];
size_t animRingHead = 0;       // next free frame (writer: MQTT callback)
size_t animRingTail = 0;       // oldest buffered frame (reader: loop flusher)
size_t animRingCount = 0;      // frames buffered
size_t animRingFlushed = 0;    // bytes of the tail frame already written to flash

bool init_wifi(char ssid[], char password[]);
void onConnect(BLEServer *pServer);
void onDisconnect(BLEServer *pServer);
void init_bluetooth();
void setAsciiValue(BLECharacteristic *ch, const String &val);
void callback(char *topic, byte *payload, unsigned int length);
void init_broker_connection();
void writeFile(fs::FS &fs, const char *path, const uint16_t *intArray);
void readFile(fs::FS &fs, const char *path);
void reconnect();
void init_display();
void verifyRegistration();
bool check_bluetooth_button_pressed();
const char *getClientId();
void stopAnimation(bool removeFile);
void abortAnimUpload();
void playSlot(uint8_t slot, uint32_t uploadId);
void startAnimation(uint8_t slot);
void animationTick();
void animUploadFlush();
void completeAnimUpload();
void handleAnimStart(const byte *payload, unsigned int length);
void handleAnimFrame(const byte *payload, unsigned int length);
void handleAnimPlay(const byte *payload, unsigned int length);

void drawBitmap(
    int16_t x,
    int16_t y,
    const uint16_t *bitmap,
    int16_t width,
    int16_t height,
    ScreenImage imageId,
    bool clearScreen
  )
{
  if (currentScreenImage != imageId || currentScreenImage == ScreenImage::Client)
  {
    if(clearScreen) dma_display->clearScreen();
    dma_display->drawRGBBitmap(x, y, bitmap, width, height);
    currentScreenImage = imageId;
  }
}

bool init_wifi(char ssid[], char password[])
{
  dma_display->clearScreen();

  currentScreenImage = ScreenImage::Wifi;
  drawXbm565(dma_display, 0, 0, 64, 32, wifi_image1bit, dma_display->color565(0, 0, 255));

  delay(1000);

  wiFiMulti.addAP(ssid, password);

  Serial.println();
  Serial.println();
  Serial.print("Waiting for WiFi... ");

  const unsigned long wifiStartMs = millis();
  while (wiFiMulti.run() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);

    if (millis() - wifiStartMs > 15000)
      break;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("Wifi connected!");
    hasWifi = true;
    drawBitmap(38, 23, epd_bitmap_check_mark, 8, 8, ScreenImage::Checkmark, false);
    delay(3000);
  }
  else
  {
    hasWifi = false;
    Serial.println("ERROR: Failed to connect to wifi!");
    drawBitmap(16, 0, epd_bitmap_bluetooth_icon, 32, 32, ScreenImage::Bluetooth, true);
    drawBitmap(40, 20, epd_bitmap_search, 8, 8, ScreenImage::Search, false);

    init_bluetooth();
  }

  delay(1000);

  return hasWifi;
}

class ServerCharacteristicCallBack : public BLECharacteristicCallbacks
{
public:
  void onWrite(BLECharacteristic *characteristic)
  {
    String value = String(characteristic->getValue().c_str());
    value.trim();

    byte serverBytes[value.length() + 1];
    value.getBytes(serverBytes, value.length() + 1);
    preferences.putBytes("server", serverBytes, value.length() + 1);
    Serial.println("Server received: " + value);
  }
};

class SsidCharacteristicCallBack : public BLECharacteristicCallbacks
{
public:
  void onWrite(BLECharacteristic *characteristic)
  {
    // Handle the written data here
    String value = String(characteristic->getValue().c_str());
    value.trim();

    byte ssidBytes[value.length() + 1];
    value.getBytes(ssidBytes, value.length() + 1);
    preferences.putBytes("wifissid", ssidBytes, value.length() + 1);
    Serial.println("SSID received: " + value);
  }
};

class PasswordCharacteristicCallBack : public BLECharacteristicCallbacks
{
public:
  void onWrite(BLECharacteristic *characteristic)
  {
    String value = String(characteristic->getValue().c_str());
    value.trim();

    byte passBytes[value.length() + 1];
    value.getBytes(passBytes, value.length() + 1);
    preferences.putBytes("wifipass", passBytes, value.length() + 1);
    Serial.println("Password received: " + value);
  }
};

class BrightnessCharacteristicCallBack : public BLECharacteristicCallbacks
{
public:
  void onWrite(BLECharacteristic *characteristic) override
  {
    // The standard ESP32 BLE lib often returns Arduino String here
    String raw = String(characteristic->getValue().c_str());

    Serial.printf("Brightness write: %d bytes\n", raw.length());

    if (raw.length() == 0)
    {
      Serial.println("Brightness write was empty; ignoring.");
      return;
    }

    // Make a trimmed copy for ASCII parsing, but keep 'raw' for raw-byte fallback
    String s = raw;
    s.trim();

    uint16_t value = 0;
    if (s.length() > 0 && isAllDigits(s))
    {
      // App sends ASCII decimal: "10".."255"
      value = (uint16_t)s.toInt();
      Serial.printf("Parsed ASCII brightness: %u\n", (unsigned)value);
    }
    else
    {
      // Fallback: treat first byte as the value (in case of raw write)
      value = static_cast<uint8_t>(raw[0]);
      Serial.printf("Parsed RAW brightness (first byte): %u\n", (unsigned)value);
    }

    // Clamp to your allowed range
    if (value < 10)
      value = 10;
    if (value > 255)
      value = 255;

    preferences.putUChar("brightness", static_cast<uint8_t>(value));
    Serial.printf("Brightness received (final): %u\n", (unsigned)value);
    dma_display->setBrightness(static_cast<uint8_t>(value));
  }

private:
  static bool isAllDigits(const String &s)
  {
    for (size_t i = 0; i < s.length(); ++i)
    {
      if (!isDigit(static_cast<unsigned char>(s[i])))
        return false;
    }
    return true;
  }
};

class ServerPortCharacteristicCallBack : public BLECharacteristicCallbacks
{
public:
  void onWrite(BLECharacteristic *characteristic) override
  {
    // getValue() can be Arduino String or std::string depending on version.
    // Convert robustly to Arduino String via c_str().
    String raw;
    {
      auto v = characteristic->getValue(); // String OR std::string
      raw = String(v.c_str());             // make an owned Arduino String
    }

    Serial.printf("ServerPort write: %d bytes\n", raw.length());
    if (raw.length() == 0)
    {
      Serial.println("Port value empty; ignoring.");
      return;
    }

    // Trim for ASCII parsing
    String s = raw;
    s.trim();

    uint32_t port = 0;
    if (s.length() > 0 && isAllDigits(s))
    {
      // App path: ASCII decimal, e.g. "1883"
      port = (uint32_t)s.toInt();
      Serial.printf("Parsed ASCII port: %u\n", (unsigned)port);
    }
    else
    {
      // Fallbacks for non-ASCII writes:
      if (raw.length() >= 2)
      {
        // Interpret first two bytes as network-order (big endian) uint16
        uint8_t b0 = (uint8_t)raw[0];
        uint8_t b1 = (uint8_t)raw[1];
        port = ((uint16_t)b0 << 8) | b1;
        Serial.printf("Parsed RAW 2-byte port (network order): %u\n", (unsigned)port);
      }
      else
      {
        // Single byte fallback
        port = (uint8_t)raw[0];
        Serial.printf("Parsed RAW 1-byte port: %u\n", (unsigned)port);
      }
    }

    // Clamp to valid TCP port range
    if (port < 1)
      port = 1;
    if (port > 65535)
      port = 65535;

    // Persist + apply
    preferences.putUShort("server_port", (uint16_t)port);
    Serial.printf("Server port saved: %u\n", (unsigned)port);
  }

private:
  static bool isAllDigits(const String &s)
  {
    for (size_t i = 0; i < s.length(); ++i)
    {
      if (!isDigit((unsigned char)s[i]))
        return false;
    }
    return true;
  }
};

class RestartCharacteristicCallBack : public BLECharacteristicCallbacks
{
public:
  void onWrite(BLECharacteristic *characteristic)
  {
    ESP.restart();
  }
};

class BLEConnectionCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("***** Connect");

    drawBitmap(16, 0, epd_bitmap_bluetooth_icon, 32, 32, ScreenImage::Bluetooth, true);
    drawBitmap(40, 20, epd_bitmap_check_mark, 8, 8, ScreenImage::Checkmark, false);
  }
  void onDisconnect(BLEServer *pServer)
  {
    Serial.println("***** Disconnect");
    deviceConnected = false;
    updateScreen = true;
    drawBitmap(16, 0, epd_bitmap_bluetooth_icon, 32, 32, ScreenImage::Bluetooth, true);
    drawBitmap(40, 20, epd_bitmap_red_cross, 7, 7, ScreenImage::Cross, false);

    delay(2000);

    BLEDevice::stopAdvertising();

    ESP.restart();
  }
};

void init_bluetooth()
{
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  Serial.println("No WiFi configured, switching on bluetooth for configuration...");
  dma_display->clearScreen();
  drawBitmap(16, 0, epd_bitmap_bluetooth_icon, 32, 32, ScreenImage::Bluetooth, true);
  drawBitmap(40, 20, epd_bitmap_search, 8, 8, ScreenImage::Search, false);
  updateScreen = false;

  if (!bluetoothInitCompleted)
  {
    BLEDevice::init("PixelCore75");
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID), 32, 0);

    BLECharacteristic *serverCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_SERVER, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    BLEDescriptor *serverDesc = new BLEDescriptor((uint16_t)0x2901);
    serverDesc->setValue("Server");
    serverCharacteristic->addDescriptor(serverDesc);

    BLECharacteristic *serverPortCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_SERVER_PORT, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    BLEDescriptor *serverPortDesc = new BLEDescriptor((uint16_t)0x2901);
    serverPortDesc->setValue("Port (1000-65434)");
    serverPortCharacteristic->addDescriptor(serverPortDesc);

    BLECharacteristic *ssidCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_WIFI_SSID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    BLEDescriptor *ssidDesc = new BLEDescriptor((uint16_t)0x2901);
    ssidDesc->setValue("Wifi SSID");
    ssidCharacteristic->addDescriptor(ssidDesc);

    BLECharacteristic *passwordCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_WIFI_PASSWORD, BLECharacteristic::PROPERTY_WRITE);
    BLEDescriptor *passDesc = new BLEDescriptor((uint16_t)0x2901);
    passDesc->setValue("Wifi password");
    passwordCharacteristic->addDescriptor(passDesc);

    BLECharacteristic *brightnessCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_BRIGHTNESS, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    BLEDescriptor *brightnessDesc = new BLEDescriptor((uint16_t)0x2901);
    brightnessDesc->setValue("Brightness (0-255)");
    brightnessCharacteristic->addDescriptor(brightnessDesc);

    BLECharacteristic *restartCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RESTART, BLECharacteristic::PROPERTY_WRITE);
    BLEDescriptor *restartDesc = new BLEDescriptor((uint16_t)0x2901);
    restartDesc->setValue("Restart panel by writing any value");
    restartCharacteristic->addDescriptor(restartDesc);

    pServer->setCallbacks(new BLEConnectionCallbacks());
    serverCharacteristic->setCallbacks(new ServerCharacteristicCallBack());
    serverPortCharacteristic->setCallbacks(new ServerPortCharacteristicCallBack());
    ssidCharacteristic->setCallbacks(new SsidCharacteristicCallBack());
    passwordCharacteristic->setCallbacks(new PasswordCharacteristicCallBack());
    brightnessCharacteristic->setCallbacks(new BrightnessCharacteristicCallBack());
    restartCharacteristic->setCallbacks(new RestartCharacteristicCallBack());

    char ssid[preferences.getBytesLength("wifissid")] = {};
    preferences.getBytes("wifissid", ssid, preferences.getBytesLength("wifissid"));
    ssidCharacteristic->setValue(ssid);

    char server[preferences.getBytesLength("server")] = {};
    preferences.getBytes("server", server, preferences.getBytesLength("server"));
    serverCharacteristic->setValue(server);

    uint16_t port = preferences.getUShort("server_port", 1883);
    setAsciiValue(serverPortCharacteristic, String(port));

    uint8_t brightness = preferences.getUChar("brightness", 128);
    setAsciiValue(brightnessCharacteristic, String(brightness));

    restartCharacteristic->setValue("restart");

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
  }

  BLEDevice::startAdvertising();

  bluetoothInitCompleted = true;
}

void setAsciiValue(BLECharacteristic *ch, const String &val)
{
  ch->setValue((uint8_t *)val.c_str(), val.length());
}

void abortAnimUpload()
{
  animUploading = false;
  animStageOnly = false;
  animPlayRequestId = 0;
  animRingHead = animRingTail = animRingCount = 0;
  animRingFlushed = 0;
  if (animUploadFile)
    animUploadFile.close();
  LittleFS.remove(ANIM_UPLOAD_PATH);
}

/** Path of a slot file. Single static buffer: the firmware is single-threaded (MQTT callback
 *  runs inside client.loop() from loop()), and no two slot paths are held simultaneously. */
static const char *animSlotPath(uint8_t slot)
{
  static char buf[12]; // "/a31.bin" + NUL, with headroom
  snprintf(buf, sizeof(buf), "/a%u.bin", slot);
  return buf;
}

void playSlot(uint8_t slot, uint32_t uploadId)
{
  if (slot >= ANIM_MAX_SLOTS)
    return;

  Serial.printf("Animation play requested: slot %u\n", slot);
  startAnimation(slot);
  if (animActive)
  {
    animAckSlot = slot;
    animAckUploadId = uploadId;
    animLoadedAckPending = true;
  }
}

void stopAnimation(bool removePlayingSlot)
{
  bool wasUploading = animUploading;
  animActive = false;
  animUploading = false;
  animLoadedAckPending = false;
  if (animFile)
    animFile.close();
  if (removePlayingSlot)
    LittleFS.remove(animSlotPath(animPlayingSlot));
  if (wasUploading)
    abortAnimUpload(); // staging is stale once the upload state resets
}

void animationTick()
{
  if (!animActive)
    return;
  if (millis() - animLastFrameMs < animDelayMs)
    return;
  // Catch-up cadence: advance by exactly one delay so a stall (e.g. a flash erase
  // during an upload) holds one frame instead of permanently slowing the animation.
  animLastFrameMs += animDelayMs;
  if (millis() - animLastFrameMs >= animDelayMs)
    animLastFrameMs = millis(); // fell a full frame or more behind: resync, don't fast-forward

  size_t offset = ANIM_FILE_HEADER_BYTES + (size_t)animFrameIdx * FRAME_BYTES;
  if (!animFile.seek(offset) || animFile.read(animBuf, FRAME_BYTES) != FRAME_BYTES)
  {
    stopAnimation(true);
    return;
  }

  dma_display->drawRGBBitmap(0, 0, (uint16_t *)animBuf, W, H);
  currentScreenImage = ScreenImage::Client;
  animFrameIdx = (animFrameIdx + 1) % animFrameCount;
}

void startAnimation(uint8_t slot)
{
  if (slot >= ANIM_MAX_SLOTS)
    return;
  stopAnimation(false);

  const char *path = animSlotPath(slot);
  if (!LittleFS.exists(path))
    return;

  File f = LittleFS.open(path, FILE_READ);
  if (!f)
    return;

  uint8_t hdr[ANIM_FILE_HEADER_BYTES];
  bool valid = f.size() >= ANIM_FILE_HEADER_BYTES + FRAME_BYTES &&
               f.seek(0) &&
               f.read(hdr, ANIM_FILE_HEADER_BYTES) == ANIM_FILE_HEADER_BYTES;
  if (valid)
  {
    animFrameCount = hdr[0] | (hdr[1] << 8);
    animDelayMs = hdr[2] | (hdr[3] << 8);
    valid = animFrameCount >= 2 && animFrameCount <= MAX_ANIM_FRAMES &&
            f.size() == ANIM_FILE_HEADER_BYTES + (size_t)animFrameCount * FRAME_BYTES;
  }
  if (!valid)
  {
    f.close();
    LittleFS.remove(path);
    return;
  }

  if (animDelayMs < ANIM_MIN_DELAY_MS)
    animDelayMs = ANIM_MIN_DELAY_MS;

  animPlayingSlot = slot;
  animFile = f;
  animFrameIdx = 0;
  animLastFrameMs = 0;
  animActive = true;
  animationTick();
}

void handleAnimStart(const byte *payload, unsigned int length)
{
  if (length != ANIM_START_PAYLOAD)
    return;

  uint32_t magic;
  memcpy(&magic, payload, sizeof(magic));
  if (magic != ANIM_MAGIC)
    return;

  uint16_t count = payload[4] | (payload[5] << 8);
  uint16_t delayMs = payload[6] | (payload[7] << 8);
  uint32_t uploadId = (uint32_t)payload[8] | ((uint32_t)payload[9] << 8) |
                      ((uint32_t)payload[10] << 16) | ((uint32_t)payload[11] << 24);
  bool stageOnly = (payload[12] & ANIM_FLAG_STAGE_ONLY) != 0;
  uint8_t slot = payload[13];
  if (slot >= ANIM_MAX_SLOTS)
    return;
  if (count < 2 || count > MAX_ANIM_FRAMES)
    return;
  if (delayMs < ANIM_MIN_DELAY_MS)
    delayMs = ANIM_MIN_DELAY_MS;

  // Stage into a second file so the currently playing animation (or static screen) is
  // unaffected during the upload; the swap onto the slot file happens on the last frame
  // (immediate play) or later on /anim/play (stage-only uploads).
  if (animUploading)
    abortAnimUpload(); // superseded by a new upload: close the old handle before re-staging
  animPlayRequestId = 0;
  LittleFS.remove(ANIM_UPLOAD_PATH); // stale partial upload from an earlier attempt

  // Space ladder for the staging file: drop the slot file being replaced first, then idle
  // slot files, and only as a last resort the playing animation.
  size_t needed = ANIM_FILE_HEADER_BYTES + (size_t)count * FRAME_BYTES;
  auto freeOk = [&]() { return LittleFS.totalBytes() - LittleFS.usedBytes() >= needed; };
  LittleFS.remove(animSlotPath(slot));
  if (!freeOk())
    for (uint8_t s = 0; s < ANIM_MAX_SLOTS && !freeOk(); s++)
      if (s != slot && !(animActive && s == animPlayingSlot))
        LittleFS.remove(animSlotPath(s));
  if (!freeOk() && animActive)
    stopAnimation(false); // sacrifice the playing animation, keep its file for last
  if (!freeOk())
    LittleFS.remove(animSlotPath(animPlayingSlot));
  if (!freeOk())
    return;

  File f = LittleFS.open(ANIM_UPLOAD_PATH, FILE_WRITE);
  if (!f)
    return;

  uint8_t hdr[ANIM_FILE_HEADER_BYTES] = {
      (uint8_t)(count & 0xFF), (uint8_t)(count >> 8),
      (uint8_t)(delayMs & 0xFF), (uint8_t)(delayMs >> 8)};
  bool ok = f.write(hdr, ANIM_FILE_HEADER_BYTES) == ANIM_FILE_HEADER_BYTES;
  if (!ok)
  {
    f.close();
    LittleFS.remove(ANIM_UPLOAD_PATH);
    return;
  }
  animUploadFile = f; // kept open until the upload completes or aborts

  Serial.printf("Animation upload started: slot %u, %u frames, %u ms delay%s\n",
                slot, count, delayMs, stageOnly ? " (stage-only)" : "");

  animUploadId = uploadId;
  animStageOnly = stageOnly;
  animSlot = slot;
  animUploadCount = count;
  animExpectedIdx = 0;
  animUploading = true;
}

void handleAnimFrame(const byte *payload, unsigned int length)
{
  if (!animUploading || length != ANIM_FRAME_PAYLOAD)
    return;

  uint32_t magic;
  memcpy(&magic, payload, sizeof(magic));
  if (magic != ANIM_FRAME_MAGIC)
    return;

  uint16_t idx = payload[4] | (payload[5] << 8);
  if (idx != animExpectedIdx)
    return;

  // Ring only; the flash write happens in loop() (animUploadFlush). Flow control
  // (gated client.loop()) keeps a slot free, a full ring here means it failed.
  if (animRingCount >= ANIM_RING_FRAMES)
  {
    abortAnimUpload(); // next anim/start can retry cleanly
    return;
  }

  memcpy(animRing[animRingHead], payload + 6, FRAME_BYTES);
  animRingHead = (animRingHead + 1) % ANIM_RING_FRAMES;
  animRingCount++;
  animExpectedIdx++;
}

/** Drains the upload ring to flash. While an animation plays, only one page per loop
 *  pass so the write stalls stay small and the animation tick keeps its cadence; with
 *  the display idle it drains freely. Finalizes the upload once the last byte lands. */
void animUploadFlush()
{
  if (!animUploading)
    return;

  size_t budget = animActive ? ANIM_FLUSH_CHUNK : FRAME_BYTES;
  while (animRingCount > 0 && budget > 0)
  {
    size_t chunk = FRAME_BYTES - animRingFlushed;
    if (chunk > budget)
      chunk = budget;
    if (!animUploadFile || animUploadFile.write(animRing[animRingTail] + animRingFlushed, chunk) != chunk)
    {
      // Keep the playing animation running; the next anim/start can retry cleanly.
      abortAnimUpload();
      return;
    }
    animRingFlushed += chunk;
    budget -= chunk;
    if (animRingFlushed >= FRAME_BYTES)
    {
      animRingFlushed = 0;
      animRingTail = (animRingTail + 1) % ANIM_RING_FRAMES;
      animRingCount--;
    }
  }

  if (animRingCount == 0 && animExpectedIdx >= animUploadCount)
    completeAnimUpload();
}

/** Last frame drained: swap the staging file onto the slot file and ack/play. */
void completeAnimUpload()
{
  Serial.println("Animation upload complete");
  animUploading = false; // before stopAnimation, so staging is preserved
  animUploadFile.close();

  // Replacing the slot the animation is currently playing from: stop playback first,
  // LittleFS refuses remove/rename on an open file.
  if (animActive && animPlayingSlot == animSlot)
    stopAnimation(false);

  LittleFS.remove(animSlotPath(animSlot)); // replace the slot's previous content
  if (!LittleFS.rename(ANIM_UPLOAD_PATH, animSlotPath(animSlot)))
  {
    Serial.println("Animation rename failed");
    return;
  }

  // Ack the staging completion so the server knows the content landed.
  if (animStageOnly)
  {
    Serial.println("Animation staged");
    animAckSlot = animSlot;
    animAckUploadId = animUploadId;
    animLoadedAckPending = true;
    if (animPlayRequestId == animUploadId)
      playSlot(animSlot, animUploadId); // /anim/play raced ahead of the final frames
    return;
  }

  playSlot(animSlot, animUploadId);
}

void handleAnimPlay(const byte *payload, unsigned int length)
{
  if (length != ANIM_PLAY_PAYLOAD)
    return;

  uint32_t magic;
  memcpy(&magic, payload, sizeof(magic));
  if (magic != ANIM_PLAY_MAGIC)
    return;

  uint8_t slot = payload[4];
  uint32_t uploadId = (uint32_t)payload[5] | ((uint32_t)payload[6] << 8) |
                      ((uint32_t)payload[7] << 16) | ((uint32_t)payload[8] << 24);
  if (slot >= ANIM_MAX_SLOTS)
    return;

  if (animUploading && uploadId == animUploadId && slot == animSlot)
  {
    // Frames are still arriving; play as soon as the upload completes.
    animPlayRequestId = uploadId;
    return;
  }

  // Play whatever the slot holds: the server only skips the upload when the slot's
  // content is unchanged (persistent cache), so no id match is needed here.
  playSlot(slot, uploadId);
}

void callback(char *topic, byte *payload, unsigned int length)
{
  if (!updateScreen)
  {
    Serial.println("MQTT message dropped: updateScreen disabled");
    return;
  }

  const char *base = getClientId();
  size_t baseLen = strlen(base);
  if (strncmp(topic, base, baseLen) == 0)
  {
    const char *suffix = topic + baseLen;
    if (strcmp(suffix, TOPIC_ANIM_START) == 0)
    {
      handleAnimStart(payload, length);
      return;
    }
    if (strcmp(suffix, TOPIC_ANIM_FRAME) == 0)
    {
      handleAnimFrame(payload, length);
      return;
    }
    if (strcmp(suffix, TOPIC_ANIM_PLAY) == 0)
    {
      handleAnimPlay(payload, length);
      return;
    }
  }

  Serial.printf("Received %u bytes on topic %s\n", length, topic);

  if (length > MAX_PAYLOAD_SIZE)
  {
    Serial.println("Payload exceeds supported size");
    return;
  }

  if (length != FRAME_BYTES)
  {
    Serial.printf("Frame dropped: %u bytes, expected %u\n", length, FRAME_BYTES);
    return; // ignore malformed frames
  }

  // A static frame stops playback but keeps the slot files: they are a persistent
  // cache, and the server skips re-uploads while their content is unchanged.
  stopAnimation(false);
  memcpy(rxBuf, payload, FRAME_BYTES);

  dma_display->drawRGBBitmap(0, 0, px, W, H);
}

void init_broker_connection()
{
  Serial.println("Connected to Wifi, connecting to broker...");

  drawBitmap(0, 0, epd_bitmap_connecting, 64, 32, ScreenImage::Connecting, true);
  delay(2000);

  client.setBufferSize(16384);

  char server[preferences.getBytesLength("server")] = {};
  preferences.getBytes("server", server, preferences.getBytesLength("server"));

  const uint16_t mqtt_port = preferences.getUShort("server_port", 1883);

  Serial.print("\nServer: ");
  Serial.println(server);
  Serial.print("Port: ");
  Serial.println(mqtt_port);

  client.setServer(server, mqtt_port);
  client.setCallback(callback);

  if (!client.connected())
  {
    updateScreen = true;
    Serial.println(WiFi.localIP());
    reconnect();
  }
}

const char *getClientId()
{
  static char client_id[32]; // enough for "PXCORE75-" + MAC (12 chars) + '\0'

  if (client_id[0] == '\0')
  {                                 // build only once
    String mac = WiFi.macAddress(); // we still need this helper
    mac.replace(":", "");

    snprintf(client_id, sizeof(client_id), "PXCORE75-%s", mac.c_str());

    Serial.println();
    Serial.println();
    Serial.println("************************");
    Serial.println("* REGISTRATION DETAILS *");
    Serial.println("************************");
    Serial.print("-> Mac address: ");
    Serial.println(WiFi.macAddress());
    Serial.print("-> Client ID / Serial: ");
    Serial.println(client_id);
    Serial.print("-> MQTT topic (frame): ");
    Serial.println(client_id);
    Serial.print("-> MQTT topics (anim): ");
    Serial.printf("%s%s, %s%s\r\n", client_id, TOPIC_ANIM_START, client_id, TOPIC_ANIM_FRAME);
    Serial.println();
    Serial.println();
  }

  return client_id;
}

void writeFile(fs::FS &fs, const char *path, const uint16_t *intArray)
{
  Serial.printf("Writing file: %s\r\n", path);

  // if(!fs.exists(path)) {
  //  Define your uint16_t array
  size_t dataSize = 2048 * sizeof(uint16_t);

  Serial.print("Estimated size: ");
  Serial.println(dataSize);

  // Open file for binary writing
  File file = LittleFS.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }

  // Write the array as binary data
  size_t written = file.write((const uint8_t *)intArray, dataSize);
  file.close();

  // Check if all data was written
  if (written == dataSize)
  {
    Serial.print("Data written successfully: ");
    Serial.println(written);
  }
  else
  {
    Serial.printf("Only %u of %u bytes written.\n", written, dataSize);
  }
  // } else {
  //   Serial.println("File already on filesystem, skipping write!");
  // }
}

void readFile(fs::FS &fs, const char *path)
{
  Serial.printf("Reading file: %s\r\n", path);

  File file = LittleFS.open(path, FILE_READ);
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }

  const size_t numElements = 2048;
  uint16_t readBuffer[numElements];

  size_t bytesRead = file.read((uint8_t *)readBuffer, sizeof(readBuffer));
  file.close();

  if (bytesRead == sizeof(readBuffer))
  {
    Serial.println("Data read successfully:");
    drawBitmap(0, 0, readBuffer, 64, 32, ScreenImage::Client, true);
  }
  else
  {
    Serial.printf("Only %u of %u bytes read.\n", bytesRead, sizeof(readBuffer));
  }
}

void reconnect()
{
  long reconnectDelay = millis() - lastReconnectAttempt;
  if (reconnectDelay > 5000)
  {
    Serial.println();
    Serial.println("Took time: ");
    Serial.print(reconnectDelay);
    Serial.println();
    Serial.printf("The client %s connects to the public MQTT broker\n", getClientId());
    if (client.connect(getClientId()))
    {
      Serial.println("connected");
      drawBitmap(0, 0, epd_bitmap_connected, 64, 32, ScreenImage::Connected, true);
      drawBitmap(48, 23, epd_bitmap_check_mark, 8, 8, ScreenImage::Checkmark, false);
      Serial.printf("Subscribed to %s: %s\n", getClientId(), client.subscribe(getClientId()) ? "ok" : "FAILED");
      char animStartTopic[48];
      char animFrameTopic[48];
      char animPlayTopic[48];
      snprintf(animStartTopic, sizeof(animStartTopic), "%s%s", getClientId(), TOPIC_ANIM_START);
      snprintf(animFrameTopic, sizeof(animFrameTopic), "%s%s", getClientId(), TOPIC_ANIM_FRAME);
      snprintf(animPlayTopic, sizeof(animPlayTopic), "%s%s", getClientId(), TOPIC_ANIM_PLAY);
      Serial.printf("Subscribed to %s: %s\n", animStartTopic, client.subscribe(animStartTopic) ? "ok" : "FAILED");
      Serial.printf("Subscribed to %s: %s\n", animFrameTopic, client.subscribe(animFrameTopic) ? "ok" : "FAILED");
      Serial.printf("Subscribed to %s: %s\n", animPlayTopic, client.subscribe(animPlayTopic) ? "ok" : "FAILED");
      if (animUploading)
        abortAnimUpload(); // an in-flight upload died with the connection; completed staging survives
      startAnimation(animPlayingSlot); // slot files persist across reconnects
    }
    else
    {
      drawBitmap(0, 0, epd_bitmap_connecting, 64, 32, ScreenImage::Connecting, true);
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }

    lastReconnectAttempt = millis();
  }
}

void init_display()
{
  HUB75_I2S_CFG mxconfig(
      PANEL_RES_X, // module width
      PANEL_RES_Y, // module height
      PANEL_CHAIN, // Chain length
      _pins_x2);

  const uint8_t brightness = preferences.getUChar("brightness", 128);

  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->setBrightness(brightness); // 0-255
  dma_display->clearScreen();
}

void verifyRegistration()
{
  uint64_t device_id = ESP.getEfuseMac();
}

void setup()
{
  Serial.begin(115200);

  pinMode(17, INPUT_PULLUP);

  Serial.println("********************************************");
  Serial.print("* ");
  Serial.print(BUILD_NAME);
  Serial.print(" ");
  Serial.println(BUILD_VERSION);
  Serial.println("********************************************");

  if (!LittleFS.begin(FORMAT_LITTLE_FS_IF_FAILED))
  {
    Serial.println("LittleFS Mount Failed");
    return;
  }

  LittleFS.remove(ANIM_UPLOAD_PATH); // any staging file is stale after a reboot
  LittleFS.remove("/anim.bin");      // legacy pre-slot animation file, reclaim its space
  Serial.printf("LittleFS: %u / %u bytes used\n",
                (unsigned)LittleFS.usedBytes(), (unsigned)LittleFS.totalBytes());

  preferences.begin("cryptoticker", false);

  char ssid[preferences.getBytesLength("wifissid")] = {};
  char password[preferences.getBytesLength("wifipass")] = {};

  preferences.getBytes("wifissid", ssid, preferences.getBytesLength("wifissid"));
  preferences.getBytes("wifipass", password, preferences.getBytesLength("wifipass"));

  init_display();

  dma_display->setFont(&Org_01);
  drawBitmap(24, 1, epd_bitmap_pixelcore75_firmware_logo, 16, 16, ScreenImage::Logo, true);
  dma_display->setCursor(6, 22);
  dma_display->println(BUILD_NAME);
  dma_display->setTextColor(dma_display->color565(0, 0, 255));
  dma_display->setCursor(19, 30);
  dma_display->println(BUILD_VERSION);

  delay(5000);

  if (!check_bluetooth_button_pressed())
  {
    if (strlen(ssid) != 0 && strlen(password) != 0)
    {
      Serial.print("Trying to connect to WiFi: ");
      Serial.println(ssid);

      if (init_wifi(ssid, password))
      {
        verifyRegistration();
        init_broker_connection();
      }
    }
    else
    {
      init_bluetooth();
    }
  }
}

bool check_bluetooth_button_pressed()
{
  if (digitalRead(17) == LOW && !buttonPressed)
  {
    Serial.println("Bluetooth button pressed");
    updateScreen = false;
    init_bluetooth();
    return true;
  }

  return false;
}

void loop()
{
  buttonPressed = check_bluetooth_button_pressed();

  if (hasWifi && !buttonPressed && updateScreen)
  {
    if (!client.connected())
    {
      init_broker_connection();
    }
    animUploadFlush(); // make ring space before pulling more frames off the socket

    // Flow control: with the ring nearly full, hold off client.loop() (which delivers
    // at most one publish per call). TCP/MQTT backpressure then throttles the server
    // until the flusher catches up; PINGREQ/PUBACK also pause briefly, which is fine.
    if (!(animUploading && animRingCount >= ANIM_RING_FRAMES - 1))
      client.loop();

    if (animLoadedAckPending && client.connected())
    {
      char animLoadedTopic[48];
      snprintf(animLoadedTopic, sizeof(animLoadedTopic), "%s%s", getClientId(), TOPIC_ANIM_LOADED);
      uint8_t animLoadedPayload[ANIM_LOADED_PAYLOAD] = {
          'A', 'N', 'I', 'L',
          animAckSlot,
          (uint8_t)(animAckUploadId & 0xFF), (uint8_t)(animAckUploadId >> 8),
          (uint8_t)(animAckUploadId >> 16), (uint8_t)(animAckUploadId >> 24)};
      if (client.publish(animLoadedTopic, animLoadedPayload, sizeof(animLoadedPayload)))
      {
        animLoadedAckPending = false;
        Serial.println("Animation loaded ack published");
      }
    }

    animationTick();
  }
  else
  {
    delay(10);
  }
}
