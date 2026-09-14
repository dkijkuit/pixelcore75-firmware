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
#include <BLESecurity.h>
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

static constexpr size_t PREF_STR_MAX = 64; // NVS strings (SSID, password, MQTT host) + NUL
static constexpr uint32_t BLE_PASSKEY = 240719; // pairing passkey; only valid when read off the panel

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
bool brokerConfigured = false;      // MQTT client one-time setup (server/callback) done
bool mqttConnectingUiShown = false; // outage UI drawn once per disconnected period

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
static constexpr size_t ANIM_FILE_HEADER_BYTES = 4; // v1 slot file: frameCount(u16) + delayMs(u16)
static constexpr size_t ANIM_START_PAYLOAD = 14;    // magic + frameCount + delayMs + uploadId + flags + slot
static constexpr size_t ANIM_FRAME_PAYLOAD = 6 + FRAME_BYTES; // v1 ANIF: magic + frameIdx + pixels
static constexpr size_t ANIM_FRAME_V2_MIN = 103;    // v2 ANIF floor: header + palette + one run per row (32 rows)
static constexpr size_t ANIM_FRAME_V2_MAX = 4103;   // v2 ANIF ceiling: header + 4096B RAW body
static constexpr size_t ANIM_FRAME_V2_HEADER_BYTES = 7; // v2 ANIF: magic + frameIdx + frameFlags
static constexpr uint16_t SLOT_FILE_V2_MAGIC = 0x3241;  // "A2" (bytes 0x41 0x32); v1 would read it as
                                                        // frameCount 12865 > 200, so the formats can't collide
static constexpr size_t SLOT_FILE_V2_HEADER_BYTES = 6;  // v2 slot file: magic(u16) + frameCount(u16) + delayMs(u16)
static constexpr size_t PAL_RLE_PALETTE_BYTES = 32;     // 16 entries x RGB565 LE
static constexpr size_t ANIM_PLAY_PAYLOAD = 9;      // magic + slot + uploadId
static constexpr size_t ANIM_LOADED_PAYLOAD = 9;    // magic + slot + uploadId
static constexpr uint32_t SLOTIDX_MAGIC = 0x49544C53;       // "SLTI" little-endian
static constexpr const char *SLOTIDX_PATH = "/slotidx.bin"; // per-slot uploadId index: content-hash cache for /anim/play
static constexpr uint32_t ACMD_MAGIC = 0x444D4341; // "ACMD" little-endian (parametric command batch)
static constexpr uint8_t ACMD_VERSION = 1;         // wrong version drops the whole batch
static constexpr const char *TOPIC_CMD = "/cmd";   // server-rendered command channel (QoS 0, not retained)
static constexpr size_t ACMD_HEADER_BYTES = 7;     // magic(u32) + version(u8) + cmdCount(u16 LE)
static constexpr size_t ACMD_FONT_PAGES = 4;       // RAM font page slots (0..3), replaceable, not persisted
static constexpr uint16_t ACMD_GLYPH_ABSENT = 0xFFFF; // codeOff[] marker: glyph not in page
static constexpr unsigned long ACMD_TICK_MS = 10;  // parametric tick floor (~10-30 ms cadence from loop())
static constexpr uint16_t ACMD_SCROLL_HOLD_PX = 8;      // px-units HELD at each ping-pong extreme before reversing
static constexpr uint16_t ACMD_SCROLL_MIN_PASS_PX = 12; // min px-units per pass: a barely-overflowing text glides
enum AcmdOpcode : uint8_t
{
  ACMD_NOP = 0x00,
  ACMD_CLS = 0x01,
  ACMD_PIX = 0x02,
  ACMD_LINE = 0x03,
  ACMD_RECT = 0x04,
  ACMD_FILL = 0x05,
  ACMD_CIRC = 0x06,
  ACMD_BLIT = 0x07,
  ACMD_FONT = 0x10,
  ACMD_TEXT = 0x11,
  ACMD_SWEEP = 0x20,
  ACMD_SCROLL = 0x21,
  ACMD_BLINK = 0x22
};
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static constexpr uint8_t SLOTIDX_VERSION = 1;
static constexpr size_t SLOTIDX_FILE_BYTES = 5 + ANIM_MAX_SLOTS * 4; // magic + version + one u32 uploadId per slot
static constexpr uint8_t ANIM_FLAG_STAGE_ONLY = 0x01; // stage to flash but wait for /anim/play
static constexpr uint8_t ANIM_FLAG_CODEC_MASK = 0x06; // bits 1-2 = upload codec
static constexpr uint8_t ANIM_FLAG_CODEC_PAL_RLE = 0x02; // codec 1: per-frame RAW/PAL_RLE, v2 slot file
static constexpr uint16_t MAX_ANIM_FRAMES = 200;    // sanity cap; ~800KB fits the ~4.9MB LittleFS many times over
static constexpr uint16_t ANIM_MIN_DELAY_MS = 10;
static constexpr size_t ANIM_RING_FRAMES = 8;  // RAM frame ring (8 slots): decouples MQTT from flash writes
static constexpr size_t ANIM_RING_SLOT_BYTES = 4104; // worst-case buffered blob: v2 frameFlags + 4096B body
static constexpr size_t ANIM_FLUSH_CHUNK = 256; // one flash page per loop pass while an animation plays

uint8_t rxBuf[FRAME_BYTES];       // raw bytes from MQTT
uint16_t *px = (uint16_t *)rxBuf; // view as RGB565 pixels (little-endian)

bool animActive = false;
bool animUploading = false;
bool animUploadV2 = false;   // current upload uses codec 1 (per-frame blobs, v2 slot file)
bool animPlayingV2 = false;  // playing slot is v2 format (offset table + blobs); v1 = fixed-size frames
bool animLoadedAckPending = false;
uint32_t animUploadId = 0;
bool animStageOnly = false;     // current upload stages but doesn't play on completion
uint8_t animSlot = 0;           // slot the current upload writes to
uint8_t animPlayingSlot = 0;    // slot currently playing (auto-resume after reconnect/reboot)
uint8_t animAckSlot = 0;        // ack payload while animLoadedAckPending
uint32_t animAckUploadId = 0;   // ack payload while animLoadedAckPending
uint32_t animPlayRequestId = 0; // /anim/play that arrived while the upload was still running
uint32_t animSlotUploadIds[ANIM_MAX_SLOTS] = {0}; // RAM mirror of /slotidx.bin; 0 = slot content unknown
uint32_t animUploadBlobBytes = 0;               // v2 upload: blob bytes accepted so far (ring + flushed)
uint32_t animFrameOffsets[MAX_ANIM_FRAMES];     // v2 upload: absolute blob offsets in the staging file
uint32_t animPlayOffsets[MAX_ANIM_FRAMES];      // v2 playback: absolute blob offsets in the slot file
uint32_t animPlayFileSize = 0;                  // v2 playback: bounds blob reads (last blob runs to EOF)
uint8_t animV2Scratch[ANIM_FRAME_V2_MAX - ANIM_FRAME_V2_HEADER_BYTES - PAL_RLE_PALETTE_BYTES]; // PAL_RLE pairs (4064 B)
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
// Slots hold one frame blob each: FRAME_BYTES for v1 uploads, frameFlags+body (1..4097 B)
// for v2 — animRingLen tracks the valid length of each slot.
uint8_t animRing[ANIM_RING_FRAMES][ANIM_RING_SLOT_BYTES];
size_t animRingLen[ANIM_RING_FRAMES]; // valid bytes per slot (set when the frame is queued)
size_t animRingHead = 0;       // next free frame (writer: MQTT callback)
size_t animRingTail = 0;       // oldest buffered frame (reader: loop flusher)
size_t animRingCount = 0;      // frames buffered
size_t animRingFlushed = 0;    // bytes of the tail frame already written to flash

// anim/start is staged here by the MQTT callback and handled from loop(): starting an
// upload does LittleFS removes/creates (flash erases) that would stall playback inline.
bool animStartPending = false;
uint8_t animStartPayload[ANIM_START_PAYLOAD];

// ACMD parametric command engine: batches render into the work canvas and commit
// atomically (base canvas swap + display push + parametric arming). The base canvas is
// immutable after commit, so parametric overlays (SWEEP/SCROLL/BLINK) composite over a
// fresh copy of it every tick — that copy also IS the "region snapshot restore" of the
// SCROLL/BLINK semantics (their snapshots equal the base region at commit time).
uint16_t acmdBase[W * H]; // committed frame
uint16_t acmdWork[W * H]; // batch render scratch + parametric composite buffer
struct AcmdFontPage
{
  bool present;           // at least one FONT command has loaded this page (RAM only, never persisted)
  uint8_t *glyphs;        // packed records: code,w,h,xAdvance,xOff,yOff + h*ceil(w/8) bitmap bytes
  size_t glyphsLen;
  uint16_t codeOff[256];  // record offset per char code; ACMD_GLYPH_ABSENT = not in page
};
AcmdFontPage acmdFonts[ACMD_FONT_PAGES]; // static storage: zero-initialized (present = false)
enum AcmdParamType { ACMD_PARAM_NONE = 0, ACMD_PARAM_SWEEP, ACMD_PARAM_SCROLL, ACMD_PARAM_BLINK };
// A batch arms ALL its parametric primitives, in command order, up to ACMD_PARAMS_MAX
// (4 — the radar's sweep + up to three scrolling info-column lines); further ones are
// validated but ignored. Every tick composites the overlays over a fresh base copy in
// that same order, so a later overlay draws over an earlier one where they overlap.
static constexpr size_t ACMD_PARAMS_MAX = 4;
struct AcmdSweepState
{
  uint8_t cx, cy, r, speed; // speedDegPerSec 1..255
  uint16_t color;
};
struct AcmdScrollState
{
  uint8_t x, y, w, h, fontId;
  uint16_t color, speedMs;
  uint8_t len;             // ASCII chars, 1..255
  char text[256];          // text + NUL
  int32_t textW;           // sum of glyph advances (+4 per unknown glyph)
};
struct AcmdBlinkState
{
  uint8_t x, y, w, h;
  uint16_t periodMs;       // >= 1
};
struct AcmdParam
{
  AcmdParamType type;
  union
  {
    AcmdSweepState sw;
    AcmdScrollState sc;
    AcmdBlinkState bl;
  } u;
};
bool acmdActive = false;
size_t acmdLiveCount = 0;          // armed parametrics (0 = static batch)
AcmdParam acmdLive[ACMD_PARAMS_MAX];
unsigned long acmdCommitMs = 0;   // parametric state derives from elapsed ms since commit
unsigned long acmdLastDrawMs = 0; // tick throttle (cadence jitter must not affect the render)

bool init_wifi(char ssid[], char password[]);
void onConnect(BLEServer *pServer);
void onDisconnect(BLEServer *pServer);
void init_bluetooth();
void setAsciiValue(BLECharacteristic *ch, const String &val);
void callback(char *topic, byte *payload, unsigned int length);
void init_broker_connection();
void reconnect();
void init_display();
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
void handleAcmd(const byte *payload, unsigned int length);
void acmdTick();

/** Read a NUL-terminated byte blob from NVS into a fixed buffer. Replaces the previous
 *  `char buf[preferences.getBytesLength(key)]` VLA pattern: on a factory-fresh panel the
 *  keys are absent (getBytesLength() == 0) and a zero-sized VLA is undefined behavior.
 *  Returns the string length (0 if unset). */
static size_t readPrefString(const char *key, char *out, size_t outSize)
{
  if (outSize == 0)
    return 0;
  out[0] = '\0';
  size_t len = preferences.getBytesLength(key);
  if (len == 0)
    return 0;
  if (len > outSize)
    len = outSize; // truncate over-long stored values
  preferences.getBytes(key, out, len);
  out[len - 1] = '\0'; // blobs are stored NUL-terminated; keep a terminator after truncation
  return strlen(out);
}

/** Show the 6-digit BLE pairing passkey on the panel so only someone physically at it
 *  can pair. Called from the BLE task (same convention as the connection callbacks). */
static void drawPasskey(uint32_t passkey)
{
  dma_display->clearScreen();
  currentScreenImage = ScreenImage::Client;
  dma_display->setTextColor(dma_display->color565(255, 255, 0));
  dma_display->setCursor(2, 22);
  dma_display->printf("%06u", (unsigned)passkey);
}

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

    // This callback runs in the BLE stack task: never delay()/ESP.restart() here (the
    // old reboot killed any playing animation on every disconnect). Just advertise
    // again so the app can reconnect.
    BLEDevice::startAdvertising();
  }
};

/** Passkey pairing for BLE provisioning: without it anyone in radio range could rewrite
 *  the WiFi/MQTT/brightness config. IO_CAP_OUT (display-only) + MITM makes the peer
 *  type the passkey this panel shows on the matrix, so pairing needs physical presence.
 *  OS-level pairing — the companion app needs no code or UUID changes. */
class BLESecurityCallbacksImpl : public BLESecurityCallbacks
{
  uint32_t onPassKeyRequest() override
  {
    Serial.printf("BLE pairing passkey: %u\n", (unsigned)BLE_PASSKEY);
    drawPasskey(BLE_PASSKEY);
    return BLE_PASSKEY;
  }
  void onPassKeyNotify(uint32_t passkey) override {}
  bool onSecurityRequest() override { return true; }
  bool onConfirmPIN(uint32_t passkey) override { return false; } // not used with IO_CAP_OUT
  void onAuthenticationComplete(esp_ble_auth_cmpl_t auth_cmpl) override
  {
    Serial.printf("BLE authentication %s\n",
                  auth_cmpl.success ? "succeeded" : "failed");
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
    BLEDevice::setSecurityCallbacks(new BLESecurityCallbacksImpl());
    BLESecurity security; // setters apply the GAP params immediately (stack is up here)
    security.setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM);
    security.setCapability(ESP_IO_CAP_OUT);
    security.setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    serverCharacteristic->setCallbacks(new ServerCharacteristicCallBack());
    serverPortCharacteristic->setCallbacks(new ServerPortCharacteristicCallBack());
    ssidCharacteristic->setCallbacks(new SsidCharacteristicCallBack());
    passwordCharacteristic->setCallbacks(new PasswordCharacteristicCallBack());
    brightnessCharacteristic->setCallbacks(new BrightnessCharacteristicCallBack());
    restartCharacteristic->setCallbacks(new RestartCharacteristicCallBack());

    char ssid[PREF_STR_MAX];
    readPrefString("wifissid", ssid, sizeof(ssid));
    ssidCharacteristic->setValue(ssid);

    char server[PREF_STR_MAX];
    readPrefString("server", server, sizeof(server));
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
  animUploadV2 = false;
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

/** Load the slot upload-id index. Anything missing or invalid (bad magic/version/size)
 *  leaves all zeros, which merely disables the ANIP hash-skip until the next completed
 *  upload rewrites the file — a reflash or corrupt index costs one redundant upload. */
static void loadSlotUploadIds()
{
  memset(animSlotUploadIds, 0, sizeof(animSlotUploadIds));
  if (!LittleFS.exists(SLOTIDX_PATH))
    return;
  File f = LittleFS.open(SLOTIDX_PATH, FILE_READ);
  if (!f)
    return;
  uint8_t ids[SLOTIDX_FILE_BYTES];
  bool valid = f.size() == SLOTIDX_FILE_BYTES &&
               f.seek(0) &&
               f.read(ids, sizeof(ids)) == sizeof(ids);
  f.close();
  uint32_t magic = 0;
  if (valid)
    memcpy(&magic, ids, sizeof(magic));
  if (!valid || magic != SLOTIDX_MAGIC || ids[4] != SLOTIDX_VERSION)
    return;
  for (uint8_t s = 0; s < ANIM_MAX_SLOTS; s++)
  {
    size_t o = 5 + (size_t)s * 4;
    animSlotUploadIds[s] = (uint32_t)ids[o] | ((uint32_t)ids[o + 1] << 8) |
                           ((uint32_t)ids[o + 2] << 16) | ((uint32_t)ids[o + 3] << 24);
  }
}

/** Persist the slot upload-id index. Full-file rewrite (133 bytes) — entries only change
 *  on upload completion, ladder eviction or corrupt-file deletion, all rare events. */
static void saveSlotUploadIds()
{
  uint8_t ids[SLOTIDX_FILE_BYTES] = {'S', 'L', 'T', 'I', SLOTIDX_VERSION};
  for (uint8_t s = 0; s < ANIM_MAX_SLOTS; s++)
  {
    uint32_t id = animSlotUploadIds[s];
    size_t o = 5 + (size_t)s * 4;
    ids[o] = (uint8_t)(id & 0xFF);
    ids[o + 1] = (uint8_t)(id >> 8);
    ids[o + 2] = (uint8_t)(id >> 16);
    ids[o + 3] = (uint8_t)(id >> 24);
  }
  File f = LittleFS.open(SLOTIDX_PATH, FILE_WRITE);
  if (!f)
    return;
  if (f.write(ids, sizeof(ids)) != sizeof(ids))
  {
    f.close();
    LittleFS.remove(SLOTIDX_PATH); // truncated index: load treats it as all zeros anyway
    return;
  }
  f.close();
}

static void setSlotUploadId(uint8_t slot, uint32_t uploadId)
{
  if (slot >= ANIM_MAX_SLOTS || animSlotUploadIds[slot] == uploadId)
    return;
  animSlotUploadIds[slot] = uploadId;
  saveSlotUploadIds();
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
  acmdActive = false; // static frames, animation starts and reconnect auto-resume supersede ACMD
  animPlayingV2 = false;
  animUploading = false;
  animLoadedAckPending = false;
  if (animFile)
    animFile.close();
  if (removePlayingSlot)
  {
    LittleFS.remove(animSlotPath(animPlayingSlot));
    setSlotUploadId(animPlayingSlot, 0); // index must not claim content for a deleted file,
                                         // or the ANIP hash fast path plays a missing slot
  }
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

  if (animPlayingV2)
  {
    // v2 slot file: seek this frame's blob, dispatch on its per-frame flag. Ingest already
    // validated the structure; any read/decode failure here means flash corruption → the
    // v1 remedy (stop + delete) applies to both formats.
    uint32_t blob = animPlayOffsets[animFrameIdx];
    if (!animFile.seek(blob))
    {
      stopAnimation(true);
      return;
    }
    uint8_t frameFlags = 0;
    if (animFile.read(&frameFlags, 1) != 1)
    {
      stopAnimation(true);
      return;
    }
    if (frameFlags == 0)
    {
      if (animFile.read(animBuf, FRAME_BYTES) != FRAME_BYTES)
      {
        stopAnimation(true);
        return;
      }
    }
    else if (frameFlags == 1)
    {
      uint8_t pal[PAL_RLE_PALETTE_BYTES];
      if (animFile.read(pal, sizeof(pal)) != sizeof(pal))
      {
        stopAnimation(true);
        return;
      }
      uint16_t pal16[16];
      for (uint8_t c = 0; c < 16; c++)
        pal16[c] = (uint16_t)pal[2 * c] | ((uint16_t)pal[2 * c + 1] << 8);
      // The last blob runs to EOF; cap the pair read at the scratch size (a valid frame
      // never needs more, since ingest enforced the 4103-byte ANIF ceiling).
      uint32_t remain = animPlayFileSize - blob - 1 - sizeof(pal);
      size_t rd = remain < sizeof(animV2Scratch) ? remain : sizeof(animV2Scratch);
      if (animFile.read(animV2Scratch, rd) != rd)
      {
        stopAnimation(true);
        return;
      }
      uint16_t *out = (uint16_t *)animBuf;
      size_t done = 0;
      for (size_t p = 0; p < rd / 2; p++)
      {
        uint8_t run = animV2Scratch[2 * p];
        uint8_t ci = animV2Scratch[2 * p + 1];
        if (run < 1 || ci > 15 || done + run > W * H)
        {
          stopAnimation(true); // corrupt pair: never index past the palette or buffer
          return;
        }
        for (uint8_t r = 0; r < run; r++)
          out[done++] = pal16[ci];
      }
      if (done != W * H)
      {
        stopAnimation(true);
        return;
      }
    }
    else
    {
      stopAnimation(true); // unknown per-frame flag: corrupt blob
      return;
    }
  }
  else
  {
    size_t offset = ANIM_FILE_HEADER_BYTES + (size_t)animFrameIdx * FRAME_BYTES;
    if (!animFile.seek(offset) || animFile.read(animBuf, FRAME_BYTES) != FRAME_BYTES)
    {
      stopAnimation(true);
      return;
    }
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

  // Format sniff on the first two bytes: "A2" = v2 (offset table + per-frame blobs),
  // anything else = v1 (fixed-size frames). Unambiguous: read as a v1 frameCount, the
  // magic is 12865 > MAX_ANIM_FRAMES, so no v1 file can start with it.
  uint8_t hdr[SLOT_FILE_V2_HEADER_BYTES];
  bool valid = f.size() >= 2 && f.seek(0) && f.read(hdr, 2) == 2;
  bool v2 = valid && hdr[0] == (uint8_t)(SLOT_FILE_V2_MAGIC & 0xFF) &&
            hdr[1] == (uint8_t)(SLOT_FILE_V2_MAGIC >> 8);
  if (valid)
  {
    if (v2)
    {
      valid = f.size() >= SLOT_FILE_V2_HEADER_BYTES && f.seek(0) &&
              f.read(hdr, SLOT_FILE_V2_HEADER_BYTES) == SLOT_FILE_V2_HEADER_BYTES;
      if (valid)
      {
        animFrameCount = hdr[2] | (hdr[3] << 8);
        animDelayMs = hdr[4] | (hdr[5] << 8);
        size_t tableEnd = SLOT_FILE_V2_HEADER_BYTES + (size_t)animFrameCount * 4;
        // Offset table must be readable, start at/after its own end, be monotonic
        // non-decreasing and point every blob inside the file (so the last blob ends
        // at/before EOF — it runs to EOF during playback).
        valid = animFrameCount >= 2 && animFrameCount <= MAX_ANIM_FRAMES &&
                f.size() > tableEnd && f.seek(SLOT_FILE_V2_HEADER_BYTES);
        uint32_t prev = (uint32_t)tableEnd;
        for (uint16_t i = 0; valid && i < animFrameCount; i++)
        {
          uint8_t e[4];
          valid = f.read(e, 4) == 4;
          uint32_t off = 0;
          if (valid)
            off = (uint32_t)e[0] | ((uint32_t)e[1] << 8) | ((uint32_t)e[2] << 16) | ((uint32_t)e[3] << 24);
          valid = valid && off >= prev && off < f.size();
          prev = off;
          if (valid)
            animPlayOffsets[i] = off;
        }
        animPlayFileSize = f.size();
      }
    }
    else
    {
      valid = f.size() >= ANIM_FILE_HEADER_BYTES + FRAME_BYTES && f.seek(0) &&
              f.read(hdr, ANIM_FILE_HEADER_BYTES) == ANIM_FILE_HEADER_BYTES;
      if (valid)
      {
        animFrameCount = hdr[0] | (hdr[1] << 8);
        animDelayMs = hdr[2] | (hdr[3] << 8);
        valid = animFrameCount >= 2 && animFrameCount <= MAX_ANIM_FRAMES &&
                f.size() == ANIM_FILE_HEADER_BYTES + (size_t)animFrameCount * FRAME_BYTES;
      }
    }
  }
  if (!valid)
  {
    f.close();
    LittleFS.remove(path);
    setSlotUploadId(slot, 0); // index must not claim content for a deleted file
    return;
  }

  if (animDelayMs < ANIM_MIN_DELAY_MS)
    animDelayMs = ANIM_MIN_DELAY_MS;

  animPlayingV2 = v2;
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
  uint8_t codecBits = payload[12] & ANIM_FLAG_CODEC_MASK;
  if (codecBits != 0 && codecBits != ANIM_FLAG_CODEC_PAL_RLE)
    return; // unknown codec (2-3): silent drop, the server times out and downgrades to RAW
  bool v2 = codecBits == ANIM_FLAG_CODEC_PAL_RLE;
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
  // slot files, and only as a last resort the playing animation. Every dropped file also
  // invalidates its slotidx entry so the index never outlives the content it describes.
  // The estimate is the v1 worst case (4 + count*4096); v2 uploads land smaller, so it
  // stays a safe upper bound.
  size_t needed = ANIM_FILE_HEADER_BYTES + (size_t)count * FRAME_BYTES;
  auto freeOk = [&]() { return LittleFS.totalBytes() - LittleFS.usedBytes() >= needed; };
  bool idxDirty = false;
  auto dropSlot = [&](uint8_t s) {
    LittleFS.remove(animSlotPath(s));
    if (animSlotUploadIds[s] != 0)
    {
      animSlotUploadIds[s] = 0;
      idxDirty = true;
    }
  };
  dropSlot(slot);
  if (!freeOk())
    for (uint8_t s = 0; s < ANIM_MAX_SLOTS && !freeOk(); s++)
      if (s != slot && !(animActive && s == animPlayingSlot))
        dropSlot(s);
  if (!freeOk() && animActive)
    stopAnimation(false); // sacrifice the playing animation, keep its file for last
  if (!freeOk())
    dropSlot(animPlayingSlot);
  if (idxDirty)
    saveSlotUploadIds();
  if (!freeOk())
    return;

  File f = LittleFS.open(ANIM_UPLOAD_PATH, FILE_WRITE);
  if (!f)
    return;

  // v1 staging: 4-byte header + raw frames. v2 staging: 6-byte header + a zeroed
  // placeholder offset table (patched with the real blob offsets on completion).
  uint8_t hdr[SLOT_FILE_V2_HEADER_BYTES];
  size_t hdrLen;
  if (v2)
  {
    hdr[0] = (uint8_t)(SLOT_FILE_V2_MAGIC & 0xFF);
    hdr[1] = (uint8_t)(SLOT_FILE_V2_MAGIC >> 8);
    hdr[2] = (uint8_t)(count & 0xFF);
    hdr[3] = (uint8_t)(count >> 8);
    hdr[4] = (uint8_t)(delayMs & 0xFF);
    hdr[5] = (uint8_t)(delayMs >> 8);
    hdrLen = SLOT_FILE_V2_HEADER_BYTES;
  }
  else
  {
    hdr[0] = (uint8_t)(count & 0xFF);
    hdr[1] = (uint8_t)(count >> 8);
    hdr[2] = (uint8_t)(delayMs & 0xFF);
    hdr[3] = (uint8_t)(delayMs >> 8);
    hdrLen = ANIM_FILE_HEADER_BYTES;
  }
  bool ok = f.write(hdr, hdrLen) == hdrLen;
  if (ok && v2)
  {
    static const uint8_t zeros[64] = {0};
    size_t tableBytes = (size_t)count * 4;
    while (ok && tableBytes > 0)
    {
      size_t chunk = tableBytes < sizeof(zeros) ? tableBytes : sizeof(zeros);
      ok = f.write(zeros, chunk) == chunk;
      tableBytes -= chunk;
    }
  }
  if (!ok)
  {
    f.close();
    LittleFS.remove(ANIM_UPLOAD_PATH);
    return;
  }
  animUploadFile = f; // kept open until the upload completes or aborts

  Serial.printf("Animation upload started: slot %u, %u frames, %u ms delay%s%s\n",
                slot, count, delayMs, v2 ? " (PAL_RLE)" : "", stageOnly ? " (stage-only)" : "");

  animUploadId = uploadId;
  animStageOnly = stageOnly;
  animSlot = slot;
  animUploadCount = count;
  animExpectedIdx = 0;
  animUploadV2 = v2;
  animUploadBlobBytes = 0;
  animUploading = true;
}

/** Structural ingest check for a PAL_RLE ANIF body: 32-byte palette + (run, colorIdx)
 *  pairs. Runs never cross row boundaries: 64 px per row, each row's runs sum to exactly
 *  64, run is 1..64, colorIdx is 0..15, and the pairs cover all 2048 pixels. Violations
 *  abort the upload so garbage never reaches flash. */
static bool palRleBodyValid(const uint8_t *body, size_t len)
{
  if (len < PAL_RLE_PALETTE_BYTES || (len - PAL_RLE_PALETTE_BYTES) % 2 != 0)
    return false;
  const uint8_t *pairs = body + PAL_RLE_PALETTE_BYTES;
  size_t pairCount = (len - PAL_RLE_PALETTE_BYTES) / 2;
  size_t total = 0, row = 0;
  for (size_t i = 0; i < pairCount; i++)
  {
    uint8_t run = pairs[2 * i];
    if (run < 1 || run > W || pairs[2 * i + 1] > 15)
      return false;
    row += run;
    total += run;
    if (row > W)
      return false; // this run crossed (or overshot) the row boundary
    if (row == W)
      row = 0;
  }
  return row == 0 && total == W * H;
}

void handleAnimFrame(const byte *payload, unsigned int length)
{
  if (!animUploading)
    return;
  if (animUploadV2)
  {
    if (length < ANIM_FRAME_V2_MIN || length > ANIM_FRAME_V2_MAX)
    {
      abortAnimUpload(); // v2 frames are structurally checked at ingest
      return;
    }
  }
  else if (length != ANIM_FRAME_PAYLOAD)
    return;

  uint32_t magic;
  memcpy(&magic, payload, sizeof(magic));
  if (magic != ANIM_FRAME_MAGIC)
    return;

  uint16_t idx = payload[4] | (payload[5] << 8);
  if (idx != animExpectedIdx)
    return;

  size_t blobLen = length - 6; // frameFlags + body for v2, raw pixels for v1
  if (animUploadV2)
  {
    if (idx >= animUploadCount)
      return; // over-sent frame: the ANIM frameCount is authoritative
    uint8_t frameFlags = payload[6];
    if (frameFlags > 1 || (frameFlags == 1 && !palRleBodyValid(payload + 7, length - 7)))
    {
      abortAnimUpload();
      return;
    }
  }

  // Ring only; the flash write happens in loop() (animUploadFlush). Flow control
  // (gated client.loop()) keeps a slot free, a full ring here means it failed.
  if (animRingCount >= ANIM_RING_FRAMES)
  {
    abortAnimUpload(); // next anim/start can retry cleanly
    return;
  }

  memcpy(animRing[animRingHead], payload + 6, blobLen);
  animRingLen[animRingHead] = blobLen;
  if (animUploadV2)
  {
    // Blob offsets are deterministic in acceptance order, independent of flush timing:
    // header + placeholder table + every previously accepted blob.
    animFrameOffsets[idx] = SLOT_FILE_V2_HEADER_BYTES + (uint32_t)animUploadCount * 4 + animUploadBlobBytes;
    animUploadBlobBytes += blobLen;
  }
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

  size_t budget = animActive ? ANIM_FLUSH_CHUNK : ANIM_RING_SLOT_BYTES;
  while (animRingCount > 0 && budget > 0)
  {
    size_t slotLen = animRingLen[animRingTail];
    size_t chunk = slotLen - animRingFlushed;
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
    if (animRingFlushed >= slotLen)
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
  if (animUploadV2)
  {
    animUploadV2 = false;
    // Patch the placeholder offset table now that every blob has landed, while the
    // staging handle is still open. A failed patch leaves an unplayable file — drop
    // the whole upload instead; the server re-sends on its ack timeout.
    bool ok = animUploadFile.seek(SLOT_FILE_V2_HEADER_BYTES);
    for (uint16_t i = 0; ok && i < animUploadCount; i++)
    {
      uint32_t off = animFrameOffsets[i];
      uint8_t e[4] = {(uint8_t)(off & 0xFF), (uint8_t)(off >> 8),
                      (uint8_t)(off >> 16), (uint8_t)(off >> 24)};
      ok = animUploadFile.write(e, sizeof(e)) == sizeof(e);
    }
    if (!ok)
    {
      animUploadFile.close();
      LittleFS.remove(ANIM_UPLOAD_PATH);
      return;
    }
  }
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
  setSlotUploadId(animSlot, animUploadId); // slot content now matches this upload's hash

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

  // Content-hash gate: a non-zero uploadId means the server skipped the upload because
  // it believes this slot already holds that content. Play only on an exact persisted-id
  // match; on mismatch stay silent (no play, no ANIL) so the server's play-ack timeout
  // falls back to an inline upload. A missing/corrupt slot file is also silent:
  // startAnimation fails and never arms the ANIL. uploadId 0 keeps legacy behavior.
  if (uploadId != 0 && animSlotUploadIds[slot] != uploadId)
    return;

  playSlot(slot, uploadId);
}

// **************************************
// ACMD v1 — parametric command engine
//
// Server-rendered command batches on <base>/cmd, executed on a RAM canvas that mirrors
// Adafruit_GFX pixel-for-pixel (the server's Java preview renderer transcribes the same
// algorithms from the same source): writeLine Bresenham with steep swap + err = dx/2,
// fillRect as per-column vlines, drawCircle's 4 initial + 8 symmetric writes, int16
// arithmetic, out-of-bounds pixels silently dropped. A whole batch renders into the
// work canvas and commits atomically; any violation drops the batch and keeps the
// prior display. The FIRST parametric primitive (SWEEP/SCROLL/BLINK) in a batch wins;
// later ones are validated but ignored.
// **************************************

static inline uint16_t acmdU16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline void acmdSwapI16(int16_t &a, int16_t &b)
{
  int16_t t = a;
  a = b;
  b = t;
}

/** Bounds-checked canvas write — the mirror of GFX drawPixel (drop, never clip-run). */
static void acmdPixel(uint16_t *cv, int32_t x, int32_t y, uint16_t color)
{
  if (x >= 0 && x < W && y >= 0 && y < H)
    cv[y * W + x] = color;
}

/** Adafruit_GFX::writeLine transcribed: steep swap, x0<=x1 ordering, err = dx/2. */
static void acmdWriteLine(uint16_t *cv, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
  int16_t adx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
  int16_t ady = (y1 > y0) ? (y1 - y0) : (y0 - y1);
  bool steep = ady > adx;
  if (steep)
  {
    acmdSwapI16(x0, y0);
    acmdSwapI16(x1, y1);
  }
  if (x0 > x1)
  {
    acmdSwapI16(x0, x1);
    acmdSwapI16(y0, y1);
  }
  int16_t dx = x1 - x0;
  int16_t dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
  int16_t err = dx / 2;
  int16_t ystep = (y0 < y1) ? 1 : -1;
  for (; x0 <= x1; x0++)
  {
    if (steep)
      acmdPixel(cv, y0, x0, color);
    else
      acmdPixel(cv, x0, y0, color);
    err -= dy;
    if (err < 0)
    {
      y0 += ystep;
      err += dx;
    }
  }
}

/** Adafruit_GFX::drawRect: four fast lines, each a writeLine. */
static void acmdDrawRect(uint16_t *cv, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  acmdWriteLine(cv, x, y, x + w - 1, y, color);
  acmdWriteLine(cv, x, y + h - 1, x + w - 1, y + h - 1, color);
  acmdWriteLine(cv, x, y, x, y + h - 1, color);
  acmdWriteLine(cv, x + w - 1, y, x + w - 1, y + h - 1, color);
}

/** Adafruit_GFX::fillRect: one vline per column. Degenerate sizes keep the GFX quirk:
 *  h = 0 paints rows y-1 and y (swapped writeLine), w = 0 still iterates once per GFX. */
static void acmdFillRect(uint16_t *cv, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  for (int16_t i = x; i < x + w; i++)
    acmdWriteLine(cv, i, y, i, y + h - 1, color);
}

/** Adafruit_GFX::drawCircle transcribed: 4 initial writes + 8 symmetric per step. */
static void acmdDrawCircle(uint16_t *cv, int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  acmdPixel(cv, x0, y0 + r, color);
  acmdPixel(cv, x0, y0 - r, color);
  acmdPixel(cv, x0 + r, y0, color);
  acmdPixel(cv, x0 - r, y0, color);

  while (x < y)
  {
    if (f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    acmdPixel(cv, x0 + x, y0 + y, color);
    acmdPixel(cv, x0 - x, y0 + y, color);
    acmdPixel(cv, x0 + x, y0 - y, color);
    acmdPixel(cv, x0 - x, y0 - y, color);
    acmdPixel(cv, x0 + y, y0 + x, color);
    acmdPixel(cv, x0 - y, y0 + x, color);
    acmdPixel(cv, x0 + y, y0 - x, color);
    acmdPixel(cv, x0 - y, y0 - x, color);
  }
}

/** Draw one FONT-page glyph record (code,w,h,xAdvance,xOff,yOff + MSB-first bitmap):
 *  pixels at (penX + xOff + gx, topY + yOff + gy); optional rect clip (SCROLL region). */
static void acmdDrawGlyph(uint16_t *cv, const uint8_t *rec, int32_t penX, int32_t topY,
                          uint16_t color, bool clip, int32_t cx, int32_t cy, int32_t cw, int32_t ch)
{
  int32_t w = rec[1];
  int32_t h = rec[2];
  int32_t ox = penX + (int8_t)rec[4];
  int32_t oy = topY + (int8_t)rec[5];
  const uint8_t *bmp = rec + 6;
  for (int32_t gy = 0; gy < h; gy++)
  {
    for (int32_t gx = 0; gx < w; gx++)
    {
      if (!((bmp[gy * ((w + 7) / 8) + (gx >> 3)] >> (7 - (gx & 7))) & 1))
        continue;
      int32_t X = ox + gx;
      int32_t Y = oy + gy;
      if (clip && (X < cx || X >= cx + cw || Y < cy || Y >= cy + ch))
        continue;
      acmdPixel(cv, X, Y, color);
    }
  }
}

/** Whole-sequence parametric equality (epoch carry-over test): same count, order,
 * types and every parameter — SCROLL text bytes included. */
static bool acmdParamsEqual(const AcmdParam &a, const AcmdParam &b)
{
  if (a.type != b.type)
    return false;
  switch (a.type)
  {
  case ACMD_PARAM_SWEEP:
    return a.u.sw.cx == b.u.sw.cx && a.u.sw.cy == b.u.sw.cy && a.u.sw.r == b.u.sw.r &&
           a.u.sw.color == b.u.sw.color && a.u.sw.speed == b.u.sw.speed;
  case ACMD_PARAM_SCROLL:
    return a.u.sc.x == b.u.sc.x && a.u.sc.y == b.u.sc.y && a.u.sc.w == b.u.sc.w &&
           a.u.sc.h == b.u.sc.h && a.u.sc.fontId == b.u.sc.fontId &&
           a.u.sc.color == b.u.sc.color && a.u.sc.speedMs == b.u.sc.speedMs &&
           a.u.sc.len == b.u.sc.len && memcmp(a.u.sc.text, b.u.sc.text, a.u.sc.len) == 0;
  case ACMD_PARAM_BLINK:
    return a.u.bl.x == b.u.bl.x && a.u.bl.y == b.u.bl.y && a.u.bl.w == b.u.bl.w &&
           a.u.bl.h == b.u.bl.h && a.u.bl.periodMs == b.u.bl.periodMs;
  default:
    return false;
  }
}

/** ACMD v1 batch parser. FIXED commands: opcode + fixed args. PAYLOAD commands
 *  (BLIT/FONT/TEXT/SCROLL): opcode + payloadLen u16 LE + fixedArgs + payload, where
 *  payloadLen counts ONLY the trailing payload bytes. Any opcode outside the v1 table
 *  stops parsing and commits the parsed prefix; any truncation or out-of-bounds
 *  argument drops the whole batch and keeps the prior display. */
void handleAcmd(const byte *payload, unsigned int length)
{
  if (length < ACMD_HEADER_BYTES)
    return;
  uint32_t magic;
  memcpy(&magic, payload, sizeof(magic));
  if (magic != ACMD_MAGIC || payload[4] != ACMD_VERSION)
    return;
  uint16_t cmdCount = acmdU16(payload + 5);
  size_t pos = ACMD_HEADER_BYTES;

  memset(acmdWork, 0, sizeof(acmdWork)); // each batch fully describes the frame (starts black)

  // Parametric candidates — collected during parse, applied to the live state only at
  // commit so a dropped batch never corrupts a running overlay. Up to ACMD_PARAMS_MAX
  // in command order; later ones are validated but ignored. Static storage: ~1 KB is
  // too much for the MQTT callback's stack, and ACMD handling is single-threaded
  // (loop task); candCount resets per call.
  static AcmdParam cand[ACMD_PARAMS_MAX];
  size_t candCount = 0;

  bool stopParsing = false;
  for (uint16_t ci = 0; ci < cmdCount && !stopParsing; ci++)
  {
    if (pos >= length)
      return; // truncation → drop whole batch, keep prior display
    uint8_t op = payload[pos++];
    switch (op)
    {
    case ACMD_NOP:
      break;

    case ACMD_CLS:
    {
      if (pos + 2 > length)
        return;
      uint16_t color = acmdU16(payload + pos);
      pos += 2;
      for (size_t i = 0; i < W * H; i++)
        acmdWork[i] = color;
      break;
    }

    case ACMD_PIX:
    {
      if (pos + 4 > length)
        return;
      uint8_t x = payload[pos], y = payload[pos + 1];
      uint16_t color = acmdU16(payload + pos + 2);
      pos += 4;
      acmdPixel(acmdWork, x, y, color);
      break;
    }

    case ACMD_LINE:
    {
      if (pos + 6 > length)
        return;
      uint8_t x0 = payload[pos], y0 = payload[pos + 1], x1 = payload[pos + 2], y1 = payload[pos + 3];
      uint16_t color = acmdU16(payload + pos + 4);
      pos += 6;
      acmdWriteLine(acmdWork, x0, y0, x1, y1, color);
      break;
    }

    case ACMD_RECT:
    {
      if (pos + 6 > length)
        return;
      uint8_t x = payload[pos], y = payload[pos + 1], w = payload[pos + 2], h = payload[pos + 3];
      uint16_t color = acmdU16(payload + pos + 4);
      pos += 6;
      acmdDrawRect(acmdWork, x, y, w, h, color);
      break;
    }

    case ACMD_FILL:
    {
      if (pos + 6 > length)
        return;
      uint8_t x = payload[pos], y = payload[pos + 1], w = payload[pos + 2], h = payload[pos + 3];
      uint16_t color = acmdU16(payload + pos + 4);
      pos += 6;
      acmdFillRect(acmdWork, x, y, w, h, color);
      break;
    }

    case ACMD_CIRC:
    {
      if (pos + 5 > length)
        return;
      uint8_t cx = payload[pos], cy = payload[pos + 1], r = payload[pos + 2];
      uint16_t color = acmdU16(payload + pos + 3);
      pos += 5;
      acmdDrawCircle(acmdWork, cx, cy, r, color);
      break;
    }

    case ACMD_BLIT:
    {
      if (pos + 2 > length)
        return;
      uint16_t payLen = acmdU16(payload + pos);
      pos += 2;
      if (pos + 4 + (size_t)payLen > length)
        return;
      uint8_t x = payload[pos], y = payload[pos + 1], w = payload[pos + 2], h = payload[pos + 3];
      pos += 4;
      const uint8_t *pay = payload + pos;
      pos += payLen;

      if (payLen < PAL_RLE_PALETTE_BYTES || ((payLen - PAL_RLE_PALETTE_BYTES) & 1) != 0)
        return;
      uint16_t pal[16];
      for (uint8_t c = 0; c < 16; c++)
        pal[c] = acmdU16(pay + 2 * c);
      // Structural validation: runs never cross rows, each row sums to exactly w,
      // run 1..w, colorIdx 0..15, and the pairs cover all w*h pixels.
      const uint8_t *pairs = pay + PAL_RLE_PALETTE_BYTES;
      size_t pairCount = (payLen - PAL_RLE_PALETTE_BYTES) / 2;
      uint16_t row = 0;
      uint16_t rowsDone = 0;
      for (size_t i = 0; i < pairCount; i++)
      {
        uint8_t run = pairs[2 * i], idx = pairs[2 * i + 1];
        if (run < 1 || run > w || idx > 15)
          return;
        row += run;
        if (row > w)
          return;
        if (row == w)
        {
          row = 0;
          rowsDone++;
        }
      }
      if (row != 0 || rowsDone != h)
        return;
      // Decode and blit at (x,y), canvas-clipped (out-of-bounds dropped).
      uint16_t col = 0;
      uint16_t rowIdx = 0;
      for (size_t i = 0; i < pairCount; i++)
      {
        uint16_t color = pal[pairs[2 * i + 1]];
        for (uint8_t k = 0; k < pairs[2 * i]; k++)
        {
          acmdPixel(acmdWork, (int32_t)x + col, (int32_t)y + rowIdx, color);
          if (++col == w)
          {
            col = 0;
            rowIdx++;
          }
        }
      }
      break;
    }

    case ACMD_FONT:
    {
      if (pos + 2 > length)
        return;
      uint16_t payLen = acmdU16(payload + pos);
      pos += 2;
      if (pos + (size_t)payLen > length)
        return; // no fixed args for FONT
      const uint8_t *pay = payload + pos;
      pos += payLen;

      if (payLen < 2)
        return;
      uint8_t pageId = pay[0], glyphCount = pay[1];
      if (pageId >= ACMD_FONT_PAGES)
        return;
      // Pass 1: validate layout (w/h 1..32) and exact payload consumption.
      size_t gpos = 2;
      for (uint16_t g = 0; g < glyphCount; g++)
      {
        if (gpos + 6 > payLen)
          return;
        uint8_t w = pay[gpos + 1], h = pay[gpos + 2];
        if (w < 1 || w > 32 || h < 1 || h > 32)
          return;
        gpos += 6 + (size_t)h * ((w + 7) / 8);
      }
      if (gpos != payLen)
        return;
      // Pass 2: stage the page, then swap in (a malloc failure keeps the old page).
      size_t total = payLen - 2;
      uint8_t *buf = total > 0 ? (uint8_t *)malloc(total) : nullptr;
      if (total > 0 && !buf)
        return;
      static uint16_t tmpOff[256]; // single-threaded firmware: shared scratch is safe
      for (uint16_t c = 0; c < 256; c++)
        tmpOff[c] = ACMD_GLYPH_ABSENT;
      if (total > 0)
        memcpy(buf, pay + 2, total);
      size_t off = 0;
      for (uint16_t g = 0; g < glyphCount; g++)
      {
        tmpOff[buf[off]] = off; // duplicate codes: last one wins
        off += 6 + (size_t)buf[off + 2] * ((buf[off + 1] + 7) / 8);
      }
      AcmdFontPage &pg = acmdFonts[pageId];
      free(pg.glyphs);
      pg.present = true;
      pg.glyphs = buf;
      pg.glyphsLen = total;
      memcpy(pg.codeOff, tmpOff, sizeof(pg.codeOff));
      break;
    }

    case ACMD_TEXT:
    {
      if (pos + 2 > length)
        return;
      uint16_t payLen = acmdU16(payload + pos);
      pos += 2;
      if (pos + 5 + (size_t)payLen > length)
        return;
      uint8_t fontId = payload[pos], x = payload[pos + 1], y = payload[pos + 2];
      uint16_t color = acmdU16(payload + pos + 3);
      pos += 5;
      const uint8_t *pay = payload + pos;
      pos += payLen;

      if (fontId >= ACMD_FONT_PAGES)
        return;
      const AcmdFontPage &pg = acmdFonts[fontId];
      if (!pg.present)
        return; // missing page → drop batch
      if (payLen < 1 || pay[0] < 1 || (size_t)payLen != 1 + pay[0])
        return;
      int32_t penX = x;
      for (uint8_t i = 0; i < pay[0]; i++)
      {
        uint16_t off = pg.codeOff[pay[1 + i]];
        if (off != ACMD_GLYPH_ABSENT)
        {
          acmdDrawGlyph(acmdWork, pg.glyphs + off, penX, y, color, false, 0, 0, 0, 0);
          penX += (int8_t)pg.glyphs[off + 3];
        }
        else
          penX += 4; // unknown glyph: advance, draw nothing
      }
      break;
    }

    case ACMD_SWEEP:
    {
      if (pos + 6 > length)
        return;
      uint8_t cx = payload[pos], cy = payload[pos + 1], r = payload[pos + 2];
      uint16_t color = acmdU16(payload + pos + 3);
      uint8_t speed = payload[pos + 5];
      pos += 6;
      if (speed < 1)
        return; // spec: 1..255
      if (candCount < ACMD_PARAMS_MAX)
      {
        AcmdParam &p = cand[candCount++];
        p.type = ACMD_PARAM_SWEEP;
        p.u.sw = {cx, cy, r, speed, color};
      }
      break;
    }

    case ACMD_SCROLL:
    {
      if (pos + 2 > length)
        return;
      uint16_t payLen = acmdU16(payload + pos);
      pos += 2;
      if (pos + 9 + (size_t)payLen > length)
        return;
      uint8_t x = payload[pos], y = payload[pos + 1], w = payload[pos + 2], h = payload[pos + 3];
      uint8_t fontId = payload[pos + 4];
      uint16_t color = acmdU16(payload + pos + 5);
      uint16_t speedMs = acmdU16(payload + pos + 7);
      pos += 9;
      const uint8_t *pay = payload + pos;
      pos += payLen;

      if (fontId >= ACMD_FONT_PAGES)
        return;
      const AcmdFontPage &pg = acmdFonts[fontId];
      if (!pg.present)
        return; // missing page → drop batch
      if (payLen < 1 || pay[0] < 1 || (size_t)payLen != 1 + pay[0])
        return;
      if (speedMs < 1)
        return; // divide-by-zero guard
      // textWidth = Σ advance: glyph xAdvance (signed), +4 per unknown glyph.
      int32_t textW = 0;
      for (uint8_t i = 0; i < pay[0]; i++)
      {
        uint16_t off = pg.codeOff[pay[1 + i]];
        textW += (off != ACMD_GLYPH_ABSENT) ? (int8_t)pg.glyphs[off + 3] : 4;
      }
      if (candCount < ACMD_PARAMS_MAX)
      {
        AcmdParam &p = cand[candCount++];
        p.type = ACMD_PARAM_SCROLL;
        p.u.sc.x = x;
        p.u.sc.y = y;
        p.u.sc.w = w;
        p.u.sc.h = h;
        p.u.sc.fontId = fontId;
        p.u.sc.color = color;
        p.u.sc.speedMs = speedMs;
        p.u.sc.len = pay[0];
        memcpy(p.u.sc.text, pay + 1, p.u.sc.len);
        p.u.sc.text[p.u.sc.len] = 0;
        p.u.sc.textW = textW;
      }
      break;
    }

    case ACMD_BLINK:
    {
      if (pos + 6 > length)
        return;
      uint8_t x = payload[pos], y = payload[pos + 1], w = payload[pos + 2], h = payload[pos + 3];
      uint16_t periodMs = acmdU16(payload + pos + 4);
      pos += 6;
      if (periodMs < 1)
        return;
      if (candCount < ACMD_PARAMS_MAX)
      {
        AcmdParam &p = cand[candCount++];
        p.type = ACMD_PARAM_BLINK;
        p.u.bl = {x, y, w, h, periodMs};
      }
      break;
    }

    default:
      // Opcode not in the v1 table → stop parsing and commit the parsed prefix.
      // (Forward-compat: future opcodes must be payload-form so extended parsers can
      // skip them as fixedArgsLen + 2 + payloadLen; a v1 parser cannot, so it stops.)
      stopParsing = true;
      break;
    }
  }

  // Commit: swap work into base, stop animation playback (slot files kept), arm the
  // parametric sequence, push the frame. Everything after this point is live state.
  // Epoch carry-over: a batch whose armed parametric SEQUENCE is identical to the live
  // one (same count, order, types and parameters — SCROLL text included) keeps the
  // previous epoch — every overlay's phase runs continuously across refresh
  // republishes, so QoS-0 arrival jitter cannot snap the sweep back to 0° or restart a
  // marquee mid-word. Any change (a different sequence, fewer/more parametrics, none),
  // or anything that clears the ACMD state (stopAnimation clears acmdActive) arms all
  // parametrics fresh. Decided BEFORE stopAnimation.
  bool carryEpoch = acmdActive && acmdLiveCount == candCount && candCount > 0;
  for (size_t i = 0; carryEpoch && i < candCount; i++)
    carryEpoch = acmdParamsEqual(acmdLive[i], cand[i]);
  memcpy(acmdBase, acmdWork, sizeof(acmdBase));
  stopAnimation(false);
  for (size_t i = 0; i < candCount; i++)
    acmdLive[i] = cand[i];
  acmdLiveCount = candCount;
  acmdCommitMs = carryEpoch ? acmdCommitMs : millis(); // carry: keep the epoch
  acmdLastDrawMs = acmdCommitMs; // carried epoch is old → next tick fires immediately
  acmdActive = true;
  currentScreenImage = ScreenImage::Client;
  dma_display->drawRGBBitmap(0, 0, acmdWork, W, H);
  Serial.printf("ACMD committed: %u commands%s\n", cmdCount,
                acmdLiveCount == 0 ? "" : " + parametric");
}

/** Parametric overlay tick (~10 ms floor, called from loop()): composites ALL armed
 * overlays over a fresh copy of the immutable base canvas and pushes it. All animation
 * state derives from elapsed milliseconds since commit — never from tick counts — so
 * loop cadence jitter cannot drift or stall a phase. Overlays apply in command order,
 * so a later overlay draws over an earlier one where their regions overlap. */
void acmdTick()
{
  if (!acmdActive || acmdLiveCount == 0)
    return;
  unsigned long now = millis();
  if (now - acmdLastDrawMs < ACMD_TICK_MS)
    return;
  acmdLastDrawMs = now;
  unsigned long elapsed = now - acmdCommitMs;

  // Fresh base copy = the per-overlay "region snapshot restore" semantics.
  memcpy(acmdWork, acmdBase, sizeof(acmdWork));
  for (size_t pi = 0; pi < acmdLiveCount; pi++)
  {
    const AcmdParam &p = acmdLive[pi];
    switch (p.type)
    {
    case ACMD_PARAM_SWEEP:
    {
      // θ = (elapsedMs × speed / 1000) mod 360; endpoint in double, lround (half away
      // from zero); GFX line from the center over the base each tick.
      uint32_t deg = (uint32_t)(((uint64_t)elapsed * p.u.sw.speed / 1000) % 360);
      double rad = (double)deg * M_PI / 180.0;
      int16_t ex = (int16_t)lround((double)p.u.sw.cx + (double)p.u.sw.r * cos(rad));
      int16_t ey = (int16_t)lround((double)p.u.sw.cy + (double)p.u.sw.r * sin(rad));
      acmdWriteLine(acmdWork, p.u.sw.cx, p.u.sw.cy, ex, ey, p.u.sw.color);
      break;
    }
    case ACMD_PARAM_SCROLL:
    {
      // Ping-pong marquee with readability pacing: penX oscillates between the
      // head-visible extreme (penX = x, tail clipped right) and the tail-visible
      // extreme (penX = x + w − textW, head clipped left) — the text never leaves
      // the region — HOLDING ACMD_SCROLL_HOLD_PX px-units of time at each extreme
      // before reversing. One pass travels travel = textW − w px in at least
      // ACMD_SCROLL_MIN_PASS_PX px-units (a barely-overflowing text glides instead
      // of rattling); one px-unit is speedMs. Phase 0 = the head hold. A text that
      // fits (textW ≤ w) has no travel: it renders statically at x.
      int32_t travel = (int32_t)p.u.sc.textW - (int32_t)p.u.sc.w;
      int32_t penX;
      if (travel < 1)
        penX = p.u.sc.x;
      else
      {
        int32_t vt = travel < (int32_t)ACMD_SCROLL_MIN_PASS_PX ? (int32_t)ACMD_SCROLL_MIN_PASS_PX : travel;
        uint32_t passMs = (uint32_t)vt * p.u.sc.speedMs;
        uint32_t holdMs = (uint32_t)ACMD_SCROLL_HOLD_PX * p.u.sc.speedMs;
        uint32_t cyc = elapsed % (2 * (holdMs + passMs));
        int32_t pen; // px left of the head extreme, 0..travel
        if (cyc < holdMs)
          pen = 0; // head hold
        else if (cyc < holdMs + passMs)
          pen = (int32_t)((uint64_t)(cyc - holdMs) * (uint32_t)travel / passMs); // right → left pass
        else if (cyc < 2 * holdMs + passMs)
          pen = travel; // tail hold
        else
          pen = travel - (int32_t)((uint64_t)(cyc - 2 * holdMs - passMs) * (uint32_t)travel / passMs); // left → right pass
        penX = p.u.sc.x - pen;
      }
      const AcmdFontPage &pg = acmdFonts[p.u.sc.fontId];
      for (uint8_t gi = 0; gi < p.u.sc.len; gi++)
      {
        uint16_t off = pg.codeOff[(uint8_t)p.u.sc.text[gi]];
        if (off != ACMD_GLYPH_ABSENT)
        {
          acmdDrawGlyph(acmdWork, pg.glyphs + off, penX, p.u.sc.y, p.u.sc.color,
                        true, p.u.sc.x, p.u.sc.y, p.u.sc.w, p.u.sc.h);
          penX += (int8_t)pg.glyphs[off + 3];
        }
        else
          penX += 4;
      }
      break;
    }
    case ACMD_PARAM_BLINK:
    {
      // Alternate content ↔ black every periodMs/2; the first half shows content
      // (content = the base copy already in acmdWork).
      if ((elapsed % p.u.bl.periodMs) >= p.u.bl.periodMs / 2)
        for (int16_t py = p.u.bl.y; py < p.u.bl.y + p.u.bl.h; py++)
          for (int16_t px = p.u.bl.x; px < p.u.bl.x + p.u.bl.w; px++)
            acmdPixel(acmdWork, px, py, 0);
      break;
    }
    default:
      break;
    }
  }
  dma_display->drawRGBBitmap(0, 0, acmdWork, W, H);
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
      // Defer to loop(): handleAnimStart does LittleFS removes/creates (flash erases)
      // that stall playback inline. Ordering is safe — PubSubClient delivers one publish
      // per client.loop() call, and loop() runs the staged start before the next
      // client.loop() can deliver the first frame.
      if (length == ANIM_START_PAYLOAD)
      {
        memcpy(animStartPayload, payload, length);
        animStartPending = true;
      }
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
    if (strcmp(suffix, TOPIC_CMD) == 0)
    {
      handleAcmd(payload, length);
      return;
    }
  }

#ifdef PC75_LOG_FRAMES
  Serial.printf("Received %u bytes on topic %s\n", length, topic);
#endif

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

/** One-time MQTT client configuration. Never connects and never draws/delays here: a
 *  down broker must not block setup()/loop(). reconnect() owns the throttled attempts
 *  and loop() draws the outage UI once per disconnected period. */
void init_broker_connection()
{
  if (brokerConfigured)
    return;
  brokerConfigured = true;
  updateScreen = true;

  Serial.println("Connected to Wifi, connecting to broker...");
  Serial.println(WiFi.localIP());

  // static: PubSubClient::setServer only stores the pointer, so the buffer must
  // outlive this function or connect() resolves DNS through dangling stack memory
  static char server[PREF_STR_MAX];
  readPrefString("server", server, sizeof(server));

  const uint16_t mqtt_port = preferences.getUShort("server_port", 1883);

  Serial.print("\nServer: ");
  Serial.println(server);
  Serial.print("Port: ");
  Serial.println(mqtt_port);

  client.setServer(server, mqtt_port);
  client.setCallback(callback);
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
      char cmdTopic[48];
      snprintf(animStartTopic, sizeof(animStartTopic), "%s%s", getClientId(), TOPIC_ANIM_START);
      snprintf(animFrameTopic, sizeof(animFrameTopic), "%s%s", getClientId(), TOPIC_ANIM_FRAME);
      snprintf(animPlayTopic, sizeof(animPlayTopic), "%s%s", getClientId(), TOPIC_ANIM_PLAY);
      snprintf(cmdTopic, sizeof(cmdTopic), "%s%s", getClientId(), TOPIC_CMD);
      Serial.printf("Subscribed to %s: %s\n", animStartTopic, client.subscribe(animStartTopic) ? "ok" : "FAILED");
      Serial.printf("Subscribed to %s: %s\n", animFrameTopic, client.subscribe(animFrameTopic) ? "ok" : "FAILED");
      Serial.printf("Subscribed to %s: %s\n", animPlayTopic, client.subscribe(animPlayTopic) ? "ok" : "FAILED");
      Serial.printf("Subscribed to %s: %s\n", cmdTopic, client.subscribe(cmdTopic) ? "ok" : "FAILED"); // QoS 0
      if (animUploading)
        abortAnimUpload(); // an in-flight upload died with the connection; completed staging survives
      startAnimation(animPlayingSlot); // slot files persist across reconnects
    }
    else
    {
      // No redraw here: the outage UI is drawn once by loop() (and only when no
      // animation is playing); redrawing per failed attempt would fight it.
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

void setup()
{
  Serial.begin(115200);

  pinMode(WF2_BUTTON_TEST, INPUT_PULLUP);

  client.setBufferSize(MAX_PAYLOAD_SIZE); // once: per-reconnect reallocs fragment the heap

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
  loadSlotUploadIds();               // slot content-hash index; invalid/missing file loads zeros (safe)
  Serial.printf("LittleFS: %u / %u bytes used\n",
                (unsigned)LittleFS.usedBytes(), (unsigned)LittleFS.totalBytes());

  preferences.begin("cryptoticker", false);

  char ssid[PREF_STR_MAX];
  char password[PREF_STR_MAX];
  readPrefString("wifissid", ssid, sizeof(ssid));
  readPrefString("wifipass", password, sizeof(password));

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
  if (digitalRead(WF2_BUTTON_TEST) == LOW && !buttonPressed)
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
    // Non-blocking reconnect state machine: configure once, draw the outage UI once per
    // disconnected period, then rely on reconnect()'s 5 s throttle. A broker outage must
    // not spin the loop with delays/redraws that clobber a playing animation.
    if (!client.connected())
    {
      init_broker_connection();
      if (!mqttConnectingUiShown)
      {
        mqttConnectingUiShown = true;
        if (!animActive) // a playing slot animation already shows life; don't clobber it
          drawBitmap(0, 0, epd_bitmap_connecting, 64, 32, ScreenImage::Connecting, true);
      }
      reconnect();
    }
    else
    {
      mqttConnectingUiShown = false;
    }

    // Staged by the MQTT callback, run here before the next client.loop() can deliver
    // the first ANIF: flash erases never run inside the callback.
    if (animStartPending && client.connected())
    {
      animStartPending = false;
      handleAnimStart(animStartPayload, ANIM_START_PAYLOAD);
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
    acmdTick(); // parametric overlay (SWEEP/SCROLL/BLINK); no-op unless ACMD is active
  }
  else
  {
    delay(10);
  }
}
