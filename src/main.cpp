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

#define PANEL_RES_X 64  // Number of pixels wide of each INDIVIDUAL panel module.
#define PANEL_RES_Y 32  // Number of pixels tall of each INDIVIDUAL panel module.
#define PANEL_CHAIN 1   // Total number of panels chained one to another

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
#define WF2_X2_E_PIN -1  // Currently unknown, so X2 port will not work (yet) with 1/32 scan panels

#define WF2_A_PIN 39
#define WF2_B_PIN 38
#define WF2_C_PIN 37
#define WF2_D_PIN 36
#define WF2_OE_PIN 35
#define WF2_CLK_PIN 34
#define WF2_LAT_PIN 33

#define WF2_BUTTON_TEST 17     // Test key button on PCB, 1=normal, 0=pressed
#define WF2_LED_RUN_PIN 40     // Status LED on PCB
#define WF2_BM8563_I2C_SDA 41  // RTC BM8563 I2C port
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

HUB75_I2S_CFG::i2s_pins _pins_x1 = { WF2_X1_R1_PIN, WF2_X1_G1_PIN, WF2_X1_B1_PIN, WF2_X1_R2_PIN, WF2_X1_G2_PIN, WF2_X1_B2_PIN, WF2_A_PIN, WF2_B_PIN, WF2_C_PIN, WF2_D_PIN, WF2_X1_E_PIN, WF2_LAT_PIN, WF2_OE_PIN, WF2_CLK_PIN };
HUB75_I2S_CFG::i2s_pins _pins_x2 = { WF2_X2_R1_PIN, WF2_X2_G1_PIN, WF2_X2_B1_PIN, WF2_X2_R2_PIN, WF2_X2_G2_PIN, WF2_X2_B2_PIN, WF2_A_PIN, WF2_B_PIN, WF2_C_PIN, WF2_D_PIN, WF2_X2_E_PIN, WF2_LAT_PIN, WF2_OE_PIN, WF2_CLK_PIN };

MatrixPanel_I2S_DMA *dma_display = nullptr;

WiFiMulti wiFiMulti;
Preferences preferences;
WiFiClient espClient;
PubSubClient client(espClient);

bool hasWifi = false;
bool updateScreen = false;
bool deviceConnected = false;
bool bluetoothInitCompleted = false;
bool buttonPressed = false;

unsigned long lastReconnectAttempt = 0;

static constexpr int W = 64;
static constexpr int H = 32;
static constexpr size_t FRAME_BYTES = W * H * 2;

uint8_t  rxBuf[FRAME_BYTES];      // raw bytes from MQTT
uint16_t *px = (uint16_t*)rxBuf;  // view as RGB565 pixels (little-endian)

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

bool init_wifi(char ssid[], char password[]) {
  dma_display->clearScreen();

  drawXbm565(dma_display, 0, 0, 64, 32, wifi_image1bit, dma_display->color565(0, 0, 255));

  delay(1000);

  wiFiMulti.addAP(ssid, password);

  Serial.println();
  Serial.println();
  Serial.print("Waiting for WiFi... ");

  while (wiFiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);

    if (millis() > 1000) break;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wifi connected!");
    hasWifi = true;
    dma_display->drawRGBBitmap(38, 23, epd_bitmap_check_mark, 8, 8);
    delay(3000);
  } else {
    hasWifi = false;
    Serial.println("ERROR: Failed to connect to wifi!");
    dma_display->clearScreen();
    dma_display->drawRGBBitmap(16, 0, epd_bitmap_bluetooth_icon, 32, 32);
    dma_display->drawRGBBitmap(40, 20, epd_bitmap_search, 8, 8);

    init_bluetooth();
  }

  delay(1000);

  return hasWifi;
}

class ServerCharacteristicCallBack : public BLECharacteristicCallbacks {
public:
  void onWrite(BLECharacteristic *characteristic) {
    String value = String(characteristic->getValue().c_str());
    value.trim();

    byte serverBytes[value.length() + 1];
    value.getBytes(serverBytes, value.length() + 1);
    preferences.putBytes("server", serverBytes, value.length() + 1);
    Serial.println("Server received: " + value);
  }
};

class SsidCharacteristicCallBack : public BLECharacteristicCallbacks {
public:
  void onWrite(BLECharacteristic *characteristic) {
    // Handle the written data here
    String value = String(characteristic->getValue().c_str());
    value.trim();

    byte ssidBytes[value.length() + 1];
    value.getBytes(ssidBytes, value.length() + 1);
    preferences.putBytes("wifissid", ssidBytes, value.length() + 1);
    Serial.println("SSID received: " + value);
  }
};

class PasswordCharacteristicCallBack : public BLECharacteristicCallbacks {
public:
  void onWrite(BLECharacteristic *characteristic) {
    String value = String(characteristic->getValue().c_str());
    value.trim();

    byte passBytes[value.length() + 1];
    value.getBytes(passBytes, value.length() + 1);
    preferences.putBytes("wifipass", passBytes, value.length() + 1);
    Serial.println("Password received: " + value);
  }
};

class BrightnessCharacteristicCallBack : public BLECharacteristicCallbacks {
public:
  void onWrite(BLECharacteristic *characteristic) override {
    // The standard ESP32 BLE lib often returns Arduino String here
    String raw = String(characteristic->getValue().c_str());

    Serial.printf("Brightness write: %d bytes\n", raw.length());

    if (raw.length() == 0) {
      Serial.println("Brightness write was empty; ignoring.");
      return;
    }

    // Make a trimmed copy for ASCII parsing, but keep 'raw' for raw-byte fallback
    String s = raw;
    s.trim();

    uint16_t value = 0;
    if (s.length() > 0 && isAllDigits(s)) {
      // App sends ASCII decimal: "10".."255"
      value = (uint16_t)s.toInt();
      Serial.printf("Parsed ASCII brightness: %u\n", (unsigned)value);
    } else {
      // Fallback: treat first byte as the value (in case of raw write)
      value = static_cast<uint8_t>(raw[0]);
      Serial.printf("Parsed RAW brightness (first byte): %u\n", (unsigned)value);
    }

    // Clamp to your allowed range
    if (value < 10) value = 10;
    if (value > 255) value = 255;

    preferences.putUChar("brightness", static_cast<uint8_t>(value));
    Serial.printf("Brightness received (final): %u\n", (unsigned)value);
    dma_display->setBrightness(static_cast<uint8_t>(value));
  }

private:
  static bool isAllDigits(const String &s) {
    for (size_t i = 0; i < s.length(); ++i) {
      if (!isDigit(static_cast<unsigned char>(s[i]))) return false;
    }
    return true;
  }
};

class ServerPortCharacteristicCallBack : public BLECharacteristicCallbacks {
public:
  void onWrite(BLECharacteristic *characteristic) override {
    // getValue() can be Arduino String or std::string depending on version.
    // Convert robustly to Arduino String via c_str().
    String raw;
    {
      auto v = characteristic->getValue();  // String OR std::string
      raw = String(v.c_str());              // make an owned Arduino String
    }

    Serial.printf("ServerPort write: %d bytes\n", raw.length());
    if (raw.length() == 0) {
      Serial.println("Port value empty; ignoring.");
      return;
    }

    // Trim for ASCII parsing
    String s = raw;
    s.trim();

    uint32_t port = 0;
    if (s.length() > 0 && isAllDigits(s)) {
      // App path: ASCII decimal, e.g. "1883"
      port = (uint32_t)s.toInt();
      Serial.printf("Parsed ASCII port: %u\n", (unsigned)port);
    } else {
      // Fallbacks for non-ASCII writes:
      if (raw.length() >= 2) {
        // Interpret first two bytes as network-order (big endian) uint16
        uint8_t b0 = (uint8_t)raw[0];
        uint8_t b1 = (uint8_t)raw[1];
        port = ((uint16_t)b0 << 8) | b1;
        Serial.printf("Parsed RAW 2-byte port (network order): %u\n", (unsigned)port);
      } else {
        // Single byte fallback
        port = (uint8_t)raw[0];
        Serial.printf("Parsed RAW 1-byte port: %u\n", (unsigned)port);
      }
    }

    // Clamp to valid TCP port range
    if (port < 1) port = 1;
    if (port > 65535) port = 65535;

    // Persist + apply
    preferences.putUShort("server_port", (uint16_t)port);
    Serial.printf("Server port saved: %u\n", (unsigned)port);
  }

private:
  static bool isAllDigits(const String &s) {
    for (size_t i = 0; i < s.length(); ++i) {
      if (!isDigit((unsigned char)s[i])) return false;
    }
    return true;
  }
};


class RestartCharacteristicCallBack : public BLECharacteristicCallbacks {
public:
  void onWrite(BLECharacteristic *characteristic) {
    ESP.restart();
  }
};

class BLEConnectionCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("***** Connect");

    dma_display->clearScreen();
    dma_display->drawRGBBitmap(16, 0, epd_bitmap_bluetooth_icon, 32, 32);
    dma_display->drawRGBBitmap(40, 20, epd_bitmap_check_mark, 8, 8);
  }
  void onDisconnect(BLEServer *pServer) {
    Serial.println("***** Disconnect");
    deviceConnected = false;
    updateScreen = true;
    dma_display->drawRGBBitmap(16, 0, epd_bitmap_bluetooth_icon, 32, 32);
    dma_display->drawRGBBitmap(40, 20, epd_bitmap_red_cross, 7, 7);

    delay(2000);

    BLEDevice::stopAdvertising();

    ESP.restart();
  }
};

void init_bluetooth() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  Serial.println("No WiFi configured, switching on bluetooth for configuration...");
  dma_display->clearScreen();
  dma_display->drawRGBBitmap(16, 0, epd_bitmap_bluetooth_icon, 32, 32);
  dma_display->drawRGBBitmap(40, 20, epd_bitmap_search, 8, 8);
  updateScreen = false;

  if (!bluetoothInitCompleted) {
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
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
  }

  BLEDevice::startAdvertising();

  bluetoothInitCompleted = true;
}

void setAsciiValue(BLECharacteristic *ch, const String &val) {
  ch->setValue((uint8_t *)val.c_str(), val.length());
}

void callback(char *topic, byte *payload, unsigned int length) {
  if (updateScreen) {
    Serial.printf("Received %u bytes on topic %s\n", length, topic);

    if (length > MAX_PAYLOAD_SIZE) {
      Serial.println("Payload exceeds supported size");
      return;
    }

    if (length != FRAME_BYTES) return;          // ignore malformed frames
    memcpy(rxBuf, payload, FRAME_BYTES);
    
    dma_display -> drawRGBBitmap(0, 0, px, W, H);
  }
}

void init_broker_connection() {
  Serial.println("Connected to Wifi, connecting to broker...");

  dma_display->clearScreen();
  dma_display->drawRGBBitmap(0, 0, epd_bitmap_connecting, 64, 32);
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

  if(!client.connected()) {
    updateScreen = true;
    Serial.println(WiFi.localIP());
    reconnect();
  }
}

const char *getClientId() {
  static char client_id[32];  // enough for "PXCORE75-" + MAC (12 chars) + '\0'

  if (client_id[0] == '\0') {        // build only once
    String mac = WiFi.macAddress();  // we still need this helper
    mac.replace(":", "");

    snprintf(client_id, sizeof(client_id), "PXCORE75-%s", mac.c_str());

    Serial.println();
    Serial.println();
    Serial.println("************************");
    Serial.println("* REGISTRATION DETAILS *");
    Serial.println("************************");
    Serial.print("-> Mac address: ");
    Serial.println(mac);
    Serial.print("-> Client ID / Serial: ");
    Serial.println(client_id);
    Serial.println();
    Serial.println();
  }

  return client_id;
}

void writeFile(fs::FS &fs, const char *path, const uint16_t *intArray) {
  Serial.printf("Writing file: %s\r\n", path);

  //if(!fs.exists(path)) {
  // Define your uint16_t array
  size_t dataSize = 2048 * sizeof(uint16_t);

  Serial.print("Estimated size: ");
  Serial.println(dataSize);

  // Open file for binary writing
  File file = LittleFS.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  // Write the array as binary data
  size_t written = file.write((const uint8_t *)intArray, dataSize);
  file.close();

  // Check if all data was written
  if (written == dataSize) {
    Serial.print("Data written successfully: ");
    Serial.println(written);
  } else {
    Serial.printf("Only %u of %u bytes written.\n", written, dataSize);
  }
  // } else {
  //   Serial.println("File already on filesystem, skipping write!");
  // }
}

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\r\n", path);

  File file = LittleFS.open(path, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  const size_t numElements = 2048;
  uint16_t readBuffer[numElements];

  size_t bytesRead = file.read((uint8_t *)readBuffer, sizeof(readBuffer));
  file.close();

  if (bytesRead == sizeof(readBuffer)) {
    Serial.println("Data read successfully:");

    dma_display->clearScreen();
    dma_display->drawRGBBitmap(0, 0, readBuffer, 64, 32);
  } else {
    Serial.printf("Only %u of %u bytes read.\n", bytesRead, sizeof(readBuffer));
  }
}

void reconnect() {
  long reconnectDelay = millis() - lastReconnectAttempt;
  if (reconnectDelay > 5000) {
    Serial.println();
    Serial.println("Took time: ");
    Serial.print(reconnectDelay);
    Serial.println();
    Serial.printf("The client %s connects to the public MQTT broker\n", getClientId());
    if (client.connect(getClientId())) {
      Serial.println("connected");
      dma_display->clearScreen();
      dma_display->drawRGBBitmap(0, 0, epd_bitmap_connected, 64, 32);
      dma_display->drawRGBBitmap(48, 23, epd_bitmap_check_mark, 8, 8);
      client.subscribe(getClientId());
    } else {
      dma_display->clearScreen();
      dma_display->drawRGBBitmap(0, 0, epd_bitmap_connecting, 64, 32);
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }

    lastReconnectAttempt = millis();
  }
}

void init_display() {
  HUB75_I2S_CFG mxconfig(
    PANEL_RES_X,  // module width
    PANEL_RES_Y,  // module height
    PANEL_CHAIN,  // Chain length
    _pins_x2
  );

  const uint8_t brightness = preferences.getUChar("brightness", 128);

  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->setBrightness(brightness);  //0-255
  dma_display->clearScreen();
}

void verifyRegistration() {
  uint64_t device_id = ESP.getEfuseMac();
}

void setup() {
  Serial.begin(115200);

  pinMode(17, INPUT_PULLUP);

  Serial.println("********************************************");
  Serial.print("* ");
  Serial.print(BUILD_NAME);
  Serial.print(" ");
  Serial.println(BUILD_VERSION);
  Serial.println("********************************************");

  if (!LittleFS.begin(FORMAT_LITTLE_FS_IF_FAILED)) {
    Serial.println("LittleFS Mount Failed");
    return;
  }

  preferences.begin("cryptoticker", false);

  char ssid[preferences.getBytesLength("wifissid")] = {};
  char password[preferences.getBytesLength("wifipass")] = {};

  preferences.getBytes("wifissid", ssid, preferences.getBytesLength("wifissid"));
  preferences.getBytes("wifipass", password, preferences.getBytesLength("wifipass"));

  init_display();

  dma_display->setFont(&Org_01);
  dma_display->clearScreen();
  dma_display->drawRGBBitmap(24, 1, epd_bitmap_pixelcore75_firmware_logo, 16, 16);
  dma_display->setCursor(6, 22);
  dma_display->println(BUILD_NAME);
  dma_display->setTextColor(dma_display->color565(0, 0, 255));
  dma_display->setCursor(19, 30);
  dma_display->println(BUILD_VERSION);

  delay(5000);

  if (!check_bluetooth_button_pressed()) {
    if (strlen(ssid) != 0 && strlen(password) != 0) {
      Serial.print("Trying to connect to WiFi: ");
      Serial.println(ssid);

      if (init_wifi(ssid, password)) {
        verifyRegistration();
        init_broker_connection();
      }
    } else {
      init_bluetooth();
    }
  }
}

bool check_bluetooth_button_pressed() {
  if (digitalRead(17) == LOW && !buttonPressed) {
    Serial.println("Bluetooth button pressed");
    updateScreen = false;
    init_bluetooth();
    return true;
  }

  return false;
}

void loop() {
  buttonPressed = check_bluetooth_button_pressed();

  if (hasWifi && !buttonPressed && updateScreen) {
    if (!client.connected()) {
      init_broker_connection();
    }
    client.loop();
  } else {
    delay(10);
  }
}
