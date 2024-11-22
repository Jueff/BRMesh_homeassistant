#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEServer.h"
#include "BLEBeacon.h"
#include <WiFi.h>
#include <ArduinoHA.h>
#include <String>
#include <cstdio>

//////////////////////////////////////////////////////
//CONFIGURATION
//////////////////////////////////////////////////////
//IP Address of your MQTT Broker (probably your Home Assistant host)
#define MQTT_BROKER_ADDR IPAddress(192,168,0,100)
//Your MQTT UserName
#define MQTT_BROKER_USER "User123"
//Your MQTT Password
#define MQTT_BROKER_PASS "password1234"
//Your WiFi SSID
#define WIFI_SSID "MyWifi"
//Your Wifi Password
#define WIFI_PASS "wifipassword"

const uint8_t my_key[] = { 0x38, 0x35, 0x31, 0x33 }; //Unique key from BRMesh app (found using USB debugging and adb logcat)

const int redundancy = 5;  // Repeats sending each command to the lights this many times; BLE broadcasting was flakey  

// LIGHT DEFINITION
// These are the lights and their names that will appear in Home Assistant
String mylightnames[] =
{
    "Flood 1",
    "Flood 2",
    "Flood 3",
    "Flood 4"
};

////////////////////////////////////////////////////
//                                               ///
// You probably don't need to modify below here  ///
//                                               ///
////////////////////////////////////////////////////

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);

char* deviceName;
uint64_t chipId;
byte mac[6];

void onRGBColorCommand(HALight::RGBColor color, HALight* sender);

BLEAdvertising *pAdvertising;   // BLE Advertisement type
#define BEACON_UUID "87b99b2c-90fd-11e9-bc42-526af7764f64" // UUID 1 128-Bit (may use linux tool uuidgen or random numbers via https://www.uuidgenerator.net/)



const uint8_t default_key[] = { 0x5e, 0x36, 0x7b, 0xc4 };

String toStringIp(IPAddress ip) {
  String res = "";
  for (int i = 0; i < 3; i++) {
    res += String((ip >> (8 * i)) & 0xFF) + ".";
  }
  res += String(((ip >> 8 * 3)) & 0xFF);
  return res;
}

void setup() {
  Serial.begin(115200);

  // Identity stuff
  chipId = ESP.getEfuseMac();
  Serial.printf("start ESP32 DEVICEID - %llX\n", ESP.getEfuseMac());

  deviceName = (char*)malloc(32);
  sprintf(deviceName, "BRMesh_%06X", (uint32_t)chipId&0xffffff);
  Serial.printf("Device name - %s\n", deviceName);
  
  uint32_t macL = (uint32_t)chipId;
  uint16_t macH = (uint16_t)(chipId>>32);
  mac[0] = macH>>8;
  mac[1] = macH;
  mac[2] = macL>>24;
  mac[3] = macL>>16;
  mac[4] = macL>>8;
  mac[5] = macL;

  //BLE STUFF
  // Create the BLE Device
  BLEDevice::init("ESP32 as iBeacon");

  pAdvertising = BLEDevice::getAdvertising();
  BLEDevice::startAdvertising();

  // CONNECT TO WIFI
  
  int tries = 0;
  
  Serial.printf("connecting to WIFI %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) 
  {
    // restart the ESP after 60 seconds without WIFI connection 
    if (++tries>120) 
    {
        ESP.restart();
    }
    delay(500); // waiting for the connection
    Serial.println("waiting for WIFI");
  }
  Serial.print("connected, device IP: ");
  Serial.println(toStringIp(WiFi.localIP()));

  //HOME ASSISTANT MQTT STUFF
        
  device.setUniqueId(mac, sizeof(mac));
  device.setName(deviceName);
  device.setManufacturer("BRMesh");
  device.setModel("BRMesh");

  int count = sizeof(mylightnames)/sizeof(String);
  Serial.printf("adding %d lights\n", count);

  for (int i=0; i<count && i<999; i++) 
  {
    char* id = (char*)malloc(4);
    sprintf(id, "%d", i+1);
    HALight* light = new HALight(id, HALight::BrightnessFeature | HALight::RGBFeature);
    light->setName(mylightnames[i].c_str());
    light->onStateCommand(onStateCommand);
    light->onBrightnessCommand(onBrightnessCommand); // optional
    light->onColorTemperatureCommand(onColorTemperatureCommand); // optional
    light->onRGBColorCommand(onRGBColorCommand); // optional
    light->setBrightnessScale(127);
    Serial.printf("added light %d: %s\n", i+1, mylightnames[i].c_str());
  }

  Serial.println("Starting MQTT");
  mqtt.begin(MQTT_BROKER_ADDR, 1883, MQTT_BROKER_USER, MQTT_BROKER_PASS);
}

void send(uint8_t* data, uint8_t dataLength);

uint8_t SEND_SEQ = 0;
uint8_t SEND_COUNT = 1;

const uint8_t BLE_CMD_RETRY_CNT = 1;
const uint8_t DEFAULT_BLE_FASTCON_ADDRESS[] = { 0xC1, 0xC2, 0xC3 };
const uint8_t addrLength = 3;

void dump(const uint8_t* data, int length)
{
  for (int i = 0; i < length; i++)
  {
    printf("%2.2X", data[i]);
  }
}

uint8_t package_ble_fastcon_body(int i, int i2, uint8_t sequence, uint8_t safe_key, int forward, const uint8_t* data, int length, const uint8_t* key, uint8_t*& payload)
{
  if (length > 12)
  {
    printf("data too long");
    payload = 0;
    return 0;
  }
  uint8_t payloadLength = 4 + 12;
  payload = (uint8_t*)malloc(payloadLength);
  payload[0] = (i2 & 0b1111) << 0 | (i & 0b111) << 4 | (forward & 0xff) << 7;
  payload[1] = sequence & 0xff;
  payload[2] = safe_key;
  payload[3] = 0; // checksum
  // fill payload with zeros
  for (int j = 4; j < payloadLength; j++) payload[j]=0;
  memcpy(payload + 4, data, length);

  uint8_t checksum = 0;
  for (int j = 0; j < length + 4; j++) 
  {
    if (j == 3) continue;
    checksum = (checksum + payload[j]) & 0xff;
  }
  payload[3] = checksum;
  for (int j = 0; j < 4; j++) {
    payload[j] = default_key[j & 3] ^ payload[j];
  }
  for (int j = 0; j < 12; j++) {
    payload[4 + j] = my_key[j & 3] ^ payload[4 + j];
  }
  return payloadLength;
}

uint8_t get_payload_with_inner_retry(int i, const uint8_t* data, int length, int i2, const uint8_t* key, int forward, uint8_t*& payload) {
  printf("data: "); dump(data, length); printf("\n");

  SEND_COUNT++;
  SEND_SEQ = SEND_COUNT;
  uint8_t safe_key = key[3];
  return package_ble_fastcon_body(i, i2, SEND_SEQ, safe_key, forward, data, length, key, payload);
}

void whiteningInit(uint8_t val, uint8_t* ctx)
{
  ctx[0] = 1;
  ctx[1] = (val >> 5) & 1;
  ctx[2] = (val >> 4) & 1;
  ctx[3] = (val >> 3) & 1;
  ctx[4] = (val >> 2) & 1;
  ctx[5] = (val >> 1) & 1;
  ctx[6] = val & 1;
}

void whiteningEncode(const uint8_t* data, int len, uint8_t* ctx, uint8_t* result)
{
  memcpy(result, data, len);
  for (int i = 0; i < len; i++) {
    int varC = ctx[3];
    int var14 = ctx[5];
    int var18 = ctx[6];
    int var10 = ctx[4];
    int var8 = var14 ^ ctx[2];
    int var4 = var10 ^ ctx[1];
    int _var = var18 ^ varC;
    int var0 = _var ^ ctx[0];

    int c = result[i];
    result[i] = ((c & 0x80) ^ ((var8 ^ var18) << 7))
      + ((c & 0x40) ^ (var0 << 6))
      + ((c & 0x20) ^ (var4 << 5))
      + ((c & 0x10) ^ (var8 << 4))
      + ((c & 0x08) ^ (_var << 3))
      + ((c & 0x04) ^ (var10 << 2))
      + ((c & 0x02) ^ (var14 << 1))
      + ((c & 0x01) ^ (var18 << 0));

    ctx[2] = var4;
    ctx[3] = var8;
    ctx[4] = var8 ^ varC;
    ctx[5] = var0 ^ var10;
    ctx[6] = var4 ^ var14;
    ctx[0] = var8 ^ var18;
    ctx[1] = var0;
  }
}

uint8_t reverse_8(uint8_t d)
{
  uint8_t result = 0;
  for (uint8_t k = 0; k < 8; k++) {
    result |= ((d >> k) & 1) << (7 - k);
  }
  return result;
}

uint16_t reverse_16(uint16_t d) {
  uint16_t result = 0;
  for (uint8_t k = 0; k < 16; k++) {
    result |= ((d >> k) & 1) << (15 - k);
  }
  return result;
}

uint16_t crc16(const uint8_t* addr, const uint8_t* data, uint8_t dataLength)
{
  uint16_t crc = 0xffff;

  for (int8_t i = addrLength - 1; i >= 0; i--) 
  {
    crc ^= addr[i] << 8;
    for (uint8_t ii = 0; ii < 4; ii++) {
      uint16_t tmp = crc << 1;

      if ((crc & 0x8000) !=0)
      {
        tmp ^= 0x1021;
      }

      crc = tmp << 1;
      if ((tmp & 0x8000) != 0)
      {
        crc ^= 0x1021;
      }
    }
  }

  for (uint8_t i = 0; i < dataLength; i++) {
    crc ^= reverse_8(data[i]) << 8;
    for (uint8_t ii = 0; ii < 4; ii++) {
      uint16_t tmp = crc << 1;

      if ((crc & 0x8000) != 0) 
      {
        tmp ^= 0x1021;
      }

      crc = tmp << 1;
      if ((tmp & 0x8000) != 0)
      {
        crc ^= 0x1021;
      }
    }
  }
  crc = ~reverse_16(crc) & 0xffff;
  return crc;
}


uint8_t get_rf_payload(const uint8_t* addr, const uint8_t* data, uint8_t dataLength, uint8_t*& rfPayload)
{

  uint8_t data_offset = 0x12;
  uint8_t inverse_offset = 0x0f;
  uint8_t result_data_size = data_offset + addrLength + dataLength+2;
  uint8_t* resultbuf = (uint8_t*)malloc(result_data_size);
  memset(resultbuf, 0, result_data_size);

  resultbuf[0x0f] = 0x71;
  resultbuf[0x10] = 0x0f;
  resultbuf[0x11] = 0x55;


  for (uint8_t j = 0; j < addrLength; j++) {
    resultbuf[data_offset + addrLength - j - 1] = addr[j];
  }

  for (int j = 0; j < dataLength; j++) {
    resultbuf[data_offset + addrLength + j] = data[j];
  }


  for (int i = inverse_offset; i < inverse_offset + addrLength + 3; i++) {
    resultbuf[i] = reverse_8(resultbuf[i]);
  }

  int crc = crc16(addr, data, dataLength);
  resultbuf[result_data_size-2] = crc & 0xff;
  resultbuf[result_data_size-1] = (crc >> 8) & 0xff;
  rfPayload = resultbuf;
  return result_data_size;
}

uint8_t do_generate_command(int i, const uint8_t* data, uint8_t length, const uint8_t* key, int forward, int use_default_adapter, int i2, uint8_t*& rfPayload)
{
  if (i2 < 0) i2 = 0;
  uint8_t* payload = 0;
  uint8_t* rfPayloadTmp = 0;
  uint8_t payloadLength = get_payload_with_inner_retry(i, data, length, i2, key, forward, payload);
  uint8_t rfPayloadLength = get_rf_payload(DEFAULT_BLE_FASTCON_ADDRESS, payload, payloadLength, rfPayloadTmp);
  free(payload);

  uint8_t ctx[7];
  whiteningInit(0x25, &ctx[0]);
  uint8_t* result = (uint8_t*)malloc(rfPayloadLength);

  whiteningEncode(rfPayloadTmp, rfPayloadLength, ctx, result);

  rfPayload = (uint8_t*)malloc(rfPayloadLength-15);
  memcpy(rfPayload, result + 15, rfPayloadLength - 15);
  free(result);
  free(rfPayloadTmp);
  return rfPayloadLength-15;
}

void single_control(const uint8_t* key, const uint8_t* result)
{
  uint8_t ble_adv_data[] = { 0x02, 0x01, 0x1A, 0x1B, 0xFF, 0xF0, 0xFF };
  uint8_t* rfPayload = 0;
  uint8_t rfPayloadLength = do_generate_command(5, result, 12, key, true /* forward ?*/, true /* use_default_adapter*/, 0, rfPayload);


  uint8_t* advPacket = (uint8_t*)malloc(rfPayloadLength + sizeof(ble_adv_data));
  memcpy(advPacket, ble_adv_data, sizeof(ble_adv_data));
  memcpy(advPacket+ sizeof(ble_adv_data), rfPayload, rfPayloadLength);
  free(rfPayload);

  for (int i=0; i<redundancy; i++) {
    send(advPacket, rfPayloadLength + sizeof(ble_adv_data));
    delay(50);
  }
  
  
  
  free(advPacket);
}

void send(uint8_t* data, uint8_t dataLength)
{

  
  BLEBeacon oBeacon = BLEBeacon();
  oBeacon.setManufacturerId(0xf0ff); // fake Apple 0x004C LSB (ENDIAN_CHANGE_U16!)
  oBeacon.setProximityUUID(BLEUUID(BEACON_UUID));
  oBeacon.setMajor(0);
  oBeacon.setMinor(0);
  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  BLEAdvertisementData oScanResponseData = BLEAdvertisementData();
  oAdvertisementData.setFlags(0x04); // BR_EDR_NOT_SUPPORTED 0x04
  std::string strServiceData = "";
  strServiceData += (char)(dataLength-4);     // Len  
  for (int i=4;i<dataLength;i++)
  {
    strServiceData += (char)data[i];
  }
  oAdvertisementData.addData(strServiceData);

  pAdvertising->setAdvertisementData(oAdvertisementData);
  //pAdvertising->setScanResponseData(oScanResponseData);
  pAdvertising->start();
  delay(50);
  pAdvertising->stop();
}

void onStateCommand(bool state, HALight* sender) {
  Serial.print("Light: '");
  Serial.print(sender->getName());
  Serial.print("' UniqueId: ");
  Serial.print(sender->uniqueId());
  Serial.print(" State: ");
  Serial.println(state);
  
  uint8_t data[] = { 0x22, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  data[2] = state ? 0x80 : 0x00;
  data[1] = atoi(sender->uniqueId());

  single_control(my_key, data);
  sender->setState(state); // report state back to the Home Assistant
}

void onBrightnessCommand(uint8_t brightness, HALight* sender) {
  Serial.print("Light: '");
  Serial.print(sender->getName());
  Serial.print("' UniqueId: ");
  Serial.print(sender->uniqueId());
  Serial.print(" Brightness: ");
  Serial.println(brightness);

  uint8_t data[] = { 0x22, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  data[2] = brightness & 127;
  data[1] = atoi(sender->uniqueId());

  single_control(my_key, data);
  sender->setBrightness(brightness); // report brightness back to the Home Assistant
}

void onColorTemperatureCommand(uint16_t temperature, HALight* sender) {
  Serial.print("Light: '");
  Serial.print(sender->getName());
  Serial.print("' UniqueId: ");
  Serial.print(sender->uniqueId());
  Serial.print(" Color temperature: ");
  Serial.println(temperature);

  sender->setColorTemperature(temperature); // report color temperature back to the Home Assistant
}

// payloads
//
// blue:  7201ffff0000000000000000
// red:   7201ff00ff00000000000000
// green: 7201ff0000ff000000000000
// white: 7201ff0000007f7f00000000


void onRGBColorCommand(HALight::RGBColor color, HALight* sender) {
  Serial.print("Light: ");
  Serial.print(sender->getName());
  Serial.print(" UniqueId: ");
  Serial.print(sender->uniqueId());
  Serial.print(" Red: ");
  Serial.print(color.red);
  Serial.print(" Green: ");
  Serial.print(color.green);
  Serial.print(" Blue: ");
  Serial.println(color.blue);

  uint8_t data[] = { 0x72, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  
  //ID: 
  data[1] = atoi(sender->uniqueId());
  data[2] = sender->getCurrentBrightness() & 127;
  
  // HALight is a RGB light only, simulating WHITE for RGBW
  if (color.blue>=250 && color.red>=250 && color.green>=250)  // switch to white
  {
    data[6] = 0x7f;
    data[7] = 0x7f;
  }
  else
  {
    data[3] = color.blue;
    data[4] = color.red;
    data[5] = color.green;
  }
  
  single_control(my_key, data);
  sender->setRGBColor(color); // report color back to the Home Assistant
}



void loop() 
{
  mqtt.loop();
}
