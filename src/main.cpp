
//------------------------------- Board Check ----------------------------------

#ifndef ARDUINO_ARCH_ESP32
#error "Select an ESP32 board"
#endif

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>              //for ESP8266 use bug free i2c driver https://github.com/enjoyneering/ESP8266-I2C-Driver

#include <core_version.h> // For ARDUINO_ESP32_RELEASE

#include <ArduinoUAVCAN.h>
#include <ArduinoMCP2515.h>
#include <ACAN_ESP32.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include "HX711.h"
#include <LiquidCrystal_PCF8574.h>

#include "display.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;
using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static const CanardNodeID UC_ID = 16;
static const uint8_t UC_NAME[50] = "pl.simle.r5.tensocan";
static const uavcan_node_GetInfo_Response_1_0 GET_INFO_DATA = {
    /// uavcan.node.Version.1.0 protocol_version
    {1, 0},
    /// uavcan.node.Version.1.0 hardware_version
    {1, 0},
    /// uavcan.node.Version.1.0 software_version
    {0, 1},
    /// saturated uint64 software_vcs_revision_id
    NULL,
    /// saturated uint8[16] unique_id
    {0x86, 0xcc, 0xed, 0x61, 0x97, 0x1f, 0x4a, 0xf9, 
     0x9d, 0x19, 0x51, 0xc3, 0x9a, 0xb9, 0xa8, 0xd1},
    /// saturated uint8[<=50] name
    {
        "pl.simle.r5.tensocan",
        strlen("pl.simle.r5.tensocan")
    },
};

static const uint32_t DESIRED_BIT_RATE = 1000UL * 1000UL; // 1 Mb/s
static const int LED_BUILTIN = 2;

static const gpio_num_t CTX_PIN = GPIO_NUM_4;
static const gpio_num_t CRX_PIN = GPIO_NUM_5;

static const gpio_num_t LOADCELL_DOUT_PIN = GPIO_NUM_17;
static const gpio_num_t LOADCELL_SCK_PIN = GPIO_NUM_16;

static const gpio_num_t DHT_PIN = GPIO_NUM_15;

const float calibration_factor = -68100;

static CanardPortID const LOADCELL_PORT_ID   = 1337U;
static CanardPortID const DHT_T_PORT_ID   = 1338U;
static CanardPortID const DHT_H_PORT_ID   = 1339U;

static CanardPortID const TEMP_PORT_ID   = 2137U;

#define COLUMS 20
#define ROWS   4

#define PAGE   ((COLUMS) * (ROWS))

#define DHTTYPE    DHT11

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

bool transmitCanFrame(CanardFrame const &);
void onReceiveCanFrame(CANMessage const &);
void onGetInfo_1_0_Request_Received(CanardTransfer const &, ArduinoUAVCAN &);
void onTemperature_1_0_Received(CanardTransfer const &, ArduinoUAVCAN &);
void get_scale();
void get_dht();

void print_ESP_chip_info();
void print_ESP_CAN_info(ACAN_ESP32_Settings &settings);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoUAVCAN uc(UC_ID, transmitCanFrame);

Heartbeat_1_0<> hb;
Real32_1_0<LOADCELL_PORT_ID> scale_measurment;
Real32_1_0<DHT_T_PORT_ID> dht_t_measurment;
Real32_1_0<DHT_H_PORT_ID> dht_h_measurment;

// external
Real32_1_0<TEMP_PORT_ID> temperature_measurment;

HX711 scale;
DHT_Unified dht(DHT_PIN, DHTTYPE);

LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display

static uint32_t gBlinkLedDate = 0;
static uint32_t gReceivedFrameCount = 0;
static uint32_t gSentFrameCount = 0;
static uint32_t temperature_last_received = 0;

static bool dht_err = false;

int show = -1;

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  int error;
  //--- Switch on builtin led
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  //--- Start serial
  Serial.begin(115200);
  while (!Serial)
  {
  }

  Wire.begin();
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();
  Serial.print("Error: ");
  Serial.print(error);

  if (error == 0) {
    Serial.println(": LCD found.");
    show = 0;
    lcd.begin(16, 2); // initialize the lcd

  } else {
    Serial.println(": LCD not found.");
  } // if

  dht.begin();

  print_ESP_chip_info();

  //--- Configure ESP32 CAN
  Serial.println("Configure ESP32 CAN");
  ACAN_ESP32_Settings settings(DESIRED_BIT_RATE);
  // settings.mRequestedCANMode = ACAN_ESP32_Settings::LoopBackMode;
  settings.mRxPin = CRX_PIN;
  settings.mTxPin = CTX_PIN;
  const uint32_t errorCode = ACAN_ESP32::can.begin(settings);

  if (errorCode == 0)
  {
    print_ESP_CAN_info(settings);
  }
  else
  {
    Serial.print("Configuration error 0x");
    Serial.println(errorCode, HEX);
  }

  /* Configure scale */
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare();

  /* Configure initial heartbeat */
  hb.data.uptime = 0;
  hb = Heartbeat_1_0<>::Health::NOMINAL;
  hb = Heartbeat_1_0<>::Mode::INITIALIZATION;
  hb.data.vendor_specific_status_code = 0;

  uc.subscribe<GetInfo_1_0::Request<>>(onGetInfo_1_0_Request_Received);
  uc.subscribe<Real32_1_0<TEMP_PORT_ID>>(onTemperature_1_0_Received);
}

void loop()
{
  /* Update the heartbeat object */
  hb.data.uptime = millis() / 1000;
  hb = Heartbeat_1_0<>::Mode::OPERATIONAL;

  /* Publish the heartbeat once/second */
  static unsigned long prev = 0;
  unsigned long const now = millis();
  if (now - prev > 1000)
  {
    get_scale();
    get_dht();
    uc.publish(hb);
    uc.publish(scale_measurment);
    prev = now;

    lcd.clear();
    print_can_stats(lcd);
    print_weight(lcd, scale_measurment.data.value);
    print_temp(lcd, temperature_measurment.data.value, 
      now - temperature_last_received > 3000);
    print_dht(lcd, dht_t_measurment.data.value, dht_h_measurment.data.value, dht_err);

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    Serial.print("Sent: ");
    Serial.print(gSentFrameCount);
    Serial.print("\t");
    Serial.print("Receive: ");
    Serial.print(gReceivedFrameCount);
    Serial.print("\t");
    Serial.print(" STATUS 0x");
    Serial.print(CAN_STATUS, HEX);
    Serial.print(" RXERR ");
    Serial.print(CAN_RX_ECR);
    Serial.print(" TXERR ");
    Serial.println(CAN_TX_ECR);
  }

  CANMessage frame;
  /* Transmit all enqeued CAN frames */
  while (uc.transmitCanFrame())
  {
  }
  while (ACAN_ESP32::can.receive(frame))
  {
    onReceiveCanFrame(frame);
    gReceivedFrameCount += 1;
  }


}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void get_dht() {
  sensors_event_t event;
  dht_err = false;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
    dht_err = true;
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    dht_t_measurment.data.value = event.temperature;
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
    dht_err = true;
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    dht_h_measurment.data.value = event.relative_humidity;
  }
}

void get_scale() {
  scale_measurment.data.value = scale.get_units(10);
}

bool transmitCanFrame(CanardFrame const &frame)
{
  CANMessage frame2;
  frame2.id = frame.extended_can_id;
  frame2.ext = true;

  // TODO: this can be fucking better
  const void *p = frame.payload;
  for (size_t i = 0; i < frame.payload_size; i++)
  {
    frame2.data[i] = *(uint8_t *) p;
    p++;
  }
  
  frame2.len = static_cast<uint8_t const>(frame.payload_size);

  // Serial.print("TX ");
  // Serial.println(frame2.id);
  const bool ok = ACAN_ESP32::can.tryToSend(frame2);
  if (ok)
  {
    gSentFrameCount += 1;
  }
  return ok;
}

void onReceiveCanFrame(CANMessage const &frame)
{
  CanardFrame const f
    {
      1,                        /* timestamp_usec  */
      frame.id & MCP2515::CAN_ADR_BITMASK,       /* extended_can_id limited to 29 bit */
      static_cast<uint8_t const>(frame.len),     /* payload_size    */
      reinterpret_cast<const void *>(frame.data) /* payload         */
    };
  Serial.print("RX ");
  Serial.println(frame.id);
  uc.onCanFrameReceived(f);
}

void onGetInfo_1_0_Request_Received(CanardTransfer const & transfer, ArduinoUAVCAN & uc)
{
  GetInfo_1_0::Response<> rsp = GetInfo_1_0::Response<>();
  rsp.data = GET_INFO_DATA;
  Serial.println("onGetInfo_1_0_Request_Received");
  uc.respond(rsp, transfer.remote_node_id, transfer.transfer_id);
}

void onTemperature_1_0_Received(CanardTransfer const & transfer, ArduinoUAVCAN & uc) {
  Real32_1_0<TEMP_PORT_ID> const d = Real32_1_0<TEMP_PORT_ID>::deserialize(transfer);
  temperature_measurment = d;
  temperature_last_received = millis();
}


void print_ESP_chip_info()
{
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  Serial.print("ESP32 Arduino Release: ");
  Serial.println(ARDUINO_ESP32_RELEASE);
  Serial.print("ESP32 Chip Revision: ");
  Serial.println(chip_info.revision);
  Serial.print("ESP32 SDK: ");
  Serial.println(ESP.getSdkVersion());
  Serial.print("ESP32 Flash: ");
  Serial.print(spi_flash_get_chip_size() / (1024 * 1024));
  Serial.print(" MB ");
  Serial.println(((chip_info.features & CHIP_FEATURE_EMB_FLASH) != 0) ? "(embeded)" : "(external)");
  Serial.print("APB CLOCK: ");
  Serial.print(APB_CLK_FREQ);
  Serial.println(" Hz");
}

void print_ESP_CAN_info(ACAN_ESP32_Settings &settings)
{
  Serial.print("Bit Rate prescaler: ");
  Serial.println(settings.mBitRatePrescaler);
  Serial.print("Time Segment 1:     ");
  Serial.println(settings.mTimeSegment1);
  Serial.print("Time Segment 2:     ");
  Serial.println(settings.mTimeSegment2);
  Serial.print("RJW:                ");
  Serial.println(settings.mRJW);
  Serial.print("Triple Sampling:    ");
  Serial.println(settings.mTripleSampling ? "yes" : "no");
  Serial.print("Actual bit rate:    ");
  Serial.print(settings.actualBitRate());
  Serial.println(" bit/s");
  Serial.print("Exact bit rate ?    ");
  Serial.println(settings.exactBitRate() ? "yes" : "no");
  Serial.print("Distance            ");
  Serial.print(settings.ppmFromDesiredBitRate());
  Serial.println(" ppm");
  Serial.print("Sample point:       ");
  Serial.print(settings.samplePointFromBitStart());
  Serial.println("%");
  Serial.println("Configuration OK!");
}