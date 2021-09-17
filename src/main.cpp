
//------------------------------- Board Check ----------------------------------

#ifndef ARDUINO_ARCH_ESP32
#error "Select an ESP32 board"
#endif

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <SPI.h>
#include <core_version.h> // For ARDUINO_ESP32_RELEASE

#include <ArduinoUAVCAN.h>
#include <ACAN_ESP32.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static const uint32_t DESIRED_BIT_RATE = 1000UL * 1000UL; // 1 Mb/s
static const int LED_BUILTIN = 2;

static const gpio_num_t CTX_PIN = GPIO_NUM_4;
static const gpio_num_t CRX_PIN = GPIO_NUM_5;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

// void spi_select();
// void spi_deselect();
// uint8_t spi_transfer(uint8_t const);
// void onExternalEvent();
bool transmitCanFrame(CanardFrame const &);

void print_ESP_chip_info();
void print_ESP_CAN_info(ACAN_ESP32_Settings &settings);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

// TODO: delete
// ArduinoMCP2515 mcp2515(spi_select,
//                        spi_deselect,
//                        spi_transfer,
//                        micros,
//                        nullptr,
//                        nullptr);

ArduinoUAVCAN uc(13, transmitCanFrame);

Heartbeat_1_0 hb;

static uint32_t gBlinkLedDate = 0;
static uint32_t gReceivedFrameCount = 0;
static uint32_t gSentFrameCount = 0;

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  //--- Switch on builtin led
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  //--- Start serial
  Serial.begin(115200);
  while (!Serial)
  {
  }

  print_ESP_chip_info();

  //--- Configure ESP32 CAN
  Serial.println("Configure ESP32 CAN");
  ACAN_ESP32_Settings settings(DESIRED_BIT_RATE);
  settings.mRequestedCANMode = ACAN_ESP32_Settings::LoopBackMode;
  settings.mRxPin = CTX_PIN;
  settings.mTxPin = CRX_PIN;
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

  /* Configure initial heartbeat */
  hb.data.uptime = 0;
  hb = Heartbeat_1_0::Health::NOMINAL;
  hb = Heartbeat_1_0::Mode::INITIALIZATION;
  hb.data.vendor_specific_status_code = 0;
}

void loop()
{
  /* Update the heartbeat object */
  hb.data.uptime = millis() / 1000;
  hb = Heartbeat_1_0::Mode::OPERATIONAL;

  /* Publish the heartbeat once/second */
  static unsigned long prev = 0;
  unsigned long const now = millis();
  if (now - prev > 1000)
  {
    uc.publish(hb);
    prev = now;
  }

  /* Transmit all enqeued CAN frames */
  while (uc.transmitCanFrame())
  {
  }

  CANMessage frame;
  if (gBlinkLedDate < millis())
  {
    gBlinkLedDate += 500;
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
    const bool ok = ACAN_ESP32::can.tryToSend(frame);
    if (ok)
    {
      gSentFrameCount += 1;
    }
  }
  while (ACAN_ESP32::can.receive(frame))
  {
    gReceivedFrameCount += 1;
  }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void spi_select()
{
  // digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);
}

void spi_deselect()
{
  // digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
}

uint8_t spi_transfer(uint8_t const data)
{
  return SPI.transfer(data);
}

void onExternalEvent()
{
  // TODO: change for acan_esp32
  // mcp2515.onExternalEventHandler();
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
  const bool ok = ACAN_ESP32::can.tryToSend(frame2);
  if (ok)
  {
    gSentFrameCount += 1;
  }
  return ok;
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