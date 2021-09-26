#include <LiquidCrystal_PCF8574.h>
#include <ACAN_ESP32.h>

void print_weight (LiquidCrystal_PCF8574 &lcd, float value);
void print_temp (LiquidCrystal_PCF8574 &lcd, float value, bool err);
void print_dht (LiquidCrystal_PCF8574 &lcd, float t, float h, bool err);
void print_can_stats (LiquidCrystal_PCF8574 &lcd);