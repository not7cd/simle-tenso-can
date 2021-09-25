#include <LiquidCrystal_PCF8574.h>
#include <ACAN_ESP32.h>

void printWeight (LiquidCrystal_PCF8574 &lcd, float value);
void printTemp (LiquidCrystal_PCF8574 &lcd, float value);
void print_can_stats (LiquidCrystal_PCF8574 &lcd);