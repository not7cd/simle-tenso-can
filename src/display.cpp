#include "display.h"

void print_weight (LiquidCrystal_PCF8574 &lcd, float value) {
    char s[16];
    sprintf(s, "% 05.1fkg", value);
    lcd.setCursor(0, 0);
    lcd.print(s);
}

void print_temp (LiquidCrystal_PCF8574 &lcd, float value, bool err) {
    char s[16];
    if (err)
    {
        sprintf(s, "T#####");
    }
    else {
        sprintf(s, "T%+05.1f", value);
    }
    
    lcd.setCursor(10, 1);
    lcd.print(s);
}

void print_dht (LiquidCrystal_PCF8574 &lcd, float t, float h, bool err) {
    char s[6];
    if (err)
    {
        sprintf(s, "DHTERR");
    } else if (millis() / 2000 % 2)
    {
        sprintf(s, "H% 3.0f\% ", h);
    }
    else {
        sprintf(s, "T%+05.1f", t);
    }
    
    lcd.setCursor(10, 0);
    lcd.print(s);
}

#define CHECK_BIT(var,pos) ((var & pos) == pos)
// TODO: bit check
void print_can_stats (LiquidCrystal_PCF8574 &lcd) {
    static const char t[] = "CAN %s";
    char s[16];
    char msg[16];
    if (CHECK_BIT(CAN_STATUS, CAN_STATUS_ERR))
    {
        if (CHECK_BIT(CAN_STATUS, CAN_STATUS_TX))
        {
            sprintf(s, t, "TXERR");
        }
        else if (CHECK_BIT(CAN_STATUS, CAN_STATUS_RX))
        {
            sprintf(s, t, "RXERR");
        }
        else {
            sprintf(s, t, "ERR!");
        }
    } else {
        sprintf(s, t, "OK");
    }
    
    lcd.setCursor(0, 1);
    lcd.print(s);
}