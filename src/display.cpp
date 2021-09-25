#include "display.h"

void printWeight (LiquidCrystal_PCF8574 &lcd, float value) {
    char s[16];
    sprintf(s, "% 05.1fkg", value);
    lcd.setCursor(0, 0);
    lcd.print(s);
}

void printTemp (LiquidCrystal_PCF8574 &lcd, float value) {
    char s[16];
    sprintf(s, "%+05.1f C", value);
    lcd.setCursor(0, 0);
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