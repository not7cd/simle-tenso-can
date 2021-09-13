/**
 *
 * HX711 library for Arduino - example file
 * https://github.com/bogde/HX711
 *
 * MIT License
 * (c) 2018 Bogdan Necula
 *
**/
#include "SPI.h"
#include "HX711.h"

// Warning light
const int LIGHT_PIN = 9;

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 4;
const int LOADCELL_SCK_PIN = 5;

const long callibration_factor = -68100;
HX711 scale;

void print_scale(const HX711 &scale) {
  Serial.print("read: \t\t");
  Serial.println(scale.read());			// print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));  	// print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));		// print the average of 5 readings from the ADC minus the tare weight (not set yet)

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);	// print the average of 5 readings from the ADC minus tare weight (not set) divided
						// by the SCALE parameter (not set yet)
}

void setup() {
  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, HIGH);

  Serial.begin(9600);

  Serial.println("Initializing the scale");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  Serial.println("Before setting up the scale:");
  print_scale(scale);

  scale.set_scale(callibration_factor);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare();				        // reset the scale to 0
  long zero_factor = scale.read_average();
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);

  Serial.println("After setting up the scale:");
  print_scale(scale);

  Serial.println("Readings:");
}

void loop() {
  Serial.print("one reading:\t");
  Serial.print(scale.get_units(), 3);
  Serial.print("\t| average:\t");
  Serial.println(scale.get_units(10), 3);

  scale.power_down();			        // put the ADC in sleep mode
  delay(1000);

  scale.power_up();
  digitalWrite(LIGHT_PIN, HIGH);
  delay(500);
  digitalWrite(LIGHT_PIN, LOW);
}
