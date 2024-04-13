#define REV_A01

#include "pinouts.h"
#include "Display.h"

enum powerSource{Wall = DISPLAY_BITS_AC_PWR, Solar = DISPLAY_BITS_SOLAR_PWR, Battery = DISPLAY_BITS_BATTERY_PWR};

void initDisplay(){
  pinMode(RCLK_DISPLAY, OUTPUT);
  pinMode(OE_DISPLAY, OUTPUT);
  pinMode(SRCLK_DISPLAY, OUTPUT);
  pinMode(DATA_DISPLAY, OUTPUT);
}

void initMux(){
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_ADC, INPUT);
}

void DisplayUpdate(uint8_t hours, uint8_t minutes, uint8_t batteryPercentage, powerSource PowerSourceToDisplay, uint8_t systemWarning, uint8_t systemFailure){
  digitalWrite(RCLK_DISPLAY, HIGH);
  shiftOut(DATA_DISPLAY, SRCLK_DISPLAY, LSBFIRST, 0xF >> ((uint8_t) batteryPercentage / 10));
  shiftOut(DATA_DISPLAY, SRCLK_DISPLAY, LSBFIRST, (DISPLAY_BITS_FAILURE * systemFailure) | (DISPLAY_BITS_WARNING * systemWarning) | PowerSourceToDisplay);
  shiftOut(DATA_DISPLAY, SRCLK_DISPLAY, LSBFIRST, DISPLAY_BITS[minutes % 10]);
  shiftOut(DATA_DISPLAY, SRCLK_DISPLAY, LSBFIRST, DISPLAY_BITS[(uint8_t) minutes / 10]);
  shiftOut(DATA_DISPLAY, SRCLK_DISPLAY, LSBFIRST, DISPLAY_BITS[hours % 10]);
  shiftOut(DATA_DISPLAY, SRCLK_DISPLAY, LSBFIRST, DISPLAY_BITS[(uint8_t) hours / 10]);
  digitalWrite(RCLK_DISPLAY, LOW);
}

uint16_t readMux(uint8_t channel){
  digitalWrite(MUX_S0, channel & 1);
  digitalWrite(MUX_S1, channel & 1 << 1);
  digitalWrite(MUX_S2, channel & 1 << 2);

  return analogRead(MUX_ADC);
}

void setBuckSetpoint(uint8_t vref, uint8_t iref){
  digitalWrite(RCLK_DAC, LOW);
  shiftOut(DATA_DAC, SRCLK_DAC, LSBFIRST, iref);
  shiftOut(DATA_DAC, SRCLK_DISPLAY, LSBFIRST, vref);
  digitalWrite(RCLK_DAC, HIGH);
}

void setup() {
  Serial.begin(9600);
  initDisplay();
  initMux();
}

void loop() {
  Serial.print("Brightness command: ");
  Serial.print(readMux(BRIGHTNESS_POT_CHANNEL));
  Serial.println();

}
