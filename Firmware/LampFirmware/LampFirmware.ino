#define REV_A01

#include "pinouts.h"
#include "Display.h"

enum powerSource{Wall = DISPLAY_BITS_AC_PWR, Solar = DISPLAY_BITS_SOLAR_PWR, Battery = DISPLAY_BITS_BATTERY_PWR};

void initDisplay(){
  pinMode(RCLK_DISPLAY, OUTPUT);
  pinMode(OE_DISPLAY, OUTPUT);
  pinMode(SRCLK_DISPLAY, OUTPUT);
  pinMode(DATA_DISPLAY, OUTPUT);
  analogWrite(OE_DISPLAY, 200);
}

void initMux(){
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_ADC, INPUT);
}

void BadWallPowerISR(){
  digitalWrite(RELAY, LOW);
}

void initPowerModule(){
  pinMode(RELAY, OUTPUT);
  pinMode(PWM1, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(WallBadINT), BadWallPowerISR, FALLING);
  pinMode(WallSense, INPUT);
}

void DisplayUpdate(uint8_t hours, uint8_t minutes, uint8_t batteryPercentage, powerSource PowerSourceToDisplay, uint8_t systemWarning, uint8_t systemFailure){
  digitalWrite(RCLK_DISPLAY, HIGH);
  shiftOut(DATA_DISPLAY, SRCLK_DISPLAY, LSBFIRST, ~ (0xFF >> ((uint8_t) ((float)batteryPercentage / 12.5))));
  shiftOut(DATA_DISPLAY, SRCLK_DISPLAY, LSBFIRST, (DISPLAY_BITS_FAILURE * systemFailure) | (DISPLAY_BITS_WARNING * systemWarning) | PowerSourceToDisplay | DISPLAY_BITS_COLON);
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
  shiftOut(DATA_DAC, SRCLK_DAC, LSBFIRST, vref);
  digitalWrite(RCLK_DAC, HIGH);
}

void setup() {
  Serial.begin(9600);
  initDisplay();
  initMux();
  initPowerModule();
  digitalWrite(RELAY, HIGH);
<<<<<<< HEAD
  digitalWrite(OE_DISPLAY, HIGH);
}

void loop() {
  Serial.print("WallVoltage: ");
  Serial.print(analogRead(WallSense));
  Serial.print(", Brightness command: ");
  Serial.print(readMux(BRIGHTNESS_POT_CHANNEL));
  Serial.println();
  analogWrite(PWM1,readMux(BRIGHTNESS_POT_CHANNEL)/4);
//  DisplayUpdate(2, 12, 50, Wall, 1, 1);
//
  if((float) analogRead(WallSense) * 43/10 * 3.3 / 1024 > 10){
    digitalWrite(RELAY, HIGH);
  }
//  digitalWrite(OE_DISPLAY, HIGH);
//digitalWrite(RCLK_DISPLAY, HIGH);
//  shiftOut(DATA_DISPLAY, SRCLK_DISPLAY, LSBFIRST, 0);
//  digitalWrite(RCLK_DISPLAY, LOW);
//delay(500);
  digitalWrite(RCLK_DISPLAY, HIGH);
  shiftOut(DATA_DISPLAY, SRCLK_DISPLAY, LSBFIRST, 0xFFFFFF);
  digitalWrite(RCLK_DISPLAY, LOW);
//  digitalWrite(OE_DISPLAY, LOW);
// digitalWrite(OE_DISPLAY, !digitalRead(OE_DISPLAY));
  delay(100);
=======
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);
}

void loop() {
  DisplayUpdate(04,20, 69,Battery, 1,1);
  Serial.print("WallVoltage: ");
  Serial.print(analogRead(WallSense));
  Serial.print(", Brightness command: ");
  Serial.print(readMux(BRIGHTNESS_POT_CHANNEL));
  Serial.println();
  analogWrite(PWM1, 255 - readMux(BRIGHTNESS_POT_CHANNEL)/4);

  

  if((float) analogRead(WallSense) * 43/10 * 3.3 / 1024 > 10){
    digitalWrite(RELAY, HIGH);
  }
>>>>>>> 138b189ee9fb78b51cb962b9ed9f4aa69279b2a3
}
