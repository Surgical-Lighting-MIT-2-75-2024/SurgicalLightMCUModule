#define REV_B01

#include "pinouts.h"
#include "Display.h"

enum powerSource{Wall = DISPLAY_BITS_AC_PWR, Solar = DISPLAY_BITS_SOLAR_PWR, Battery = DISPLAY_BITS_BATTERY_PWR};
const uint16_t UPDATE_DELAY = 1000;
const uint8_t TL_NOISE_THRESHOLD = .2;

// GLOBAL VARS
bool isACMains = false;
bool isSolar = false;
bool isBattery = false;
bool isSysWarning = false;
float wallVoltage = 0;
float solarVoltage = 0;
float batteryVoltage = 0;
float timeLeft = 0;
uint16_t brightnessCommand = 0;
unsigned long time_now = millis();

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
  shiftOut(DATA_DISPLAY, SRCLK_DISPLAY, LSBFIRST, ~ (0xFF >> ((uint8_t) ((float)batteryPercentage / 12.5)) + 1));
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

void setBuckSetpoint(float vout, uint8_t iout){
  uint8_t vref_bin = vout * 10 ;
  digitalWrite(RCLK_DAC, LOW);
  shiftOut(DATA_DAC, SRCLK_DAC, LSBFIRST, 255); // current ref doesn't work. 
  shiftOut(DATA_DAC, SRCLK_DAC, LSBFIRST, vref_bin);
  digitalWrite(RCLK_DAC, HIGH);
}

void initBuck(){
  pinMode(RCLK_DAC, OUTPUT);
  pinMode(DATA_DAC, OUTPUT);
  pinMode(SRCLK_DAC, OUTPUT);
}

// Brightness command averaging function
const int bc_numReadings = 500;     // Number of readings to average
int bc_readings[bc_numReadings];      // Array to store the readings
int bc_readingsIndx = 0;                  // Index of the current reading
int bc_total = 0;                  // Running total of all readings
int brightnessAverage = 0;                // Smoothed value
uint8_t bc_noiseThreshold = 5;
void bc_averagingFilter() {
  // Read the input value
  int sensorValue = readMux(BRIGHTNESS_POT_CHANNEL)/4;
  // Filter out small noise
  if (abs(sensorValue - brightnessAverage) < bc_noiseThreshold) {
    sensorValue = brightnessAverage;  // Replace noisy value with the current average
  }
  // Subtract the oldest reading from the total
  bc_total -= bc_readings[bc_readingsIndx];
  // Store the new reading in the array
  bc_readings[bc_readingsIndx] = sensorValue;
  // Add the new reading to the total
  bc_total += sensorValue;
  // Move to the next index
  bc_readingsIndx = (bc_readingsIndx + 1) % bc_numReadings;
  
  // Calculate the average
  brightnessAverage = bc_total / bc_numReadings;
}

// Wall voltage averaging function
const int wv_numReadings = 100;     // Number of readings to average
float wv_readings[wv_numReadings];      // Array to store the readings
int wv_readingsIndx = 0;                  // Index of the current reading
float wv_total = 0;                  // Running total of all readings
uint8_t wv_count = 1;
void wv_averagingFilter(float wvInput) {
  // Subtract the oldest reading from the total
  wv_total -= wv_readings[wv_readingsIndx];
  // Store the new reading in the array
  wv_readings[wv_readingsIndx] = wvInput;
  // Add the new reading to the total
  wv_total += wvInput;
  // Move to the next index
  wv_readingsIndx = (wv_readingsIndx + 1) % wv_numReadings;
  
  // Calculate the average
  if (wv_count < wv_numReadings) {
    wallVoltage = wv_total / wv_count;
    wv_count++;
  } else {
    wallVoltage = wv_total / wv_numReadings;
  }
}

// Battery voltage averaging function
const int bv_numReadings = 100;     // Number of readings to average
float bv_readings[bv_numReadings];      // Array to store the readings
int bv_readingsIndx = 0;                  // Index of the current reading
float bv_total = 0;                  // Running total of all readings
uint8_t bv_count = 1;
void bv_averagingFilter(float bvInput) {
  // Subtract the oldest reading from the total
  bv_total -= bv_readings[bv_readingsIndx];
  // Store the new reading in the array
  bv_readings[bv_readingsIndx] = bvInput;
  // Add the new reading to the total
  bv_total += bvInput;
  // Move to the next index
  bv_readingsIndx = (bv_readingsIndx + 1) % bv_numReadings;
  
  // Calculate the average
  if (bv_count < bv_numReadings) {
    batteryVoltage = bv_total / bv_count;
    bv_count++;
  } else {
    batteryVoltage = bv_total / bv_numReadings;
  }
}

// Calculates battery capacity
float maxPower = 86.4; // W
float loadEfficiency = .8;
float capacityEfficiency = .7;
float batteryCapacity = 80 * capacityEfficiency; // Ah
float maxBatteryVoltage = 14.4; // V
float minBatteryVoltage = 10.0; // V
float batteryCapacityCalculation() {
  float dutyCycle = brightnessCommand ? (brightnessCommand / 255.0) : 1;
  float loadPower = (maxPower * dutyCycle) / loadEfficiency; // W
  float batteryCurrent = loadPower / batteryVoltage; // A
  float timeLeft = (batteryCapacity / batteryCurrent) * ((batteryVoltage - minBatteryVoltage)/(maxBatteryVoltage - minBatteryVoltage));
  return timeLeft;
}


void setup() {
  Serial.begin(9600);
  initDisplay();
  initMux();
  initPowerModule();
  digitalWrite(RELAY, HIGH);
  digitalWrite(OE_DISPLAY, HIGH);
  initBuck();
}

void loop() {
  // Calculates the brightness filter.
  setBuckSetpoint(13.5, 1);
  bc_averagingFilter();
  brightnessCommand = 255 - (brightnessAverage > 250 ? 255: brightnessAverage);
  analogWrite(PWM1, brightnessCommand);
  if (millis() - time_now > UPDATE_DELAY) {
    // Calculates the wall voltage from the WallSense line.
    // If it's greater than 10V, make the relay high and turn on the AC Mains indicator
    wv_averagingFilter(analogRead(WallSense) * 43/10 * 3.3 / 1024);
    if(wallVoltage > 10){
      digitalWrite(RELAY, HIGH);
      isACMains = true;
    } else {
      isACMains = false;
    }
  
    // Calculates the solar/mppt voltage from the SolarSense line.
    isSolar = false;
//    solarVoltage = readMux(SOLAR_SENSE_CHANNEL) * 11 * 3.3 / 1024;
//    if (solarVoltage > 10) {
//      isSolar = true;
//    } else {
//      isSolar = false;
//    }

    // Calculates the battery voltage from the BatterySense line.
    bv_averagingFilter(analogRead(BatterySense) * 43/10 * 3.3 / 1024);
    isBattery = batteryVoltage > 10;
    float newTimeLeft = batteryCapacityCalculation();
    timeLeft = (abs(timeLeft - newTimeLeft) > TL_NOISE_THRESHOLD) ? newTimeLeft : timeLeft;
    float batteryPercentage = (batteryVoltage - minBatteryVoltage) / (maxBatteryVoltage - minBatteryVoltage);
    Serial.print("Battery Voltage: ");
    Serial.println(batteryVoltage);
    Serial.print("Wall Voltage: ");
    Serial.println(wallVoltage);
    Serial.println(analogRead(WallSense));
    Serial.print("Time Left: ");
    Serial.println(timeLeft);
    Serial.print("Battery Percentage: ");
    Serial.println(batteryPercentage);
    Serial.print("Brightness Command: ");
    Serial.println(brightnessCommand);
    Serial.print("Duty Cycle: ");
    Serial.println(brightnessCommand / 255.0);
    Serial.print("Battery Current: ");
    Serial.println((maxPower * loadEfficiency * (brightnessCommand / 255.0)) / wallVoltage);
    
    // Updating display
    DisplayUpdate(isBattery ? max(min(floor(timeLeft), 99), 0) : 0, isBattery ? max(min((timeLeft - floor(timeLeft)) * 60, 59), 0) : 0, isBattery ? batteryPercentage * 100 : 0, isACMains ? Wall : (isSolar ? Solar : Battery), isSysWarning, 0);
    time_now = millis();
  }
}
