#define REV_A01

#include "pinouts.h"
#include "Display.h"

enum powerSource{Wall = DISPLAY_BITS_AC_PWR, Solar = DISPLAY_BITS_SOLAR_PWR, Battery = DISPLAY_BITS_BATTERY_PWR};
uint8_t UPDATE_COUNT = 5000;

// GLOBAL VARS
bool isACMains = false;
bool isSolar = false;
bool isSysWarning = false;
float wallVoltage = 0;
float solarVoltage = 0;
float batteryVoltage = 0;
uint16_t updateCount = UPDATE_COUNT;
uint16_t brightnessCommand = 0;

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
  wallVoltage = wv_total / wv_numReadings;
}

// Battery voltage averaging function
const int bv_numReadings = 100;     // Number of readings to average
float bv_readings[bv_numReadings];      // Array to store the readings
int bv_readingsIndx = 0;                  // Index of the current reading
float bv_total = 0;                  // Running total of all readings
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
  batteryVoltage = bv_total / bv_numReadings;
}

// Calculates battery capacity
float maxPower = 86.4;
float loadEfficiency = .8;
float capacityEfficiency = .7;
float batteryCapacity = 80 * capacityEfficiency;
float maxBatteryVoltage = 14.4;
float minBatteryVoltage = 10.0;
float batteryCapacityCalculation() {
  float dutyCycle = brightnessCommand / 255.0;
  float loadPower = maxPower * loadEfficiency * dutyCycle;
  float batteryCurrent = loadPower / wallVoltage;
  float timeLeft = (batteryCapacity / batteryCurrent) * (1 - ((wallVoltage - maxBatteryVoltage)/(maxBatteryVoltage - minBatteryVoltage)));
  return timeLeft;
}


void setup() {
  Serial.begin(9600);
  initDisplay();
  initMux();
  initPowerModule();
  digitalWrite(RELAY, HIGH);
  digitalWrite(OE_DISPLAY, HIGH);
}

void loop() {
//  // Brightness
//  Serial.print("Battery Voltage: ");
//  Serial.print(batteryVoltage);
//  Serial.print(", Brightness command: ");
//  Serial.println(brightnessAverage);
//  Serial.println(readMux(BRIGHTNESS_POT_CHANNEL));
//  Serial.println();
  bc_averagingFilter();
  brightnessCommand = 255 - (brightnessAverage > 250 ? 255: brightnessAverage);
  analogWrite(PWM1, brightnessCommand);

  if (updateCount == 0) {
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
    float timeLeft = batteryCapacityCalculation();
    float batteryPercentage = (wallVoltage - minBatteryVoltage) / (maxBatteryVoltage - minBatteryVoltage);
    Serial.print("Capacity: ");
    Serial.println(wallVoltage);
    Serial.println(timeLeft);
    Serial.println(floor(timeLeft));
    Serial.println((timeLeft - floor(timeLeft)) * 60);
    Serial.println(batteryPercentage);
    
  
    // Updating display
    DisplayUpdate(floor(timeLeft), (timeLeft - floor(timeLeft)) * 60, batteryPercentage * 100, isACMains ? Wall : (isSolar ? Solar : Battery), isSysWarning, 0);
    updateCount = UPDATE_COUNT;
  }
  updateCount = updateCount - 1;
}
