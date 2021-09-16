#include <max6675.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>

// Sensor construction
const int thermo_SO = 5, thermo_CS = 4, thermo_SCK = 3;
MAX6675 thermo(thermo_SCK, thermo_CS, thermo_SO);
const unsigned int lcd_RS = 6, lcd_E = 7, lcd_D4 = 8, lcd_D5 = 9, lcd_D6 = 10, lcd_D7 = 11;
LiquidCrystal lcd(lcd_RS, lcd_E, lcd_D4, lcd_D5, lcd_D6, lcd_D7);

// Reading Thermocouple, MAX6675 requires a delay in reading
float previousTemp;
const unsigned long refreshTempRate = 500;
unsigned long lastRefreshTemp = 0;

// Temperature Set Point
unsigned int tempSetPointRange[2] = {1024, 0};
const byte tempSetPointPotPin = A0;
bool tempSetPointPotConfigured = false;

// LCD Refresh Rate
const unsigned long refreshLCDRate = 200;
unsigned long lastRefreshLCD = 0;
const unsigned long displayCycleRate = 2000;
unsigned long lastDisplayCycle = 0;
int currentDisplayID = 0; // Cycle through the display types

// Timer Start Threshold
float thresholdTemp = 120;
unsigned long timerThreshold = 0;
bool thresholdMet = false;

// Heat Relay
const unsigned int heatRelayPin = 12;
const unsigned long heatingInterval = 330 ;
unsigned long previousHeatingStart = 0;

// PID control
double pid_input;
double pid_output;
double pid_setPoint;
PID tempController(&pid_input, &pid_output, &pid_setPoint, 1.0, 0.1, 0, P_ON_M, DIRECT);

// Plot Temp
const unsigned long refreshPlot = 0;
unsigned long lastRefreshPlot = 0;

// Forward Declarations of Helper Functions
float getCurrentTemp();
float getTempSetPoint();
int calculateRequiredHeat(float setTemp, float currentTemp);
void setHeatOutput(int heatPercent);
void writeLCDdata(float tempSetPoint, int displayID, int heatPercent);
String getTime();

void setup(){
  // Initialize Display methods
  Serial.begin(9600);
  lcd.begin(16,2);

  // Initialize PID controller
  tempController.SetMode(AUTOMATIC);
  tempController.SetOutputLimits(0,100);

  // Initialize relay control
  pinMode(heatRelayPin, OUTPUT);
  digitalWrite(heatRelayPin, LOW);
  
  // Allow circuit to stabilize [Thermocouple]
  delay(500);
}

void loop(){
  // Get Current Time
  unsigned long currentTime = millis();

  // Get Current Temp and Set Point
  float tempSetPoint = getTempSetPoint();
  float currentTemp = getCurrentTemp();

  // Calculate required heat
  int heatPercent = calculateRequiredHeat(tempSetPoint, currentTemp);
  setHeatOutput( heatPercent );

  // Write LCD
  if (currentTime - lastRefreshLCD >= refreshLCDRate){
    writeLCDdata(tempSetPoint, currentDisplayID, heatPercent);
    lastRefreshLCD = currentTime;
    // Check if ready to cycle to next display item
    if (currentTime - lastDisplayCycle >= displayCycleRate){
      currentDisplayID = (currentDisplayID + 1) % 2;
      lastDisplayCycle = currentTime;
    }
  }

//  delay(50);
}

//======================================================================================================
// Helper Functions
//======================================================================================================
float getCurrentTemp(){
  unsigned long currentTime = millis();
  if (currentTime - lastRefreshTemp >= refreshTempRate){
    float tempValue = thermo.readFahrenheit();
    // Guard against nan values in temp
    if (isnan(tempValue)){
      return previousTemp;
    }
    previousTemp = tempValue;
    lastRefreshTemp = currentTime;
  }
  return previousTemp;
}

float getTempSetPoint(){
  unsigned int tempSet = analogRead(tempSetPointPotPin);

  // Measure range of potentiometer range
  if (tempSet < tempSetPointRange[0]){
    tempSetPointRange[0] = tempSet;
  }
  if (tempSet > tempSetPointRange[1]){
    tempSetPointRange[1] = tempSet;
  }

  // If not configured, make set point the current temp
  if (tempSetPointRange[1] - tempSetPointRange[0] > 500 && !tempSetPointPotConfigured){
    tempSetPointPotConfigured = true;
  }
  if (!tempSetPointPotConfigured){
    return getCurrentTemp();
  }

  // Once calibrated, calculate set point based on percentage of potentiometer
  float potPercentage = (float)(tempSet - tempSetPointRange[0]) / (float)(tempSetPointRange[1] - tempSetPointRange[0]);
  float tempSetPoint = potPercentage * (400.0 - 50.0) + 50.0;

  // Round Set point to nearest 5
  float rounded = tempSetPoint;
  if (fmod(tempSetPoint, 5) > 2.5){
    rounded += 5.0 - fmod(tempSetPoint, 5);
  } else {
    rounded -= fmod(tempSetPoint, 5);
  }

  return rounded;
}

int calculateRequiredHeat(float setTemp, float currentTemp){
  pid_input = (double)currentTemp;
  pid_setPoint = (double)setTemp;
  tempController.Compute();

  unsigned long currentTime = millis();
  if (currentTime - lastRefreshPlot >= refreshPlot){
    String output = "Temp:" + String(currentTemp,0) + "  Set:" + String(setTemp,0) + "  Heat:" + String((int)pid_output) + "  Low:" + String(0.9*setTemp,0) + "  High:" + String(1.1*setTemp,0);
    Serial.println(output);
    lastRefreshPlot = currentTime;
  }
  return (int)pid_output;
}

void setHeatOutput(int heatPercent){
  unsigned long currentTime = millis();
  unsigned int heatProgress = (float)(currentTime - previousHeatingStart) * 100.0 / (float)heatingInterval;

  if (heatProgress < heatPercent || heatPercent > 96){ /* If we haven't made it to the setting, Active High */
    digitalWrite(heatRelayPin, HIGH);
  } else {
    digitalWrite(heatRelayPin, LOW);
  }
  if (currentTime - previousHeatingStart >= heatingInterval){
    previousHeatingStart = currentTime;
  }
}

void writeLCDdata(float tempSetPoint, int displayID, int heatPercent){
  // Build Current Temp String
  String currentTempAsString( getCurrentTemp(), 0);
  String line_1 = "TEMP ";
  for (unsigned int i = 0; i < 3-currentTempAsString.length(); i++){
    line_1 += " ";
  }
  line_1 += currentTempAsString;
  line_1 += (char)223;
  line_1 += "F";

  // Build Setpoint String
  String line_2 = " SET ";
  String tSet;
  if (tempSetPointPotConfigured){
    tSet += String( tempSetPoint, 0 );
    tSet += (char)223;
    tSet += "F";
  } else {
    tSet += "Conf";
  }
  for (unsigned int i = 0; i < 5-tSet.length(); i++){
    line_2 += " ";
  }
  line_2 += tSet;

  // Cycling Display Data
  if (displayID == 0){
    // Display Heat Rate
    line_1 += "  HEAT";
    String heat(heatPercent);
    for (unsigned int i = 0; i < 5-heat.length(); i++){
      line_2 += " ";
    }
    line_2 += heat + "%";
  } else if (displayID == 1){
    // Display Elapsed Time
    line_1 += "  TIME";
    String elapsed = getTime();
    for (unsigned int i = 0; i < 6-elapsed.length(); i++){
      line_2 += " ";
    }
    line_2 += elapsed;
  }

  // Write Lines to LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(line_1);
  lcd.setCursor(0,1);
  lcd.print(line_2);

  // Serial Output
//  Serial.println(line_1);
//  Serial.println(line_2);
}

String getTime(){
  unsigned long currentTime = millis();

  // Only start timer once above threshold temp
  if (getCurrentTemp() < thresholdTemp && !thresholdMet){
    return String("Wait");
  }
  if (getCurrentTemp() >= thresholdTemp && !thresholdMet){
    timerThreshold = currentTime;
    thresholdMet = true;
  }

  unsigned long elapsed = (currentTime - timerThreshold)/1000;
  if (elapsed < 60){
    return String(elapsed) + "s";
  } else if (elapsed >= 60 && elapsed < 3600){
    long mins = elapsed / 60;
    long secs = elapsed - mins * 60;
    return String(mins) + ":" + (secs < 10 ? String(0) : "") + String(secs);
  } else {
    return String(">1hr");
  }
}

/*
// Sensor Objects
const unsigned int lcd_RS = 6, lcd_E = 7, lcd_D4 = 8, lcd_D5 = 9, lcd_D6 = 10, lcd_D7 = 11;
LiquidCrystal lcd(lcd_RS, lcd_E, lcd_D4, lcd_D5, lcd_D6, lcd_D7);
const int thermo_SO = 5, thermo_CS = 4, thermo_SCK = 3;
MAX6675 thermo(thermo_SCK, thermo_CS, thermo_SO);

// LCD Refresh Rate
const unsigned long refreshRateLCD = 200;
unsigned long previousRefreshLCD = 0;
const unsigned long cycleRate = 2000;
unsigned long previousCycle = 0;
int currentDisp = 0; // Cycle through the display types

// Plot Temp
const unsigned long refreshPlot = 0;
unsigned long lastRefreshPlot = 0;

// Timer Start Threshold
unsigned int thresholdTemp = 120;
unsigned long timerThreshold = 0;
bool thresholdMet = false;

// Temp Set Potentiometer
unsigned int setTempPotRange[] = {1024, 0};
const byte setTempPotPin = A0;
bool setTempPotConfigured = false;

// Heat Relay
const unsigned int heatRelayPin = 12;
const unsigned long heatingInterval = 330 ;
unsigned long previousHeatingStart = 0;

// PID control
double pid_input;
double pid_output;
double pid_setPoint;
PID tempController(&pid_input, &pid_output, &pid_setPoint, 0.5, 0.05, 1.0, DIRECT);

// Forward declaration of helper functions
float getSetTemp();
float getCurrentTemp();
int calculateRequiredHeat(int setTemp, int currentTemp);
void setHeatOutput(int heatPercent);
String getTime();
void writeLCDdata(int setTemp, int currentDisp);

void setup() {
  lcd.begin(16,2);
  tempController.SetMode(AUTOMATIC);
  tempController.SetOutputLimits(0,100);
  Serial.begin(9600);
  pinMode(heatRelayPin, OUTPUT);
  digitalWrite(heatRelayPin, LOW);
}

void loop() {
  // Get Current Time
  unsigned long currentTime = millis();
  
  // Read Temp set point
  float setTemp = getSetTemp();
  float currentTemp = getCurrentTemp();

  // Calculate required Heat
  int heatPercent = calculateRequiredHeat(setTemp, currentTemp);
  setHeatOutput(heatPercent);

  // Write LCD
  if (currentTime - previousRefreshLCD >= refreshRateLCD){
    writeLCDdata(setTemp, currentDisp, heatPercent);
    previousRefreshLCD = currentTime;
    // Check if ready to cycle to next display item
    if (currentTime - previousCycle >= cycleRate){
      currentDisp = (currentDisp + 1) % 2;
      previousCycle = currentTime;
    }
  }
}

//================================================================================
// Helper Function Bodies
//================================================================================
void setHeatOutput(int heatPercent){
  unsigned long currentTime = millis();
  unsigned int heatProgress = (float)(currentTime - previousHeatingStart) * 100.0 / (float)heatingInterval;

  if (heatProgress < heatPercent || heatPercent > 96){*/ /* If we haven't made it to the setting, Active High*//*
    digitalWrite(heatRelayPin, HIGH);
  } else {
    digitalWrite(heatRelayPin, LOW);
  }
  if (currentTime - previousHeatingStart >= heatingInterval){
    previousHeatingStart = currentTime;
  }
}

float getSetTemp(){
  unsigned int temp = analogRead(setTempPotPin);
  if (temp < setTempPotRange[0]){
    setTempPotRange[0] = temp;
  }
  if (temp > setTempPotRange[1]){
    setTempPotRange[1] = temp;
  }
  if (setTempPotRange[1] - setTempPotRange[0] > 500 && !setTempPotConfigured){
    setTempPotConfigured = true;
  }
  if (!setTempPotConfigured){
    return getCurrentTemp();
  }

  float potPercentage = (float)(temp - setTempPotRange[0]) / (float)(setTempPotRange[1] - setTempPotRange[0]);
  float setTemp = potPercentage * (400.0 - 50.0) + 50.0;
  
  // Round percentage to nearest 5
  float rounded = setTemp;
//  if (fmod(setTemp, 5) > 2.5){
//    rounded += 5.0 - fmod(setTemp, 5);
//  } else {
//    rounded -= fmod(setTemp, 5);
//  }
  
//  Serial.print("Percentage: " + String(potPercentage));
//  Serial.print("  Temp: " + String(setTemp));
//  Serial.println("  Temp: " + String(rounded));
  return rounded;
}

float getCurrentTemp(){
//  return map(analogRead(A1), 0, 1024, 20.0,500.0);
  return thermo.readFahrenheit();
}

int calculateRequiredHeat(int setTemp, int currentTemp){
  pid_input = (double)currentTemp;
  pid_setPoint = (double)setTemp;
//  tempController.Compute();

  unsigned long currentTime = millis();
  if (currentTime - lastRefreshPlot >= refreshPlot){
    String output = "Temp:" + String(currentTemp) + "  Set:" + String(setTemp) + "  Heat:" + String((int)pid_output);
    Serial.println(output);
    lastRefreshPlot = currentTime;
  }
  return (int)pid_output;
}

void writeLCDdata(int setTemp, int currentDisp, int heatPercent){
  // Get Current Temp
  String currentTemp = String( getCurrentTemp() );
  String line_1 = "TEMP ";
  for (unsigned int i = 0; i < 3-currentTemp.length(); i++){
    line_1 += " ";
  }
  line_1 += currentTemp;
  line_1 += (char)223;
  line_1 += "F";

  // Write Temp Setpoint
  String line_2 = " SET ";
  String tSet;
  if (setTempPotConfigured){
    tSet = setTemp;
    tSet += (char)223;
    tSet += "F";
  } else {
    tSet = "Conf";
  }
  for (unsigned int i = 0; i < 5-tSet.length(); i++){
    line_2 += " ";
  }
  line_2 += tSet;

  // Cycling Display
  if (currentDisp == 0){
    // Display Heat Rate
    line_1 += "  HEAT";
    String heat(heatPercent);
    for (unsigned int i = 0; i < 5-heat.length(); i++){
      line_2 += " ";
    }
    line_2 += heat + "%";
  } else if (currentDisp == 1){
    // Display Elapsed Time
    line_1 += "  TIME";
    String elapsed = getTime();
    for (unsigned int i = 0; i < 6-elapsed.length(); i++){
      line_2 += " ";
    }
    line_2 += elapsed;
  }

  // Write Lines to LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(line_1);
  lcd.setCursor(0,1);
  lcd.print(line_2);
}

String getTime(){
  unsigned long currentTime = millis();

  // Only start timer once above threshold temp
  //  if (getCurrentTemp() < thresholdTemp && !thresholdMet){
  if (getCurrentTemp() < thresholdTemp && !thresholdMet){
    return String("Wait");
  }
  // if (getCurrentTemp() >= thresholdTemp && !thresholdMet){
  if (getCurrentTemp() >= thresholdTemp && !thresholdMet){
    timerThreshold = currentTime;
    thresholdMet = true;
  }
  
  int elapsed = (currentTime-timerThreshold)/1000;
  if (elapsed < 60){
    return String(elapsed) + "s";
  } else if (elapsed >= 60 || elapsed < 3600){
    int mins = elapsed / 60;
    int secs = elapsed - mins * 60;
    return String(mins) + ":" + (secs < 10 ? String(0) : "") + String(secs);
  } else {
    return String(">1hr");
  }
}
*/
