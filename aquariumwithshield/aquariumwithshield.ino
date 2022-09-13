#include <Wire.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Ethernet.h>
#include <aREST.h>
#include <avr/wdt.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);
OneWire  ds(10);
IPAddress ip(192,168,2,2);
EthernetServer server(80);

#define SensorPin A0
#define Offset 0.10
#define LED 13
#define samplingInterval 20
#define printInterval 800
#define TdsSensorPin A1
#define VREF 5.0
#define SCOUNT  30
#define ArrayLenth  40

int pHArray[ArrayLenth];
int pHArrayIndex = 0;
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
int speakerPin = 7;
int lampSwitch = 8;
int co2Switch = 9;
int pumpOutSwitch  = 11;
int switchFour = 12;

bool lampSwitchStatus = false;
bool co2SwitchStatus = false;
bool pumpOutSwitchStatus  = false;
bool switchFourStatus = false;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25;

float celsius;
float fahrenheit;
float pHValue;
float voltage;

aREST rest = aREST();

bool turnPumpOutOn();
bool turnPumpOutOff();
bool turnLampOn();
bool turnLampOff();
bool turnCo2On();
bool turnCo2Off();


void setup()
{
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  lcd.init();
  lcd.backlight();
  pinMode(TdsSensorPin, INPUT);
  pinMode(lampSwitch, OUTPUT);
  pinMode(co2Switch, OUTPUT);
  pinMode(pumpOutSwitchStatus, OUTPUT);
  pinMode(switchFour, OUTPUT);
  pinMode (speakerPin, OUTPUT);

  rest.variable("lampSwitchStatus",&lampSwitchStatus);
  rest.variable("co2SwitchStatus",&co2SwitchStatus);
  rest.variable("pumpOutSwitchStatus",&pumpOutSwitchStatus);
  rest.variable("switchFourStatus",&switchFourStatus);
  rest.variable("tdsValue",&tdsValue);
  rest.variable("tdsVoltage",&averageVoltage);
  rest.variable("celsius",&celsius);
  rest.variable("fahrenheit",&fahrenheit);
  rest.variable("phValue",&pHValue);
  rest.variable("phVoltage",&voltage);

  rest.function("turnPumpOutOn",turnPumpOutOn);
  rest.function("turnPumpOutOff",turnPumpOutOff);
  rest.function("turnLampOn",turnLampOn);
  rest.function("turnLampOff",turnLampOff);
  rest.function("turnCo2On",turnCo2On);
  rest.function("turnCo2Off",turnCo2Off);

  rest.set_id("0001");
  rest.set_name("aquarium-lamp-01");

  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, ip);
  }
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

  // Start watchdog
  wdt_enable(WDTO_4S);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Aquarium Lamp");
  lcd.setCursor(0, 1);
  lcd.print("Version 1.0");
  lcd.setCursor(0, 2);
  lcd.print("Dawid Krzus");
  lcd.setCursor(0, 3);
}

bool turnPumpOutOn() {
  digitalWrite(pumpOutSwitch, HIGH);
  pumpOutSwitchStatus = true;
  return true;
}

bool turnPumpOutOff() {
  digitalWrite(pumpOutSwitch, LOW);
  pumpOutSwitchStatus = false;
  return false;
}

bool turnLampOn() {
  digitalWrite(lampSwitch, HIGH);
  lampSwitchStatus = true;
  return true;
}

bool turnLampOff() {
  digitalWrite(lampSwitch, LOW);
  lampSwitchStatus = false;
  return false;
}

bool turnCo2On() {
  digitalWrite(co2Switch, HIGH);
  co2SwitchStatus = true;
  return true;
}

bool turnCo2Off() {
  digitalWrite(co2Switch, LOW);
  co2SwitchStatus = false;
  return false;
}

void triggerBuzzer() {
  analogWrite (speakerPin, 255);
  delay (500);
  analogWrite (speakerPin, 0);
}

int tdsMedian(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

void tdsMeasure() {
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = tdsMedian(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    float compensationVolatge = averageVoltage / compensationCoefficient;
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5;


    Serial.print("TDS: ");
    Serial.println(tdsValue);
    Serial.print("TDS Voltage: ");
    Serial.println(averageVoltage);
  }
}

void tempMeasure() {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[9];
  byte addr[8];
  if ( !ds.search(addr)) {
    ds.reset_search();
    delay(250);
    return;
  }

  //Serial.print("ROM =");
  for ( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return;
  }
  Serial.println();

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print(" Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");

}

float phMeasure() {
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  if (millis() - samplingTime > samplingInterval)
  {
    pHArray[pHArrayIndex++] = analogRead(SensorPin);
    if (pHArrayIndex == ArrayLenth)pHArrayIndex = 0;
    voltage = phAvergeArray(pHArray, ArrayLenth) * 5.0 / 1024;
    pHValue = 3.5 * voltage + Offset;
    samplingTime = millis();
  }
  if (millis() - printTime > printInterval)  //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {

    Serial.print("Ph: ");
    Serial.println(pHValue);
    Serial.print("Ph Voltage: ");
    Serial.println(voltage);

    digitalWrite(LED, digitalRead(LED) ^ 1);
    printTime = millis();
  }

  return pHValue;
}

double phAvergeArray(int* arr, int number) {
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5) { //less than 5, calculated directly statistics
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  } else {
    if (arr[0] < arr[1]) {
      min = arr[0]; max = arr[1];
    }
    else {
      min = arr[1]; max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;      //arr<min
        min = arr[i];
      } else {
        if (arr[i] > max) {
          amount += max;  //arr>max
          max = arr[i];
        } else {
          amount += arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount / (number - 2);
  }//if
  return avg;
}

void clearLcdAndTriggerBuzzer() {
  lcd.clear();
  lcd.setCursor(0, 0);
  triggerBuzzer();
}

void loop()
{
  phMeasure();
  tempMeasure();
  tdsMeasure();

  EthernetClient client = server.available();
  rest.handle(client);
  wdt_reset();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TDS: ");
  lcd.print(tdsValue);
  lcd.print("ppm ");
  lcd.print("V: ");
  lcd.print(averageVoltage);
  lcd.print("V");

  lcd.setCursor(0, 1);
  lcd.print("Tmp: ");
  lcd.print(celsius);
  lcd.print("C, ");
  lcd.print(fahrenheit);
  lcd.print("F");

  lcd.setCursor(0, 2);
  lcd.print("Ph: ");
  lcd.print(pHValue);
  lcd.print("ph ");
  lcd.print("V: ");
  lcd.print(voltage);
  lcd.print("V");

  delay(2000);
    
  lcd.clear();
  
  if (pHValue < 6) {
    lcd.setCursor(0, 0);
    Serial.println("Ph is too low!");
    lcd.print("Ph is too low!");
    turnCo2Off();
  }

  if (pHValue > 8) {
    lcd.setCursor(0, 0);
    Serial.println("Ph is too high!");
    lcd.print("Ph is too high!");
    turnCo2On();
  }

  if (celsius < 22) {
    lcd.setCursor(0, 1);
    Serial.println("Temp is too low!");
    lcd.print("Temp is too low!");
  }

  if (celsius > 26) {
    lcd.setCursor(0, 1);
    Serial.println("Temp is too high!");
    lcd.print("Temp is too high!");
  }

  if (tdsValue > 200) {
    lcd.setCursor(0, 2);
    Serial.println("TDS is too high!");
    lcd.print("TDS is too high!");
  }

  delay(2000);
    
  lcd.clear();
  lcd.setCursor(0, 0);
  (lampSwitchStatus == true)? lcd.println("Lamp is on!") : lcd.println("Lamp is off!");
  lcd.setCursor(0, 1);
  (co2SwitchStatus == true)? lcd.println("Co2 is on!") : lcd.println("Co2 is off!");
  lcd.setCursor(0, 2);
  (pumpOutSwitchStatus == true)? lcd.println("Pump out is on!") : lcd.println("Pump out is off!");  
  lcd.setCursor(0, 3);
  (switchFourStatus == true)? lcd.println("Switch 4 is on!") : lcd.println("Switch 4 is off!");  
  delay(2000);

}
