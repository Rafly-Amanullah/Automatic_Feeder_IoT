#define BLYNK_TEMPLATE_ID "TMPL2BZebhDQt"
#define BLYNK_TEMPLATE_NAME "Automatic Feeder"
#define BLYNK_AUTH_TOKEN "bpu2DHgOoIWQ8sMF4S8oeI8tgYJpN0C3"
#define ONE_WIRE_BUS 15
#define echoPin 33
#define trigPin 25
#define relaypin 4 
#define VREF 5000
#define ADC_RES 1024
#define READ_TEMP (25)
#define CAL1_V (131)
#define CAL1_T (25)
#define SIM800L_IP5306_VERSION_20200811
#define TINY_GSM_MODEM_SIM800
#define SerialMon Serial
#define SerialAT Serial1
#define TINY_GSM_DEBUG SerialMon

#include <utilities.h>
#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>
#include <SoftwareSerial.h>
#include <StreamDebugger.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <DFRobot_ESP_EC.h>
#include <EEPROM.h>
#include <MedianFilter.h>

OneWire oneWire(ONE_WIRE_BUS);	
DallasTemperature sensors(&oneWire);
DFRobot_ESP_EC ec;
Adafruit_ADS1X15 ads;
MedianFilter filter(31, 0);

char auth[] = BLYNK_AUTH_TOKEN;
char apn[]  = "internet";
char user[] = "";
char pass[] = "";

#ifdef DUMP_AT_COMMANDS
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif
TinyGsmClient client(modem);

BlynkTimer timer;

int suhu, distance, step = 0, percentage, filteredDistance;
const int minDistance = 20, maxDistance = 100;
float voltage, ecValue, temperature = 25, usValue;
unsigned long duration, previousMicros = 0;
const unsigned long interval = 10;
const uint16_t DO_Table[41] = {
  14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
  11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
  9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
  7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};
uint8_t Temperaturet;
uint16_t ADC_Raw, ADC_Voltage, DO;

BLYNK_WRITE(V4)
{
  int pinValue = param.asInt(); 
  if (pinValue == 1){
    digitalWrite(relaypin, HIGH);
    Serial.print("Device On");
  }
  else{
    digitalWrite(relaypin,LOW);
    Serial.println("Device Off");
  }
}
int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
}
void Event()
{
  Blynk.virtualWrite(V0,suhu);
  Blynk.virtualWrite(V1,ecValue);
  Blynk.virtualWrite(V2,percentage);
  Blynk.virtualWrite(V3,ADC_Voltage); 
}

void ultrasonic()
{
  unsigned long currentMicros = micros();
  if (step == 0) {
    digitalWrite(trigPin, LOW);
    step = 1;
    previousMicros = currentMicros;
  } 
  else if (step == 1 && currentMicros - previousMicros >= interval) {
    digitalWrite(trigPin, HIGH);
    previousMicros = currentMicros;
    step = 2;
  } 
  ;else if (step == 2 && currentMicros - previousMicros >= interval) {
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;
    filteredDistance = filter.in(distance);
    step = 0;
  }
}
int mapDistanceToPercentage(int dist) {
    int mappedPercentage = map(dist, minDistance, maxDistance, 100, 0);
    if (mappedPercentage < 0) {
        mappedPercentage = 0;
    } else if (mappedPercentage > 100) {
        mappedPercentage = 100;
    }
    return mappedPercentage;
}
void setup()
{
  pinMode(relaypin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(115200);
  setupModem();
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  SerialMon.println("Initializing modem...");
  modem.restart();
  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
  EEPROM.begin(32);
	ec.begin();
	ads.setGain(GAIN_ONE);
	ads.begin();
  Blynk.begin(auth, modem, apn, user, pass, "blynk.cloud", 8080);
  
  if (Blynk.connected()) {
    Serial.println("Connected to Blynk server!");
  } else {
    Serial.println("Not connected to Blynk server.");
  }  
  timer.setInterval(1000L, Event);
}
void loop()
{
  Blynk.run();
  timer.run();
  unsigned long currentMillis = millis();
  static unsigned long previousMillis = 0;
  const long interval = 100;
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        ultrasonic();
        Serial.println(filteredDistance);
        percentage = mapDistanceToPercentage(filteredDistance);
        sensors.requestTemperatures(); 
        suhu = sensors.getTempCByIndex(0);
        // voltage = ads.readADC_SingleEnded(0) / 10;
        // ecValue = ec.readEC(voltage, temperature );
        // Temperaturet = (uint8_t)READ_TEMP;
        // ADC_Raw = ads.readADC_SingleEnded(1)/10;
        // ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
    }
}