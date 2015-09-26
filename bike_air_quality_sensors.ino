/*
  A set of Carbon Dioxide and Carbon Monoxide sensors
  combined with humidity/temperature sensors
  for measuring the air quality while cycling.
  
  by Damien & Simon
  
  Thanks for the DHT sensor library:
  https://learn.adafruit.com/dht/using-a-dhtxx-sensor
  
  Thanks for the Sharp Optical Dust Sensor library:
  Kudos: http://arduinodev.woofex.net/2012/12/01/standalone-sharp-dust-sensor/
  GitHub: https://github.com/Trefex/arduino-airquality/tree/master/Module_Dust-Sensor

  MQ-7 Carbon Monoxide sensor:
  http://thesis.jmsaavedra.com/make/
*/

// Arduino Task Scheduler.
#include <TaskScheduler.h>
Task dustTask(1000, -1, &dustTaskCallback);
Task coReadTask(1000, -1, &coReadTaskCallback);
Task coHeaterTask(60000, -1, &coHeaterTaskCallback);
Scheduler runner;

// DHT humidity/temp sensor
#include "DHT.h"
#define DHTPIN 2
#define DHTTYPE DHT22

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);


// Sharp Optical Dust Sensor GP2Y10
int measurePin = 5;
int ledPower = 12;
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;


// MQ7 Sensor
int pwmPower = 3; // Digital 3
int mqSensor = 4; // Analog 4
float mqReading = 0;
bool heaterOn = false;


void readTemperatureAndHumidity() {
  // DHT measurements.
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("=Failed to read from DHT temp/humidity sensor!");
    return;
  }
  
  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

//  Serial.print("Humidity: ");
  Serial.print("\th:");
  Serial.print(h);
//  Serial.print(" %\t");
//  Serial.print("Temperature: ");
  Serial.print("\tt:");
  Serial.println(t);
//  Serial.print(" *C ");
//  Serial.print(f);
//  Serial.print(" *F\t");
//  Serial.print("Heat index: ");
//  Serial.print(hic);
//  Serial.print(" *C ");
//  Serial.print(hif);
//  Serial.println(" *F");
}


void dustTaskCallback() {
  // Callback function to read the dust sensor
  // NOTE: Also reads the temp/humidity sensor too
  
  // Sharp Optical Dust Sensor GP2Y10 loop
  digitalWrite(ledPower,LOW); // power on the LED
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin); // read the dust value
  
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH); // turn the LED off
  delayMicroseconds(sleepTime);

  // 0 - 5.0V mapped to 0 - 1023 integer values 
  calcVoltage = voMeasured * (5.0 / 1024); 
  
  // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
  // Chris Nafis (c) 2012
  dustDensity = (0.17 * calcVoltage - 0.1)*1000; 
  
//  Serial.print("Raw Signal Value (0-1023): ");
  Serial.print("d:");
  Serial.print(voMeasured);
  
//  Serial.print(" - Voltage: ");
//  Serial.print(calcVoltage);
  
//  Serial.print(" - Dust Density [ug/m3]: ");
//  Serial.println(dustDensity);
  
  readTemperatureAndHumidity();
}


void coReadTaskCallback() {
  // Callback function to read the Carbon Monoxide sensor
  // NOTE: Also reads the temp/humidity sensor too
  
  Serial.print("c:");
  mqReading = analogRead(mqSensor);
  Serial.print(mqReading);
  
  readTemperatureAndHumidity();
}


void coHeaterTaskCallback() {
  // Callback function to heat up the Carbon Monoxide sensor to the right temperature

  if (!heaterOn) {
    analogWrite(pwmPower, 0);
    heaterOn = true;
    Serial.println("=");
    Serial.println("=Heater is on for 60s");
    Serial.println("=");
    coHeaterTask.setInterval(60000);
    
  } else {
    analogWrite(pwmPower, (255 - 255*(1.4/5)));
    heaterOn = false;
    Serial.println("=");
    Serial.println("=Heater is off for 90s");
    Serial.println("=");
    coHeaterTask.setInterval(90000);
  }
  
}


void setup() {
  
  Serial.begin(9600);
  Serial.println("=BIKE AIR QUALITY SENSOR");
  Serial.println("========================");
  
  // Task scheduler setup
  Serial.println("=Setting up tasks");
  dustTask.enable();
  runner.addTask(dustTask);
  coReadTask.enable();
  runner.addTask(coReadTask);
  coHeaterTask.enable();
  runner.addTask(coHeaterTask);
  
  runner.enableAll();
  
  // DHT setup
  Serial.println("=Setting up DHT22 temp/humidity sensor");
  dht.begin();
  
  // Sharp Optical Dust Sensor GP2Y10 setup
  pinMode(ledPower,OUTPUT);
  
}

void loop() {
  
  // Run tasks
  runner.execute();

}
