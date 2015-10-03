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
  
  BLE Nano
  http://redbearlab.com/blenano/
  https://github.com/RedBearLab/nRF51822-Arduino
  http://redbearlab.com/getting-started-nrf51822
*/

#include <stdio.h>

/**
 * Source: https://chromium.googlesource.com/chromiumos/platform/vboot_reference/+/master/firmware/lib/crc8.c
 * Return CRC-8 of the data, using x^8 + x^2 + x + 1 polynomial.  A table-based
 * algorithm would be faster, but for only a few bytes it isn't worth the code
 * size. */
uint8_t Crc8(const void *vptr, int len)
{
	const uint8_t *data = (uint8_t *) vptr;
	unsigned crc = 0;
	int i, j;
	for (j = len; j; j--, data++) {
		crc ^= (*data << 8);
		for(i = 8; i; i--) {
			if (crc & 0x8000)
				crc ^= (0x1070 << 3);
			crc <<= 1;
		}
	}
	return (uint8_t)(crc >> 8);
}

// Arduino Task Scheduler for reading sensors every 1 second.
// https://github.com/arkhipenko/TaskScheduler
#include <TaskScheduler.h>
Task readSensorsTask(5000, -1, &readSensorsTaskCallback);
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


void writeData(float temperature, float humidity, char sensor, float reading) {
  const int TEXT_BUF = 100;
  char buf[TEXT_BUF];
  int bytes = snprintf(buf, TEXT_BUF, "t: %s h: %s %c: %s ", String(temperature, 1).c_str(), String(humidity, 1).c_str(), sensor, String(reading, 1).c_str());
  Serial.write(buf, bytes);
  uint8_t crc = Crc8(buf, bytes);
  bytes = snprintf(buf, TEXT_BUF, "*%03d*\n", crc);
  Serial.write(buf, bytes);
}

void readSensorsTaskCallback() {
  // read temperature, humidity
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  // read particle sensor
  writeData(temperature, humidity, 'p', readDustSensor());
  // read CO sensor
  writeData(temperature, humidity, heaterOn ? 'C' : 'c', readCoSensor());
}


float readDustSensor() {
  // Function to read the dust sensor
  
  // Sharp Optical Dust Sensor GP2Y10 loop
  digitalWrite(ledPower,LOW); // power on the LED
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin); // read the dust value
  
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH); // turn the LED off
  delayMicroseconds(sleepTime); 
  
  return voMeasured;
}


float readCoSensor() {
  // Function to read the Carbon Monoxide sensor
  
  mqReading = analogRead(mqSensor);

  return mqReading;
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
  readSensorsTask.enable();
  runner.addTask(readSensorsTask);
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
