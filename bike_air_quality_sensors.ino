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

  TODO: MQ-7 Carbon Monoxide sensor:
  http://thesis.jmsaavedra.com/make/
*/

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


void setup() {
  
  // DHT setup
  Serial.begin(9600);
  Serial.println("DHT22 is alive!");
  dht.begin();
  
  // Sharp Optical Dust Sensor GP2Y10 setup
  pinMode(ledPower,OUTPUT);
  
}

void loop() {
  
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
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  
  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.print(" *C ");
  Serial.print(hif);
  Serial.println(" *F");
  
  
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
  
  Serial.print("Raw Signal Value (0-1023): ");
  Serial.print(voMeasured);
  
  Serial.print(" - Voltage: ");
  Serial.print(calcVoltage);
  
  Serial.print(" - Dust Density [ug/m3]: ");
  Serial.println(dustDensity);
  
  delay(1000);
  
}
