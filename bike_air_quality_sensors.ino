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
  
  BLE Redbear Duo
  http://redbear.cc/duo/
  https://github.com/redbear/Duo
  https://github.com/redbear/STM32-Arduino
*/

// **BLE**

// For RedBear Duo don't connect to the cloud.
SYSTEM_MODE(MANUAL);

#define DEVICE_NAME                "Air quality"

#define CHARACTERISTIC1_MAX_LEN    20
#define CHARACTERISTIC2_MAX_LEN    20

#define TXRX_BUF_LEN               20

/******************************************************
 *               Variable Definitions
 ******************************************************/
static uint8_t service1_uuid[16]       ={0x71,0x3d,0x00,0x00,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e};
static uint8_t service1_tx_uuid[16]    ={0x71,0x3d,0x00,0x03,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e};
static uint8_t service1_rx_uuid[16]    ={0x71,0x3d,0x00,0x02,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e};
// From iOS app
//static uint8_t service1_rx_uuid[16]    ={0x71,0x3D,0x00,0x03,0x50,0x3E,0x4C,0x75,0xBA,0x94,0x31,0x48,0xF1,0x8D,0x94,0x1E,};

static uint8_t  appearance[2]    = {0x00, 0x02};
static uint8_t  change[2]        = {0x00, 0x00};
static uint8_t  conn_param[8]    = {0x28, 0x00, 0x90, 0x01, 0x00, 0x00, 0x90, 0x01};

static uint16_t character1_handle = 0x0000;
static uint16_t character2_handle = 0x0000;
static uint16_t character3_handle = 0x0000;

static uint8_t characteristic1_data[CHARACTERISTIC1_MAX_LEN]={0x01};
static uint8_t characteristic2_data[CHARACTERISTIC2_MAX_LEN]={0x00};


static btstack_timer_source_t characteristic2;

static advParams_t adv_params;
static uint8_t adv_data[]={0x02,0x01,0x06,0x08,0x08,'B','i','s','c','u','i','t',0x11,0x07,0x1e,0x94,0x8d,0xf1,0x48,0x31,0x94,0xba,0x75,0x4c,0x3e,0x50,0x00,0x00,0x3d,0x71};

char rx_buf[TXRX_BUF_LEN];
static uint8_t rx_buf_num;
static uint8_t rx_state=0;


/******************************************************
 *               Function Definitions
 ******************************************************/

 
void deviceConnectedCallback(BLEStatus_t status, uint16_t handle) {
    switch (status){
        case BLE_STATUS_OK:
//            Serial.println("Device connected!");
            break;
        default:
            break;
    }
}

void deviceDisconnectedCallback(uint16_t handle){
    Serial.println("Disconnected.");
}


int gattWriteCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size)
{
//    Serial.print("Write value handler: ");
//    Serial.println(value_handle, HEX);

    if(character1_handle == value_handle)
    {
        memcpy(characteristic1_data, buffer, min(size,CHARACTERISTIC1_MAX_LEN));
        Serial.print("Characteristic1 write value: ");
        for(uint8_t index=0; index<min(size,CHARACTERISTIC1_MAX_LEN); index++)
        {
            Serial.print(characteristic1_data[index], HEX);
            Serial.print(" ");
        }
        Serial.println(" ");
    }
  

    return 0;
}

//void m_uart_rx_handle()
//{   //update characteristic data
//    ble.sendNotify(character2_handle, (uint8_t*)rx_buf, CHARACTERISTIC2_MAX_LEN);
//    memset(rx_buf, 0x00,20);
//    rx_state = 0;
//}

static void  characteristic2_notify(btstack_timer_source_t *ts)
{   

    if (Serial.available()) {
      //read the serial command into a buffer
      
      uint8_t rx_len =min(Serial.available(),CHARACTERISTIC2_MAX_LEN);
      
      Serial.readBytes(rx_buf, rx_len);
      //send the serial command to the server
  
//      Serial.print("Sent: ");
//      Serial.println(rx_buf);
      rx_state = 1;
    }
    if(rx_state != 0)
    {
        ble.sendNotify(character2_handle, (uint8_t*)rx_buf, CHARACTERISTIC2_MAX_LEN);
        memset(rx_buf, 0x00,20);
        rx_state = 0;
    }
    // reset
    ble.setTimer(ts, 200);
    ble.addTimer(ts);
  
}

// **BLE END**

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

void readSensorsTaskCallback();
void coHeaterTaskCallback();

Task readSensorsTask(2000, -1, &readSensorsTaskCallback);
Task coHeaterTask(60000, -1, &coHeaterTaskCallback);
Scheduler runner;

// DHT humidity/temp sensor
//#include "DHT.h"
//#define DHTPIN 2
//#define DHTTYPE DHT22

// Initialize DHT sensor.
//DHT dht(DHTPIN, DHTTYPE);

#include <dht.h>
dht DHT;
#define DHT22_PIN 6

struct
{
    uint32_t total;
    uint32_t ok;
    uint32_t crc_error;
    uint32_t time_out;
    uint32_t connect;
    uint32_t ack_l;
    uint32_t ack_h;
    uint32_t unknown;
} stat = { 0,0,0,0,0,0,0,0};


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


struct SENSOR_READINGS {
    float temperature;
    float humidity;
    float particles;
    float co;
    char heaterOn;
};

float readDustSensor();
float readCoSensor();

void readSensorsTaskCallback() {
  struct SENSOR_READINGS readings;
  uint32_t start;
  int chk;
  uint32_t stop;
  // read temperature, humidity
  SINGLE_THREADED_BLOCK() {
    start = micros();
    chk = DHT.read22(DHT22_PIN);
    readings.humidity = DHT.humidity;
    readings.temperature = DHT.temperature;
    stop = micros();
  }
  stat.total++;
  switch (chk)
  {
  case DHTLIB_OK:
      stat.ok++;
      // Serial.print("OK,\t");
      break;
  case DHTLIB_ERROR_CHECKSUM:
      stat.crc_error++;
      Serial.print("Checksum error,\t");
      break;
  case DHTLIB_ERROR_TIMEOUT:
      stat.time_out++;
      Serial.print("Time out error,\t");
      break;
  default:
      stat.unknown++;
      Serial.print("Unknown error,\t");
      break;
  }
//    Serial.print(DHT.humidity, 1);
//    Serial.print(",\t");
//    Serial.print(DHT.temperature, 1);
//    Serial.print(",\t");
//    Serial.print(stop - start);
//    Serial.println();
  // DISPLAY DATA
  // read particle sensor
  readings.particles = readDustSensor();
  // read CO sensor
  readings.heaterOn = heaterOn ? 1 : 0;
  readings.co = readCoSensor();
  //  Serial.write((uint8_t*) &readings, sizeof(SENSOR_READINGS));
  // Send data over BLE
  // TOFIX: Only sending every 80bytes instead of 20bytes
  ble.sendNotify(character2_handle, (uint8_t*) &readings, sizeof(SENSOR_READINGS));
  // for debugging
  Serial.print("t: ");
  Serial.print(readings.temperature);
  Serial.print(" h: ");
  Serial.print(readings.humidity);
  Serial.print(" p: ");
  Serial.print(readings.particles);
  Serial.print(heaterOn ? " C: " : " c: ");
  Serial.println(readings.co);
//    Serial.println(sizeof(SENSOR_READINGS));
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
    // Serial.println("=");
    // Serial.println("=Heater is on for 60s");
    // Serial.println("=");
    coHeaterTask.setInterval(60000);
    
  } else {
    analogWrite(pwmPower, (255 - 255*(1.4/5)));
    heaterOn = false;
    // Serial.println("=");
    // Serial.println("=Heater is off for 90s");
    // Serial.println("=");
    coHeaterTask.setInterval(90000);
  }
  
}


void setup() {
  
  Serial.begin(115200);
  // Serial.println("=BIKE AIR QUALITY SENSOR");
  // Serial.println("========================");

  // **BLE**

  ble.init();

  ble.onConnectedCallback(deviceConnectedCallback);
  ble.onDisconnectedCallback(deviceDisconnectedCallback);
  ble.onDataWriteCallback(gattWriteCallback);

  ble.addService(0x1800);
  ble.addCharacteristic(0x2A00, ATT_PROPERTY_READ|ATT_PROPERTY_WRITE, (uint8_t*)DEVICE_NAME, sizeof(DEVICE_NAME));
  ble.addCharacteristic(0x2A01, ATT_PROPERTY_READ, appearance, sizeof(appearance));
  ble.addCharacteristic(0x2A04, ATT_PROPERTY_READ, conn_param, sizeof(conn_param));
  ble.addService(0x1801);
  ble.addCharacteristic(0x2A05, ATT_PROPERTY_INDICATE, change, sizeof(change));


  ble.addService(service1_uuid);
  character1_handle = ble.addCharacteristicDynamic(service1_tx_uuid, ATT_PROPERTY_NOTIFY|ATT_PROPERTY_WRITE|ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, characteristic1_data, CHARACTERISTIC1_MAX_LEN);
  character2_handle = ble.addCharacteristicDynamic(service1_rx_uuid, ATT_PROPERTY_NOTIFY, characteristic2_data, CHARACTERISTIC2_MAX_LEN);
  
  adv_params.adv_int_min = 0x00A0;
  adv_params.adv_int_max = 0x01A0;
  adv_params.adv_type    = 0;
  adv_params.dir_addr_type = 0;
  memset(adv_params.dir_addr,0,6);
  adv_params.channel_map = 0x07;
  adv_params.filter_policy = 0x00;
  
  ble.setAdvParams(&adv_params);
  
  ble.setAdvData(sizeof(adv_data), adv_data);

  ble.startAdvertising();

  // set one-shot timer
  //characteristic2.process = &characteristic2_notify;
  //ble.setTimer(&characteristic2, 500);//100ms
  //ble.addTimer(&characteristic2);

  // **BLE END**
  
  // Task scheduler setup
  // Serial.println("=Setting up tasks");
  readSensorsTask.enable();
  runner.addTask(readSensorsTask);
  coHeaterTask.enable();
  runner.addTask(coHeaterTask);
  
  runner.enableAll();
  
  // DHT setup
  // Serial.println("=Setting up DHT22 temp/humidity sensor");
//  dht.begin();
  Serial.println("dht22_test.ino");
  Serial.print("LIBRARY VERSION: ");
  Serial.println(DHT_LIB_VERSION);
  
  // Sharp Optical Dust Sensor GP2Y10 setup
  pinMode(ledPower,OUTPUT);
  
}

void loop() {
  
  // Run tasks
  runner.execute();
//  readSensorsTaskCallback();

}
