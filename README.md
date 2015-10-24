# Bike air quality sensors

[<img src="https://igcdn-photos-h-a.akamaihd.net/hphotos-ak-xaf1/t51.2885-15/e35/11430393_893793017342471_1356872161_n.jpg" width="30%" style="float: right;" />](https://instagram.com/p/8FcTIRgpwA)

A set of Carbon Monoxide & dust particle sensors combined with humidity/temperature sensors for measuring the air quality while cycling.

The sensors are hooked up to an Arduino which then sends the data via serial over Bluetooth to our Android or iOS phone to be time stamped and have GPS data added.

This data will then be sent to a Rails server to graph the data - and also provide an API for others to interact with it.

by Damien & Simon

## Hardware

* [Arduino Uno](https://www.arduino.cc/en/Main/arduinoBoardUno) for prototyping, something smaller for the final product
* [MQ-7 Carbon Monoxide sensor](https://www.sparkfun.com/products/9403)
* [GP2Y10 Dust Particle sensor](https://www.sparkfun.com/products/9689)
* [DHT-22 Temperature/Humidity sensor](https://www.sparkfun.com/products/10167)
* [BLE Nano Bluetooth module](http://littlebirdelectronics.com.au/collections/redbearlabs/products/ble-nano-kit) for iOS
* [HC-06 Bluetooth module](http://www.miniinthebox.com/hc-06-wireless-bluetooth-transceiver-rf-main-module-serial-for-arduino_p903460.html) for Android

## Output

The sensors are read every second and output in a struct:

`struct SENSOR_READINGS {
    float temperature;
    float humidity;
    float particles;
    float co;
    char heaterOn;
};`

* Temperature in Celsius
* Humidity %
* Dust particle raw sensor value
* Carbon monoxide raw sensor value
* Heater on/off sent as 1 or 0

## Libraries

Thanks for the DHT sensor library:
<https://learn.adafruit.com/dht/using-a-dhtxx-sensor>

Thanks for the Sharp Optical Dust Sensor library:
<http://arduinodev.woofex.net/2012/12/01/standalone-sharp-dust-sensor/>

MQ-7 Carbon Monoxide sensor:
<http://thesis.jmsaavedra.com/make/>

## iOS & Android apps

[iOS app](https://github.com/sighmon/BikeAirQualitySensorsiOS)

[Android app](https://github.com/33d/bike-air-sensor-logger)

Both apps add:

* ISO 8601 formatted time, including the time zone.
* The position - latitude and longitude.

Then they export the data to a CSV file.


## Python data parser for the CSV

The Carbon Monoxide sensor can only be read after it's been heating for 60 seconds, so we need to throw away all of the data around this.

This Python parser does just that. :-)

<https://github.com/sighmon/bike_air_quality_sensor_csv_parser>

## TODO

* <s>Connect sensors to the Arduino.</s>
* <s>Send data via Bluetooth.</s>
* <s>Build an iOS app.</s>
* <s>Build an Android app.</s>
* Build a Rails server to open up the data via a public API.

