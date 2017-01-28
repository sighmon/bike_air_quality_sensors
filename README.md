# Bike air quality sensors

<img src="https://raw.githubusercontent.com/sighmon/bike_air_quality_sensors/redbear-duo-serial/sensors-redbearduo-v0.1.jpg" width="100%" />

A set of Carbon Monoxide & dust particle sensors combined with humidity/temperature sensors for measuring the air quality while cycling.

The sensors are hooked up to an Arduino which then sends the data via serial over Bluetooth to our Android or iOS phone to be time stamped and have GPS data added.

This data will then be sent to a Rails server to graph the data - and also provide an API for others to interact with it.

by Damien & Simon

## Hardware

* [Redbear Duo](https://github.com/redbear/Duo)
* [MQ-7 Carbon Monoxide sensor](https://www.sparkfun.com/products/9403)
* [GP2Y10 Dust Particle sensor](https://www.sparkfun.com/products/9689)
* [DHT-22 Temperature/Humidity sensor](https://www.sparkfun.com/products/10167)
* P-channel MOSFET
* 10K ohm resistor
* 20K ohm resistor
* 150 ohm resistor
* 220µF capacitor

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

### Power usage

* When heating the Carbon Monoxide sensor: 200 mA
* When not heating the Carbon Monoxide sensor: 130 mA


### Data from the iOS app with GPS location & timestamp

Google Spreadsheet: [bike sensor data](https://docs.google.com/spreadsheets/d/1QTtR4N6sVxgsv6Za4kGe6Mn_MdasWhGjeMLYdR82Do4/edit?usp=sharing)

[Interactive graph of the data](https://docs.google.com/spreadsheets/d/1QTtR4N6sVxgsv6Za4kGe6Mn_MdasWhGjeMLYdR82Do4/pubchart?oid=1328642199&amp;format=interactive)

<img src="https://raw.githubusercontent.com/sighmon/bike_air_quality_sensors/master/chart-data.png" width="100%" />


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


## Manufactured by OSH Park

<img src="https://raw.githubusercontent.com/sighmon/bike_air_quality_sensors/redbear-duo-serial/kicad/air-quality-sensors-v0.1.jpg" width="100%" />

I got the first batch of boards manufactured by <https://oshpark.com>. You can see the shared project here: <https://oshpark.com/profiles/sighmon>


## GovHack Machinery of Data entry video

<iframe width="100%" height="315" src="https://www.youtube.com/embed/KiIhrwRlb6w" frameborder="0" allowfullscreen></iframe>

## Troubleshooting

- If the RedBear Duo gets itself into a flashing blue light of doom, hold down both ```reset + setup``` for 3 seconds, then let go of the reset. Hold the setup button until the flashing turns yellow. That’s DFU mode to [re-upload the user firmware](https://github.com/redbear/Duo/blob/master/docs/firmware_deployment_guide.md).

``` $ dfu-util -d 2b04:d058 -a 0 -s 0x80C0000 -D duo-user-part.bin```

## TODO

* <s>Connect sensors to the Arduino.</s>
* <s>Send data via Bluetooth.</s>
* <s>Build an iOS app.</s>
* <s>Build an Android app.</s>
* Build a Rails server to open up the data via a public API.
