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

The sensors are read every second and output in the format:

* d: = Dust particle raw sensor value
* c: = Carbon monoxide raw sensor value
* h: = Humidity %
* t: = Temperature in Celsius

<code>d:148.00	h:30.20	t:21.00</code>
<code>c:253.00	h:30.20	t:21.00</code>

## Libraries

Thanks for the DHT sensor library:
<https://learn.adafruit.com/dht/using-a-dhtxx-sensor>

Thanks for the Sharp Optical Dust Sensor library:
<http://arduinodev.woofex.net/2012/12/01/standalone-sharp-dust-sensor/>

MQ-7 Carbon Monoxide sensor:
<http://thesis.jmsaavedra.com/make/>

## TODO

* <s>Connect sensors to the Arduino.</s>
* <s>Send data via Bluetooth.</s>
* Build a Rails server.
* Build an Android app.
* Build an iOS app.