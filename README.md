# 6LoWeather: an open source 6LoWPAN weather station

This project allows to build an open weather station, running on [Zolertia's RE-Mote platform][RE-Mote], using 6LoWPAN/IPv6 with NAT64 to resolve IPv4-only MQTT brokers.

The application is developed on [6lbr][6lbr], a [Contiki OS][contiki] based implementation of a deployable Border Router, with several enhancements such as Link-layer security, NAT64, DTLS and others.

The project has a 3-clause BSD license inherited from Contiki.

## Create your own

To build your own weather station the following components are required:

* Two [Zolertia RE-Mote platform][RE-Mote]
* One [BeagleBone Black][BBB]
* At least two microUSB cables, one to connect a RE-Mote to the BeagleBone, another to program the devices
* A [weather meter][weather] from Sparkfun
* A [SHT21 temperature and humidity sensor][sht21] from Sensirion
* A [BMP180 atmospheric pressure sensor][bmp180] from Bosch
* Optionally a LiPo rechargeable battery
* Optionally a solar panel

The connections are as follow:

![](https://github.com/contiki-os/contiki/raw/master/platform/zoul/images/remote-pinout-back.png)

* Connect the SHT21 and BMP180 I2C's SDA and SCL to `PC2` and `PC3` pins, power the sensors using the `DGND` and `D3+3` pins.
* Connect the weather meter's `JP7` connector (a RJ11) as shown on the image below, the `ANEMOMETER` pin corresponds to the `PC1` pin and `WINDAD` must be connected to `PA5` (ADC1)
* Connect the weather meter's `JP5` connector (a RJ11) as shown on the image below, the `RAIN` pin corresponds to the `PC0` pin
* Note the weather meter's anemometer and rain pins require a pull-down resistor, and the wind direction pin requires a voltage divider.  Use the suggested values

![](https://raw.githubusercontent.com/mfabregas/6loweather/master/images/weather-meter-pinout.png)


## Download the code

````
sudo apt-get install git 
git clone --recursive https://github.com/mfabregas/6loweather
cd 6loweather
git config --local include.path ../.gitconfig
````

## Flash the compiled images

The next steps describe how to program your devices with the ready to use example, using the public [Mosquitto MQTT broker][mosquitto].

Binaries (precompiled images) are available in the `app/binaries` folder.  A helper script `flash_devices.sh` allows to program the devices, being required only to connect them over USB (python is required).

````
6LoWeather: ./flash_devices.sh [-d <router|6loweather>] [-p <PORT>] [-s <1|0>]
````

* The `-d` arguments selects which type of device to program.  The `router` is the device to be connected to the BeagleBoneBlack and interface with `6lbr` border router.  The `6loweather` will program to a device the weather station application
* The `-p` argument specifies in which USB port the device is connected to, in Linux this will be tipically `/dev/ttyUSB0`, while in windows it will be `COMxx`
* The `-s` argument specifies if using Link-Layer security (LLSEC) or not, `1` to enable and `0` to disable

To find out in which ports the devices are connected (Linux only), run the following script:

````
cd app/binaries
./motelist
````

An example is provided below, given two devices connected to ports `/dev/ttyUSB0` and `/dev/ttyUSB1` ports:

````
cd app/binaries
./flash_devices.sh -d router -p /dev/ttyUSB0 -s 1
./flash_devices.sh -d 6loweather -p /dev/ttyUSB1 -s 1
````

## Contact

The `6loweather` application is maintained by Marc Fabregas <mfabregas@zolertia.com>

[RE-Mote]: http://www.zolertia.io/products "Zolertia RE-Mote"
[BBB]: https://beagleboard.org/black "BeagleBone Black"
[6lbr]: http://cetic.github.io/6lbr/ "6lbr"
[contiki]: http://www.contiki-os.org/ "Contiki"
[mosquitto]: https://mosquitto.org/ "Mosquitto"
[weather]: https://www.sparkfun.com/products/8942 "Weather Meter"
[sht21]: http://zolertia.io/product/sensors/temp-humidity-sensor-sht21 "SHT21"
[bmp180]: http://zolertia.io/product/barometer-sensor "BMP180"
