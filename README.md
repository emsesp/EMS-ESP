# IRT-ESP
IRT-ESP is a project to build an electronic controller circuit using an Espressif ESP8266 micro controller to communicate with iRT based Boilers from the Nefit Ecomline HR Classic range and compatibles such as Buderus.

This project is a fork of the [Proddy EMS-ESP](https://github.com/proddy/EMS-ESP) project.

It can operate in two modes: active and passive. In passive mode it just decodes iRT messages and report the status via MQTT or the build-in web interface. In active mode it can maintain a set water temperature, but this is a work in progress. Currently the only supported hardware is a modified [EMS Bus gateway](https://bbqkees-electronics.nl/) from BBQKees. Without the hardware modification it will not work !

For more information about the iRT protocol have a look at the [wiki](https://github.com/Victor-Mo/IRT-EMS-ESP/wiki).

## Supported features
In passive mode the firmware can decode the water temperature, if the boiler is running, what mode: heating or warm water, and if the pump is running.

In the (very experimental) active mode it will start the burner and sets a max water temp. The burner power is a fixed value depended on the water temperature you set, '`set_water 35`' will set a maximum water temperature of 35 degrees Celsius and a very low burner power. '`set_water 90`' will run the boiler at full power and a maximum water temperature of 90 degrees Celsius.

Please keep in mind this software is highly experimental. It works on my boiler, it may not work on yours. The active mode can seriously damage your boiler, for example running it at full power for extended time. Always monitor your boiler when running in active mode !

But if it does not work, create a log file (`log j`), create a ticket, and I will see what I can do.

## Building the software

For building the software, have a look at the [EMS-ESP wiki](https://emsesp.github.io/docs/#/Building-firmware).

## First start, passive mode
Because the software is kept compatible with the EMS-ESP, the TX-Mode needs to be set on the first run. Telnet to the device and issue the following commands:

`set tx_mode 4`

`restart`

The device will now restart, if the device is back. Reconnect and enable logging (`log j`). The output should look like this:
```
log j

System Logging set to Jabber mode
(00:56:43.985) irt_parseTelegram: 00 05 35: 01 01 FE 90 90 E6 E6 D0 D0 2E 2E CF 30 82 82 6B 6B B8 B8 86 86 00 FF A3 A3 07 07 D3 D3 CA CA 03 FC A4 A4 7D 7D AC AC DC DC 2A D5 8A 8A A1 A1 98 98 83 83 FE 01 
(00:56:48.985) irt_parseTelegram: 00 05 35: 01 01 FE 90 90 40 40 5A 5A 61 61 CF 30 93 93 73 73 0E 0E 4D 4D FF 00 C9 C9 C3 C3 79 79 AB AB 00 FF F0 F0 01 01 CD CD ED ED 00 FF F0 F0 01 01 D8 D8 B9 B9 05 FA 
(00:56:54.025) irt_parseTelegram: 00 05 35: 01 01 FE 90 90 C3 C3 79 79 F2 F2 CF 30 82 82 C3 C3 79 79 E0 E0 00 FF A3 A3 C3 C3 79 79 C1 C1 03 FC A4 A4 C3 C3 79 79 C6 C6 2A D5 8A 8A C3 C3 79 79 E8 E8 FE 01 
(00:56:59.065) irt_parseTelegram: 00 05 35: 01 01 FE 90 90 C3 C3 79 79 F2 F2 CF 30 81 81 C3 C3 79 79 E3 E3 51 AE 86 86 C3 C3 79 79 E4 E4 E9 16 85 85 C3 C3 79 79 E7 E7 74 8B 83 83 C3 C3 79 79 E1 E1 A0 5F 
(00:57:03.625) irt_parseTelegram: 00 05 35: 01 01 FE 90 90 C3 C3 79 79 F2 F2 CF 30 82 82 C3 C3 79 79 E0 E0 00 FF A3 A3 C3 C3 79 79 C1 C1 03 FC A4 A4 C3 C3 79 79 C6 C6 2A D5 8A 8A C3 C3 79 79 E8 E8 FE 01 
```
## First start, active mode
In order to work in active mode the IRT-EMS-ESP should be the only device on the bus ! Telnet to the device and issue the following commands:

`set tx_mode 5`

`restart`

The device will now restart, if the device is back. Reconnect and enable logging (`log j`). The output should look like this:
```
log j

System Logging set to Jabber mode
(00:00:14.769) irt_rawTelegram: 00 05 01: 00  - "."
(00:00:14.869) irt_rawTelegram: 00 05 01: 01  - "."
(00:00:14.990) irt_rawTelegram: 00 05 01: 02  - "."
(00:00:15.110) irt_rawTelegram: 00 05 01: 03  - "."
(00:00:15.230) irt_rawTelegram: 00 05 01: 00  - "."
(00:00:15.288) irt_tx: 01 05 1E: 90 A5 F0 28 00 00 82 A5 F0 3A 00 00 A3 A5 F0 1B 00 00 A4 A5 F0 1C 00 00 8A A5 F0 32 00 00
(00:00:15.330) irt_rawTelegram: 00 05 01: 01  - "."
(00:00:15.450) irt_rawTelegram: 00 05 01: 02  - "."
(00:00:15.570) irt_rawTelegram: 00 05 01: 03  - "."
(00:00:15.670) irt_rawTelegram: 00 05 01: 00  - "."
set(00:00:16.571) irt_rawTelegram: 00 05 35: 01 01 FE 90 90 A5 A5 F0 F0 28 28 67 98 82 82 A5 A5 F0 F0 3A 3A 00 FF A3 A3 A5 A5 F0 F0 1B 1B 03 FC A4 A4 A5 A5 F0 F0 1C 1C 20 DF 8A 8A A5 A5 F0 F0 32 32 FE 01
| 81:35 82:00 83:A0 85:74 86:E9 8A:FE 90:67 A3:03 A4:20   0H

```

Set the desired water temerature (`set_water 30`) and change the logging to only show the status line (`log s`):

```
log s

System Logging set to Solar Module only
01:35 04:00 05:04 07:00 11:FF 14:00 15:04 17:FF 73:52 78:05 | 81:35 82:00 83:A0 85:74 86:E9 8A:FE 90:67 93:00 A3:03 A4:1F A6:1E C9:05 E8:05 ED:00   0H
Starting scheduled query from EMS devices
01:35 04:00 05:04 07:00 11:FF 14:00 15:04 17:FF 73:52 78:01 | 81:35 82:00 83:A0 85:74 86:E9 8A:FE 90:67 93:00 A3:03 A4:1F A6:1E C9:05 E8:05 ED:00   0H

set_water 30

Irt set water temp, wc 1

Water temp set to 30 (1e)
01:35 04:00 05:04 07:69 11:FF 14:00 15:04 17:FF 73:52 78:07 | 81:35 82:00 83:A0 85:74 86:E9 8A:FE 90:67 93:00 A3:03 A4:1F A6:1E C9:05 E8:05 ED:00   0H
01:23 04:00 05:04 07:75 11:FF 14:00 15:04 17:FF 73:52 78:05 | 81:35 82:84 83:00 85:04 86:E9 8A:FE 90:67 93:00 A3:53 A4:24 A6:1D C9:02 E8:05 ED:00   -H
01:23 04:00 05:04 07:75 11:FF 14:00 15:04 17:FF 73:52 78:05 | 81:35 82:84 83:00 85:04 86:E9 8A:FE 90:67 93:00 A3:53 A4:24 A6:1D C9:01 E8:05 ED:00   -H

```


## Hardware change needed

The software runs on a modified [EMS Bus Wi-Fi Gateway](https://bbqkees-electronics.nl/product/gateway-premium-ii/). In short; Remove the 100 KOhm resistor (R5) to ground en replace it with a 1.5 MOhm resistor to 5 Volt. **Without the modification it will not work !**

The 1.5 MOhm resistor works for my setup where the Thermostat is 5 meters a way from the boiler. This value may have to be adapted for different distances.

I adapted BBQKees's schematic to show the change: ![Modified Schematic](doc/schematics/IRT-V09_schema.png)

BBQKees also sells an already modified [interface board](https://bbqkees-electronics.nl/product/irt-interface-board-experimental/).
