# Changelog

### Added
- Boiler WB Greenstar 8000 (195) and Cascade Modul MC400 (210) 
- optional id to info command to output heatingcircuits separated
- Reset to factory setting with PButton (io0 to GND for >9 sec)

### Fixed
- telegrams matched to masterthermostat 0x18
- Boiler Junkers Cerapur Aero
- multiple roomcontrollers
- readback after write with delay (give ems-devices time to set the value)
- Thermostat ES72/RC20, device 66 to RC20_2 command-set
- recognize sending devices which are not in telegram 0x07
- MQTT payloads not adding to queue when MQTT is re-connecting (fixes #369)
- fix mixerTemp and tankMiddleTemp (PR #714 @joanwa)
- icon update for pumps
- add mixer wiring naming from documentation
- wwtemp feedback and ems+ command
- fix for HA topics with invalid command formats - #728
- fix ems+ values #723, #732

### Changed
- split `show values` in smaller packages and separate heating circuits
- extended length of IP/hostname from 32 to 48 chars (#676)
- check flowsensor for `tap_water_active`
- serviceCode `~H` instead of non-ascii 3-dashes
- mqtt `base` instead of hostname
- mqtt don't store full topics, add base on publish to save memory
- mqtt max. topic length fixed to 128
- thermostat `time` to `datetime`
- adc scaled to mV output
- split boiler mqtt to `data`, `data_ww`, `data_info`
- Reduce Web UI artefact size by removing moment.js 

### Removed
