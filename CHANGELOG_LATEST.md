# Changelog

### Added
- Boiler WB Greenstar 8000

### Fixed
- telegrams matched to masterthermostat 0x18
- Boiler Junkers Cerapur Aero
- multible roomcontrollers
- readback after write with delay (give ems-devices time to set the value)

### Changed
- split `show values` in smaller packages and separate heating circuits
- extended length of IP/hostname from 32 to 48 chars (#676)
- check flowsensor for `tap_water_active`
- serviceCode `~H` instead of nonacii 3-dashes
- mqtt `base` instead of hostname
- mqtt don't store full topics, add base on publish to save memory
- mqtt max. topic length fixed to 128
- thermostat `time` to `datetime`
- adc scaled to mV output
- split boiler mqtt to `data`, `data_ww`, `data_info`

### Removed
