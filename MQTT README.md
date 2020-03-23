MQTT commands
*************

version: 1.0
last modified: 23/3/2020

topic:                            JSON payload             		Comment: 
*******************************************************************************************************************************
ems-esp/generic_cmd              |                                     |
                                 | cmd: coldshot	               | Let some cold water flow...
                                 |                                     |
ems-esp/shower_data              |                                     |
                                 | alert: 0, 1                         | 0 off, 1 on
                                 | timer: 0, 1	                       | 0 off, 1 on
                                 |                                     |
ems-esp/boiler_cmd               |                                     |
                                 | cmd: comfort                        | set WW to mode
                                 |   data: hot, comfort, intelligent   | 
                                 |                                     |
                                 | cmd: flowtemp                       | set WW temperature to [integer]
                                 |   data: [integer, as string]        | 
                                 |                                     |
ems-esp/boiler_cmd_wwactivated   |                                     |
                                 | 1, on, 0, off                       | set WWActivated
                                 |                                     |
ems-esp/boiler_cmd_wwonetime     |                                     |
                                 | 1, on, 0, off                       | set WWOneTime
                                 |                                     |
ems-esp/boiler_cmd_wwcirculation |                                     |
                                 | 1, on, 0, off                       | set WWCirculation
                                 |                                     |
ems-esp/boiler_cmd_wwtemp        |                                     |
                                 | [integer, as string]                | set WW temperature 
                                 |                                     |
ems-esp/settings_cmd             |                                     |
                                 | cmd: language                       | set language
                                 |   data: french, german, italian     | 
                                 |                                     |
                                 | cmd: building                       | set building type (determine heating curve)
                                 |   data: light, medium, heavy        | 
                                 |                                     |
                                 | cmd: display                        | set display on thermostat
                                 |   data: [integer, as string]        | 
                                 |                                     |
ems-esp/thermostat_cmd_temp[1-4] |                                     |
                                 | [float, as string]                  | set temperature setpoint for HC[1-4] (mode auto?)
                                 |                                     |
ems-esp/thermostat_cmd_mode      |                                     |
                                 | [string: auto, day, manual, heat,   |
                                 |  night, off, comfort, holiday,      | 
                                 |  nofrost, eco]                      | set thermostat mode
                                 |                                     |
ems-esp/thermostat_cmd           |                                     |
                                 | cmd: temp                           | same as 'thermostat_cmd_temp[1-4]'
                                 |   data: [float, as string]          | 
                                 |                                     |
                                 | cmd: mode                           | same as 'thermostat_cmd_mode'
                                 |   data: [string]                    | 
                                 |                                     |
                                 | cmd: daytemp[1-4]                   | set temp. setpoint for day mode
                                 |   data: [float, as string]          |
                                 |                                     |
                                 | cmd: nightemp[1-4]                  | set temp. setpoint for night mode
                                 |   data: [float, as string]          |
                                 |                                     |
                                 | cmd: holidaytemp[1-4]               | set temp. setpoint for holiday mode
                                 |   data: [float, as string]          |
                                 |                                     |
                                 | cmd: ecotemp[1-4]                   | set temp. setpoint for eco mode
                                 |   data: [float, as string]          |
                                 |                                     |
                                 | cmd: heattemp[1-4]                  | set temp. setpoint for heating
                                 |   data: [float, as string]          |
                                 |                                     |
                                 | cmd: nofrosttemp[1-4]               | set temp. setpoint for no frost protection
                                 |   data: [float, as string]          |
                                 |                                     |
*******************************************************************************************************************************

Example: 
  mosquitto_pub -u username -P mypass -t prefix/ems-esp/settings_cmd -m "{\"cmd\":\"building\",\"data\":\"heavy\"}"
