# Purpose of XML parser

In order to mutate configuration parameters, we need to firstly know what are the valid ranges of the parameters.

This is a XML parser for ArduPilot. <br>
It parses the following properties from xml files: <br>
&emsp;&emsp;(1) Parameter name <br>
&emsp;&emsp;(2) Description <br>
&emsp;&emsp;(3) Valid range <br>
&emsp;&emsp;(4) Increment unit <br>
&emsp;&emsp;(5) Read-only or not <br>

# How to execute it?
```bash
python xml_parse_ardupilot.py -i ./apm_pdef_copter.xml -o ./output_copter.csv
python xml_parse_ardupilot.py -i ./apm_pdef_plane.xml -o ./output_plane.csv
python xml_parse_ardupilot.py -i ./apm_pdef_rover.xml -o ./output_rover.csv
python xml_parse_ardupilot.py -i ./apm_pdef_submarine.xml -o ./output_submarine.csv
python xml_parse_ardupilot.py -i ./apm_pdef_antenna_tracker.xml -o ./output_antenna_tracker.csv
```
The output file can be either '.txt' or '.csv'.

# Execution results
```bash
[Name;;Description;;Range;;Increment;;Value;;Read_only]
(e.g.,) RTL_ALT_FINAL;;This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission. Set to zero to land.;;0 1000;;1;;;;
```
Here, the separator of fields is ';;' and the separator of 'Value' field is '|', e.g., 1|2|3|4.
