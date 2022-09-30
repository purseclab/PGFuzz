# Purpose of XML parser

In order to mutate configuration parameters, we need to firstly know what are the valid ranges of the parameters.

This is a XML parser for PX4. <br>
It parses the following properties from XML files: <br>
&emsp;&emsp;(1) Parameter name <br>
&emsp;&emsp;(2) Description <br>
&emsp;&emsp;(3) Valid range <br>
&emsp;&emsp;(4) Increment unit <br>
&emsp;&emsp;(5) Read-only or not <br>

# Where can we get such XML files?
```bash
[PX4_root_directory] make parameters_metadata
[PX4_root_directory] make px4_sitl gazebo
```
Then, you can find 'parameters.xml' in [PX4_root_directory]

# How to execute it?
```bash
python xml_parse_px4.py -i ./parameters_copter.xml -o ./output_copter.csv
```
The output file can be either '.txt' or '.csv'.

# Execution results
```bash
[Name;;Description;;Range;;Default]
(e.g.,) MPC_THR_XY_MARG;;Margin that is kept for horizontal control when prioritizing vertical thrust. To avoid completely starving horizontal control with high vertical error;;0.0 0.5;;0.3
```
In some cases, the XML file mentions only 'min' or 'max'.<br>
So, 'min:x' represents the XML file only mentions the minimum parameter value.<br>
On the other hand, 'max:y' expresses the XML file only include the maximum parameter value.<br>
