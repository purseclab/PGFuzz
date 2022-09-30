#!/usr/bin/python3

# Author: Hyungsub Kim
# Email: kim2956@purdue.edu
# This is a XML parser for PX4.
# It parses the following properties from XML files:
#   (1) parameter name
#   (2) description
#   (3) valid range
#   (4) increment unit
#   (5) Read-only or not

# Usage: xml_parse_ardupilot.py -i <inputfile> -o <outputfile>
# (e.g.,) python xml_parse_px4.py -i ./parameters_copter.xml -o ./output_copter.csv
#
# Result:
# [Name;;Description;;Range;;Default]
# (e.g.,) MPC_THR_XY_MARG;;Margin that is kept for horizontal control when prioritizing vertical thrust. To avoid completely starving horizontal control with high vertical error;;0.0 0.5;;0.3
#
# In some cases, the XML file mentions only 'min' or 'max'.
# So, 'min:x' represents the XML file only mentions the minimum parameter value.
# On the other hand, 'max:y' expresses the XML file only include the maximum parameter value.
#
# How to get such XML file from PX4?
# [PX4_root_directory] make parameters_metadata
# [PX4_root_directory] make px4_sitl gazebo
# Then, you can find 'parameters.xml' in [PX4_root_directory]

import sys, getopt
import csv
import re
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element, dump, ElementTree

def main(argv):
    # Parse command line arguments (i.e., input and output file)
    
    output_file_type = 0 # -1: txt, 1: csv
    inputfile = ''
    outputfile = ''

    try:
        opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])

    except getopt.GetoptError:
        print("xml_parse_px4.py -i <inputfile> -o <outputfile>")
        sys.exit(2)
    
    for opt, arg in opts:
        if opt == '-h':
            print("xml_parse_px4.py -i <inputfile> -o <outputfile>")
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg
    print('Input file is "', inputfile)
    print('Output file is "', outputfile)


    if re.search(".txt", outputfile):
        output_file_type = -1
        
        store_file = open(outputfile, 'w')
        store_file.close()

        store_file = open(outputfile, 'a')

    elif re.search(".csv", outputfile):
        output_file_type = 1
        store_file = open(outputfile, 'w')
        store_file.close()

        store_file = open(outputfile, 'a')
        # creating a csv writer object
        csv_writer = csv.writer(store_file) 
        
        # writing the fields 
        fields = ['Name', 'Description', 'Range', 'Default'] 
        csv_writer.writerow(fields) 

    else:
        print("The output file can be either \'.txt\' or \'.csv\'.")
        sys.exit(2)



    doc = ET.parse(inputfile)

    # Load root node
    root = doc.getroot()

    count = 1      
 
    for parameters in root.iter("group"):
    
        for object in parameters.iter("parameter"):
            
            Name = ""
            Description = ""
            Min = ""
            Max = ""
            Range = ""
            Default = ""

            Name = object.get("name")

            if object.find("long_desc") != None:
                Description = object.find("long_desc").text
            else:
                Description = object.find("short_desc").text

            # In some cases, the XML file mentions only 'min' or 'max'.
            min_exist = 0
            max_exist = 0

            if object.find("min") != None:
                Min = object.find("min").text
                min_exist = 1

            if object.find("max") != None:
                Max = object.find("max").text
                max_exist = 1

            if min_exist == 1 and max_exist == 1:
                Range = Min + " " + Max
            elif min_exist == 1 and max_exist == 0:
                Range = "min:" + Min
            elif min_exist == 0 and max_exist == 1:
                Range = "max:" + Max

            Default = object.get("default")
            
            print("%d, parameter name: %r, description: %r, range: %r, default: %r" % (count, Name, Description, Range, Default))
            
            count = count +1
                
            if output_file_type == -1:
                write = ";;".join([Name, Description, Range, Default, "\n"]).encode('utf-8')
                store_file.write(write)

            elif output_file_type == 1:
                # writing the data rows 
                write = ";;".join([Name, Description, Range, Default]).encode('utf-8')
                row = write.split(';;')
                csv_writer.writerow(row)

            print("[Name, Description, Range, Default]")


if __name__ == "__main__":
   main(sys.argv[1:])
