#!/usr/bin/python3

# Author: Hyungsub Kim
# Email: kim2956@purdue.edu
# This is a XML parser for ArduPilot.
# It parses the following properties from xml files:
#   (1) parameter name
#   (2) description
#   (3) valid range
#   (4) increment unit
#   (5) Read-only or not

# Usage: xml_parse_ardupilot.py -i <inputfile> -o <outputfile>
# (e.g.,) python xml_parse_ardupilot.py -i ./apm_pdef_copter.xml -o ./output_copter.csv
# (e.g.,) python xml_parse_ardupilot.py -i ./apm_pdef_plane.xml -o ./output_plane.csv
# (e.g.,) python xml_parse_ardupilot.py -i ./apm_pdef_rover.xml -o ./output_rover.csv
# (e.g.,) python xml_parse_ardupilot.py -i ./apm_pdef_submarine.xml -o ./output_submarine.csv
# (e.g.,) python xml_parse_ardupilot.py -i ./apm_pdef_antenna_tracker.xml -o ./output_antenna_tracker.csv
#
# Result:
# [Name;;Description;;Range;;Increment;;Value;;Read_only]
# (e.g.,) Value = 1|2|3|4

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
        print("xml_parse_ardupilot.py -i <inputfile> -o <outputfile>")
        sys.exit(2)
    
    for opt, arg in opts:
        if opt == '-h':
            print("xml_parse_ardupilot.py -i <inputfile> -o <outputfile>")
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
        fields = ['Name', 'Description', 'Range', 'Increment', 'Value', 'Read_only'] 
        csv_writer.writerow(fields) 

    else:
        print("The output file can be either \'.txt\' or \'.csv\'.")
        sys.exit(2)



    doc = ET.parse(inputfile)

    # Load root node
    root = doc.getroot()

    count = 1

    for iteration in range (2):
    
        if iteration == 0:
            param_type = root.find('vehicles')

        elif iteration == 1:
            param_type = root.find('libraries')


        #parameters = param_type.find('parameters')
    
        for parameters in param_type.iter("parameters"):

    
            for object in parameters.iter("param"):
                #print(object.findtext("field"))
                Name = object.get("name")
                Description = object.get("documentation")
                Range = ""
                Increment = ""
                Value = ""
                Read_only = ""

                for field in object.iter("field"):
                    if field.get("name") == "Range":
                        Range = field.text
                    elif field.get("name") == "Increment":
                        Increment = field.text
                    elif field.get("name") == "ReadOnly":
                        Read_only = "True"


                values = object.find('values')
                if values != None:
                    for v in values.iter("value"):
                        Value += v.get("code") + '|'

                print("%d, parameter name: %r, description: %r, range: %r, increment: %r, value: %r, read-only?: %r" % (count, Name, Description, Range, Increment, Value, Read_only))
                count = count +1
            
                if Name == None:
                    Name = ""
                else:
                    # Let's get rid of "ArduCopter:".
                    Name = re.sub('ArduCopter:', '', Name)
                    Name = re.sub('ArduSub:', '', Name)
                    Name = re.sub('Rover:', '', Name)
                    Name = re.sub('ArduPlane:', '', Name)
                    Name = re.sub('AntennaTracker:', '', Name)

                if Description == None:
                    Description = ""
                
                if output_file_type == -1:
                    write = ";;".join([Name, Description, Range, Increment, Value, Read_only, "\n"])
                    store_file.write(write)

                elif output_file_type == 1:
                    # writing the data rows 
                    write = ";;".join([Name, Description, Range, Increment, Value, Read_only])
                    row = write.split(';;')
                    csv_writer.writerow(row)

                print("[Name, Description, Range, Increment, Value, Read_only]")


if __name__ == "__main__":
   main(sys.argv[1:])
