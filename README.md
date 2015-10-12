# onimesh

## Prerequisites
* OpenNI2 SDK - http://structure.io/openni

## Usage
onimesh <outputDir> <OniInputFiles>
      outputDir - Output directory for data files
  oniInputFiles - One or more .oni input files

### Important Usage Notes:
* Directory paths must end with a '\' character, failure to do so will result in erroneous output file names
* At least one input file is required. Multiple input files need to be space delimited
** Example: onimesh c:\onimeshOutputDir\ c:\onimeshInputFiles\inputFile1.oni c:\onimeshInputFiles\inputFile2.oni
* Output Directory and Input files may contain relative paths
** Example: onimesh ..\onimeshOutputDir\ inputFile1.oni ..\inputFile2.oni ..\inputDirectory2\inputFile3.oni
* File and directory paths that contain a ' ' space character must be enclosed in double quotation marks
** Example: onimesh "c:\onimesh Output Dir\" "..\onimesh Input Files\input file 1.oni"

