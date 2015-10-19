# onimesh

### Prerequisites
##### OSX
* OpenNI2 SDK (version 2.2 beta currently) - http://structure.io/openni
  1. Download the zipped OSX file
  2. Double click the downloaded file to unzip the folder
  3. Move the folder to the desired install directory ("/Applications" suggested for minimal build configuration, if installing to a different location use your selected install path in place of "/Applications" in the commands below)
  4. Open a Terminal window
  5. Enter the command `sudo /Applications/OpenNI-MacOSX-x64-2.2/install.sh`
  6. Enter your admin password when prompted to install OpenNI2

##### Windows
* OpenNI2 SDK (version 2.2 beta currently) - http://structure.io/openni
  1. Download both x86 and x64 Windows installers
  2. Double click the downloaded x86 installer
  3. Follow the on screen instructions to install the x86 version
  4. Double click the downloaded x64 installer
  5. Follow the on screen instructions to install the x64 version
* Visual C++ Redistributable for Visual Studio 2015 - https://www.microsoft.com/en-us/download/details.aspx?id=48145
  1.  Download both x86 and x64 Windows installers
  2. Double click the downloaded x86 installer
  3. Follow the on screen instructions to install the x86 version
  4. Double click the downloaded x64 installer
  5. Follow the on screen instructions to install the x64 version

### Building Binaries
##### OSX
1. Clone the onimesh repository to a local drive
2. If you didn't install OpenNI2 to "/Applications" you will need to edit the makefile lines below with your install path:
`OPENNI2_INCLUDE=/OpenNI2InstallPath/Include`
`OPENNI2_DLIBRARY=/OpenNI2InstallPath/Redist/libOpenNI2.dylib`
3. Open a Terminal window
4. In Terminal, navigate to the root of the repository
5. Enter the command `make`
6. Output binary is located in the "bin" directory

##### Windows
1. Clone the onimesh repository to a local drive
2. Using Visual Studio 2015 open "onimesh.sln" in the root of the repository
3. Select the "Build" button
4. Output binary is located in the target directory

### Usage
`onimesh <outputDir> <OniInputFiles>`

    outputDir - Output directory for data files

    oniInputFiles - One or more .oni input files

##### Important Usage Notes:
* Directory paths must end with a '\\' character, failure to do so will result in erroneous output file names
* At least one input file is required. Multiple input files need to be space delimited
  * Example: `onimesh c:\onimeshOutputDir\ c:\onimeshInputFiles\inputFile1.oni c:\onimeshInputFiles\inputFile2.oni`
* Output Directory and Input files may contain relative paths
  * Example: `onimesh ..\onimeshOutputDir\ inputFile1.oni ..\inputFile2.oni ..\inputDirectory2\inputFile3.oni`
* File and directory paths that contain a ' ' space character must be enclosed in double quotation marks
  * Example: `onimesh "c:\onimesh Output Dir\" "..\onimesh Input Files\input file 1.oni"`

