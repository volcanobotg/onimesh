# onimesh

### Prerequisites
##### OSX
###### OpenNI2 SDK (version 2.2 beta currently) - http://structure.io/openni
1. Download the zipped OSX file
2. Double click the downloaded file to unzip the folder
3. Move the folder to the desired install directory ("/Applications" suggested for minimal build configuration, if installing to a different location use your selected install path in place of "/Applications" in the commands below)
4. Open a Terminal window
5. Enter the command `sudo /Applications/OpenNI-MacOSX-x64-2.2/install.sh`
6. Enter your admin password when prompted to install OpenNI2

###### Java JDK (required for MacPorts only) - http://www.oracle.com/technetwork/java/javase/downloads/index.html
1. Navigate to the Java website
2. Select the Download Java JDK button
3. Download a JDK for Mac (ex. jdk-8u65-macosx-x64.dmg)
4. Install the downloaded file

###### Point Cloud Library and PCL Dependencies (version 1.7.2 currently) - http://pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php
1. Download the appropriate version of MacPorts for your version of OSX here https://distfiles.macports.org/MacPorts/
2. Install MacPorts with the default installer settings
3. Open a Terminal window, enter your admin password when prompted
4. Install XCode Command Line Tools with the command `xcode-select --install` and select Install from the popup
5. Install CMake with the command `sudo port install cmake`
6. Install Boost with the command `sudo port install boost`
7. Install Eigen with the command `sudo port install eigen3`
8. Install FLANN with the command `sudo port install flann`
9. Install VTK with the command `sudo port install vtk5 +qt4_mac`
10. Install Point Cloud Library with the command `sudo port install libpcl`

##### Windows
###### OpenNI2 SDK (version 2.2 beta currently) - http://structure.io/openni
1. Download both x86 and x64 Windows installers
2. Install both x86 and x64 installers

###### Visual C++ Redistributable for Visual Studio 2015 - https://www.microsoft.com/en-us/download/details.aspx?id=48145
1. Download both x86 and x64 Windows installers
2. Install both x86 and x64 installers

###### Point Cloud Library (version 1.7.2 currently) - http://pointclouds.org/
1. Navigate to this website https://onedrive.live.com/redir?resid=EC9EBB2646FF189A!51248&authkey=!AOPBX-WypndUncw&ithint=file%2cexe
2. Download the files "PCL-1.7.2-AllInOne-msvc2015-win64" and "PCL-1.7.2-AllInOne-msvc2015-win32"
3. Install both of the downloaded files (these files contain PCL and all PCL dependencies)

### Building Binaries
##### OSX
1. Install all OSX prerequisites above
2. Clone the onimesh repository to a local drive
3. If you didn't install OpenNI2 to "/Applications" you will need to edit the makefile lines below with your install path:

    `OPENNI2_INCLUDE=/OpenNI2InstallPath/Include`
    `OPENNI2_DLIBRARY=/OpenNI2InstallPath/Redist/libOpenNI2.dylib`

4. Open a Terminal window
5. In Terminal, navigate to the root of the repository
6. Enter the command `make`
7. Output binary is located in the "bin" directory

##### Windows
1. Install all Windows prerequisites above
2. Add the following system environment variables

    | Variable Name  | Variable Value |
    | ------------- | ------------- |
    | PCL_ROOT  | C:\Program Files (x86)\PCL 1.7.2 |
    | PCL_ROOT64  | C:\Program Files\PCL 1.7.2 |

3. Add the following to the end of your "Path" variable:

    `;%PCL_ROOT%\bin;%PCL_ROOT%\3rdParty\FLANN\bin;%PCL_ROOT%\3rdParty\VTK\bin;%PCL_ROOT64%\bin;%PCL_ROOT64%\3rdParty\FLANN\bin;%PCL_ROOT64%\3rdParty\VTK\bin`

4. Clone the onimesh repository to a local drive
5. Using Visual Studio 2015 open "onimesh.sln" in the root of the repository
6. Select the "Build" button
7. Output binary is located in the target directory

### Usage
`onimesh <outputDir> <OniInputFiles>`

    outputDir - Output directory for data files

    oniInputFiles - One or more .oni input files

##### Important Usage Notes:
* Directory paths may contain '\' or '/' characters
* At least one input file is required. Multiple input files need to be space delimited
  * Example: `onimesh c:\onimeshOutputDir\ c:\onimeshInputFiles\inputFile1.oni c:\onimeshInputFiles\inputFile2.oni`
* Output Directory and Input files may contain relative paths
  * Example: `onimesh ..\onimeshOutputDir\ inputFile1.oni ..\inputFile2.oni ..\inputDirectory2\inputFile3.oni`
* File and directory paths that contain a ' ' space character must be enclosed in double quotation marks
  * Example: `onimesh "c:\onimesh Output Dir\" "..\onimesh Input Files\input file 1.oni"`

