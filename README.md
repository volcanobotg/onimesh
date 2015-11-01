# onimesh

## Prerequisites
The following prerequisites must be installed before running or compiling onimesh.

#### OSX
###### OpenNI2 SDK (version 2.2 beta currently)
1. Go to the website http://structure.io/openni
2. Download the zipped OSX file
3. Double click the downloaded file to unzip the folder
4. Move the folder to the desired install directory ("/Applications" suggested for minimal build configuration, if installing to a different location use your selected install path in place of "/Applications" in the commands below)
5. Open a Terminal window
6. Enter the command `sudo /Applications/OpenNI-MacOSX-x64-2.2/install.sh`
7. Enter your admin password when prompted to install OpenNI2

###### Java JDK (required for Homebrew only)
1. Navigate to the website http://www.oracle.com/technetwork/java/javase/downloads/index.html
2. Select the Download Java JDK button
3. Download a JDK for Mac (ex. jdk-8u65-macosx-x64.dmg)
4. Install the downloaded file

###### Point Cloud Library and PCL Dependencies (version 1.7.2 currently)
1. Open a Terminal window
2. Install Homebrew with the command `ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"`
3. Press enter when prompted
4. Enter your admin password when prompted
5. Enter the command `brew update`
6. Enter the command `brew tap homebrew/science`
7. Enter the command `brew install pcl` (this can take a long time to complete)

#### Windows
###### OpenNI2 SDK (version 2.2 beta currently)
1. Navigate to the website http://structure.io/openni
2. Download both x86 and x64 Windows installers
3. Install both x86 and x64 installers

###### Visual C++ Redistributable for Visual Studio 2015
1. Navigate to the website https://www.microsoft.com/en-us/download/details.aspx?id=48145
2. Download both x86 and x64 Windows installers
3. Install both x86 and x64 installers

###### Point Cloud Library and PCL Dependencies (version 1.7.2 currently)
1. Navigate to the website https://drive.google.com/folderview?id=0B0QLyytR4adsOGV0Q19OanpDM00&usp=sharing
2. Download the zip file according to your needs

	`OmDependenciesMinimal.zip` - If you only need to run onimesh (requires ~1.5GB disk space)
	
	`OmDependenciesWithPdb.zip` - If you plan on compiling onimesh (requires ~11.5GB disk space)

3. Unzip the contents of the zip file to the root directory of the C drive so that you have the folder structure C:\OmDependencies\msvc_2015_x86\...
4. Add the following system environment variables

    | Variable Name  | Variable Value |
    | ------------- | ------------- |
    | OM_DEPENDENCIES_ROOT  | C:\OmDependencies\msvc_2015_x86 |
    | OM_DEPENDENCIES_ROOT64  | C:\OmDependencies\msvc_2015_x64 |

5. Add the following to the end of your system "Path" variable:

	`;%OM_DEPENDENCIES_ROOT%\PCL\bin;%OM_DEPENDENCIES_ROOT%\PCL\3rdParty\FLANN\bin;%OM_DEPENDENCIES_ROOT%\PCL\3rdParty\VTK\bin;%OM_DEPENDENCIES_ROOT64%\PCL\bin;%OM_DEPENDENCIES_ROOT64%\PCL\3rdParty\FLANN\bin;%OM_DEPENDENCIES_ROOT64%\PCL\3rdParty\VTK\bin`

## Building Binaries
onimesh is built with the following dependencies
* OpenNI2 2.2
* Point Cloud Library 1.7.2
  * Boost 1.57.0
  * Eigen 3.2.4
  * FLANN 1.8.4
  * QHull 2012.1
  * VTK 6.3.0
    * Qt 5.5.1

#### OSX
1. Install all OSX prerequisites above
2. Clone the onimesh repository to a local drive
3. If you didn't install OpenNI2 to "/Applications" you will need to edit the makefile lines below with your install path:

    `OPENNI2_INCLUDE=/OpenNI2InstallPath/Include`
	
    `OPENNI2_DLIBRARY=/OpenNI2InstallPath/Redist/libOpenNI2.dylib`

4. Open a Terminal window
5. In Terminal, navigate to the root of the repository
6. Enter the command `make`
7. Output binary is located in the "bin" directory

#### Windows
1. Install all Windows prerequisites above being sure to select the OmDependenciesWithPdb.zip
2. Clone the onimesh repository to a local drive
3. Using Visual Studio 2015 open "onimesh.sln" in the root of the repository
4. Select the "Build" button
5. Output binary is located in the target directory

## Usage
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

