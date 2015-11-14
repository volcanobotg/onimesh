/*
* Copyright (c) 2009-2012, Willow Garage, Inc.
* Copyright (c) 2012-, Open Perception, Inc.
* Copyright (c) 2015-, Christopher Padgett, David Andrews, Luis Henrique Fae Barboza
*
* You may use, distribute and modify this code under the terms of the BSD license.
*
* You should have received a copy of the full BSD license with this file.
* If not, please visit: https://github.com/volcanobotg/onimesh for full license information.
*/

// Macroguard
#ifndef _omfilesystemhelper
#define _omfilesystemhelper

#include <iostream>

namespace onimesh
{
	/// <summary>
	/// Creates a name for an output file
	/// </summary>
	std::string getOutputFileName(const char* outputDirectory, const char* inputFile, const char* fileExtension);

	/// <summary>
	/// Creates a directory
	/// </summary>
	bool createDirectory(const std::string directoryPath);

	/// <summary>
	/// Deletes a directory and all its contents if it exists
	/// </summary>
	bool deleteDirectory(const std::string directoryPath);
}

// End Macroguard
#endif
