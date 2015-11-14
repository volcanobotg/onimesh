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

#include <boost/filesystem.hpp>
#include <iostream>
#include "filesystemHelper.h"

namespace onimesh
{
	/// <summary>
	/// Creates a name for an output file
	/// </summary>
	std::string getOutputFileName(const char* outputDirectory, const char* inputFile, const char* fileExtension)
	{
		// If the path contains '/' characters
		if (std::string(inputFile).find_last_of('/') != std::string::npos)
		{
			// Check if the directory needs a trailing '/'
			if (std::string(outputDirectory).back() == '/' || std::string(outputDirectory).back() == '\\')
				return std::string(std::string(outputDirectory) + std::string(inputFile).substr(std::string(inputFile).find_last_of('/') + 1) + std::string(fileExtension));

			return std::string(std::string(outputDirectory) + std::string("/") + std::string(inputFile).substr(std::string(inputFile).find_last_of('/') + 1) + std::string(fileExtension));
		}
		// If the path contains '\' characters
		else if (std::string(inputFile).find_last_of('\\') == std::string::npos)
		{
			// Check if the directory needs a trailing '\'
			if (std::string(outputDirectory).back() == '/' || std::string(outputDirectory).back() == '\\')
				return std::string(std::string(outputDirectory) + std::string(inputFile).substr(std::string(inputFile).find_last_of('\\') + 1) + std::string(fileExtension));

			return std::string(std::string(outputDirectory) + std::string("\\") + std::string(inputFile).substr(std::string(inputFile).find_last_of('\\') + 1) + std::string(fileExtension));
		}

		// Otherwise the input file does not contain a path
		// Check if the directory needs a trailing '/'
		if (std::string(outputDirectory).back() == '/' || std::string(outputDirectory).back() == '\\')
			return std::string(std::string(outputDirectory) + std::string(inputFile) + std::string(fileExtension));

		return std::string(std::string(outputDirectory) + std::string("/") + std::string(inputFile) + std::string(fileExtension));
	}

	/// <summary>
	/// Creates a directory
	/// </summary>
	bool createDirectory(const std::string directoryPath)
	{
		boost::system::error_code returnedError;
		if (!boost::filesystem::exists(directoryPath))
			boost::filesystem::create_directories(directoryPath, returnedError);

		if (returnedError)
		{
			std::cerr << "\nUnable to create directory:" << returnedError.category().name() << '\n' << returnedError.value() << '\n';
			return false;
		}

		return true;
	}

	/// <summary>
	/// Deletes a directory and all its contents if it exists
	/// </summary>
	bool deleteDirectory(const std::string directoryPath)
	{
		boost::system::error_code returnedError;
		if (boost::filesystem::exists(directoryPath))
			boost::filesystem::remove_all(directoryPath, returnedError);

		if (returnedError)
		{
			std::cerr << "\nUnable to delete directory:" << returnedError.category().name() << '\n' << returnedError.value() << '\n';
			return false;
		}

		return true;
	}
}