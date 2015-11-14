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
#ifndef _onimeshfunctions
#define _onimeshfunctions

#include <fstream>
#include <OpenNI.h>
#include "common.h"

namespace onimesh
{
	/// <summary>
	/// Creates a name for an output file
	/// </summary>
	std::string getOutputFileName(const char* outputDirectory, const char* inputFile, const char* fileExtension);

	/// <summary>
	/// Exports a depth frame to the excel data file
	/// </summary>
	void outputFrameToCsv(std::ofstream& outFileStream, openni::VideoFrameRef frameReference);

	/// <summary>
	/// Reads oni input and exports data as excel docs and point clouds
	/// </summary>
	void outputOniData(const int argc, const char** argv);

	/// <summary>
	/// Creates an output file path
	/// </summary>     
	std::string getOutputFilePath(const char* outputDirectory, const char* inputFile);

	/// <summary>
	/// Takes a cloud object and writes it to a PCD file.
	/// </summary>
	void cloud_cb(const CloudConstPtr& cloud);

	/// <summary>
	/// Reads oni input and exports data as point clouds
	/// </summary>
	void outputPointCloud(const int argc, const char** argv);

	/// <summary>
	/// This is a dummy example to show the use of writing a pcl file
	/// that contains a 5x2 cloud with random values
	/// Code taken from here:
	/// http://pointclouds.org/documentation/tutorials/writing_pcd.php#writing-pcd
	/// </summary>
	void createDummyPointCloud();

	/// <summary>
	/// Creates a static meshes from point cloud data files
	/// </summary>  
	void staticMesh(std::string pcdFileName);
}

// End Macroguard
#endif
