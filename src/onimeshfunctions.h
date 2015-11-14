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

namespace onimesh
{
	/// <summary>
	/// Exports a depth frame to the excel data file
	/// </summary>
	void outputFrameToCsv(std::ofstream& outFileStream, const openni::VideoFrameRef frameReference);

	/// <summary>
	/// Exports a depth frame to the excel data file
	/// </summary>
	void outputFrameToPcd(const std::string outputFrameDirectory, const openni::Device* device, const openni::VideoFrameRef frameReference);

	/// <summary>
	/// Reads oni input and exports data as excel docs and point clouds
	/// </summary>
	void outputOniData(const int argc, const char** argv);

	/// <summary>
	/// Creates a static meshes from point cloud data files
	/// </summary>  
	void staticMesh(std::string pcdFileName);
}

// End Macroguard
#endif
