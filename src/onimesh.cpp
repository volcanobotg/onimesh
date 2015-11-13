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

#include <iostream>
#include "onimeshfunctions.h"

int main(const int argc, const char** argv)
{
	// Argument check
	if (argc < 3)
	{
		std::cerr << "\nUsage: onimesh <outputDir> <OniInputFiles>\n"
			<< "      outputDir - Output directory for data files\n"
			<< "  oniInputFiles - One or more .oni input files\n";
		exit(1);
	}

	// Reads oni input and exports data as excel docs
	onimesh::outputExcel(argc, argv);

    // Reads oni input and exports pcd files
    onimesh::outputPointCloud(argc, argv);

	// Uncomment the following line to test creating a dummy point cloud
	//onimesh::createDummyPointCloud();

	std::cout << "Onimesh output generation complete.\n";
	exit(0);
}